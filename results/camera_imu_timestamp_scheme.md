# Final camera / IMU timestamping scheme

## Scope

This document freezes the Stage 9 timestamp architecture for live OV9281 + Matek H743 Mono+IMU input.

## Common time domain

The common runtime time domain is Raspberry Pi `CLOCK_MONOTONIC`, represented in integer nanoseconds.

### Camera

The OV9281 USB UVC capture provides the native V4L2 timestamp in the monotonic/SOE domain. The raw timestamp is preserved for diagnostics.

For the currently validated capture configuration — OV9281 USB UVC, 640x480 MJPEG @ 120 FPS, current USB bridge/driver/timestamp semantics — the measured physical camera-to-IMU offset is compensated by shifting the camera timestamp forward by `+10.5 ms`:

`camera_corrected_ns = v4l2_timestamp_ns + 10,500,000 ns`

The sign was verified independently. A dedicated start/stop yaw test measured raw global offset `-10.55 ms`; applying camera `+10.5 ms` reduced the residual to approximately `-0.05 ms` in the sign verifier. A later compensated validation measured global residual `-0.150 ms` with correlation `-0.964`.

The `+10.5 ms` value is configuration-specific, not an intrinsic constant of the OV9281 sensor. It must be re-measured if the camera transport/interface, USB bridge, resolution, source FPS, capture mode or timestamp semantics change. In particular, switching from USB UVC to CSI requires a new offset validation before the correction is reused.

The implementation is centralized in `tools/camera_imu_timestamp_policy.hpp`.

### IMU

Matek H743 `HIGHRES_IMU.time_usec` is FC boot-relative time and is not used directly as an RPi timestamp.

MAVLink TIMESYNC samples continuously establish the affine FC-to-RPi mapping:

`t_rpi = rpi_ref + A * (t_fc - fc_ref)`

The reference-centered form is required to avoid loss of numerical precision. `A` is estimated from TIMESYNC data and must not be hardcoded because the measured FC/RPi clock-rate mismatch is approximately 2,000 ppm and changes slightly between runs.

Mapped IMU timestamps are expressed in the same RPi `CLOCK_MONOTONIC` nanosecond domain as corrected camera timestamps.

### TIMESYNC mapping validity and fail-safe rule

The live pipeline must explicitly track whether the FC-to-RPi affine mapping is valid.

Until a valid mapping has been established after startup, IMU samples must not be forwarded to Kimera as synchronized measurements. Likewise, if the mapping becomes stale or otherwise invalid, the pipeline must temporarily stop forwarding synchronized IMU data until a valid mapping is restored.

Serial receive time must never be substituted as a fallback IMU measurement timestamp. Such a fallback would silently convert UART/MAVLink transport latency into sensor timing error.

The exact online estimator/validity thresholds belong to the Stage 12 live implementation, but the fail-safe contract is fixed here: invalid TIMESYNC mapping means synchronization unavailable, not "use receive time instead".

## Camera continuity policy

The continuity guard operates on consecutive raw/source V4L2 frames at the native 120 FPS capture rate, before temporal decimation.

Consecutive source frames must satisfy:

1. `current_sequence == previous_sequence + 1`.
2. `current_corrected_timestamp_ns > previous_corrected_timestamp_ns`.
3. Corrected source-frame timestamp delta must not exceed `20 ms`.

The `20 ms` ceiling applies only to the 120 FPS source stream. It must not be applied to the approximately 30 FPS frames selected for Kimera, whose normal inter-frame interval is around `33.3 ms`.

After temporal selection, each Kimera frame keeps the actual corrected timestamp of its source V4L2 frame. No synthetic 30 FPS timestamps are generated. A selected-frame interval is therefore expected to be around `33.3 ms` and may vary according to the real source timestamps and selector decisions.

If a source continuity check fails, no visual motion increment may be formed across that source discontinuity. Tracking is reinitialized on the current valid source frame before further visual measurements are accepted.

The sequence check is mandatory and cannot be replaced by a timestamp-gap check. Validation found a real one-frame source loss where the observed timestamp delta was only `7.998 ms`; the loss was detected only by the V4L2 sequence discontinuity.

`tools/camera_timestamp_policy_check.cpp` validated the shared policy on the captured yaw dataset: 3620 frames, 1 source frame missing, 1 rejected pair, 0 corrected timestamp mismatches, `RESULT: PASS`.

## Transport and arrival timestamps

Camera receive time and IMU serial receive time are diagnostics only. They must not replace source timestamps for VIO synchronization.

A 120 s disk-backed diagnostic run demonstrated why: blocking MJPEG file I/O produced simultaneous camera and UART servicing stalls of approximately 1.6 s while the IMU source timestamp stream itself remained continuous. Repeating the run with MJPEG output in `/dev/shm` reduced camera delivery max to `12.304 ms` and IMU transport max to `4.501 ms`.

Therefore live VIO must use corrected/mapped source timestamps, not message arrival times.

## Runtime rules for live Kimera input

- Time unit: integer nanoseconds.
- Common domain: RPi `CLOCK_MONOTONIC`.
- Camera source timestamp: native V4L2 monotonic/SOE timestamp.
- Camera VIO timestamp: corrected source timestamp (`raw + 10.5 ms`) only for the currently validated USB UVC 640x480 MJPEG @ 120 FPS configuration.
- Recalibrate camera-to-IMU offset after camera interface/mode/timestamp-semantic changes, especially USB-to-CSI migration.
- Source continuity guard: run on consecutive 120 FPS V4L2 source frames; `sequence` contiguous, corrected `dt > 0`, corrected source `dt <= 20 ms`.
- Temporal decimation: select approximately 30 FPS frames only after source continuity validation; preserve each selected frame's actual corrected source timestamp; never synthesize fixed-rate timestamps.
- The source-stream `20 ms` threshold must not be applied to selected ~30 FPS Kimera frame intervals.
- IMU VIO timestamp: affine-mapped `HIGHRES_IMU.time_usec` using continuously maintained MAVLink TIMESYNC mapping.
- Forward synchronized IMU data only while the TIMESYNC affine mapping is valid; if it is not yet valid or becomes stale/invalid, synchronization is unavailable until recovery.
- Never fall back from FC source time to UART receive time for IMU sample timing.
- Preserve raw source and receive timestamps in diagnostic logs.
- Reject visual measurements across source camera discontinuities and reset tracking.
- Do not perform blocking disk I/O in the camera/UART servicing path; logging must be asynchronous or otherwise isolated from sensor acquisition.
- Visual-estimator outliers that occur with valid timestamps are a frontend-quality problem and are separate from timestamp-discontinuity handling.

## Stage 9 validation summary

- Common FC/RPi clock mapping validated in 30 s and 300 s runs.
- Physical camera-to-IMU offset measured by yaw correlation.
- Offset sign verified and `+10.5 ms` camera correction implemented for the current USB capture configuration.
- Compensated residual measured near zero.
- Camera and IMU jitter measured over 120 s.
- Large disk-I/O-induced diagnostic stalls isolated from source timestamp quality.
- Runtime camera continuity policy implemented and independently validated with `RESULT: PASS`.
- Source 120 FPS continuity and selected ~30 FPS VIO timing are explicitly separated.
- TIMESYNC mapping validity has a fail-safe contract: no receive-time fallback.

This scheme is the timestamp contract to be used when implementing Stage 12 live Mono+IMU input.