# Final camera / IMU timestamping scheme

## Scope

This document freezes the Stage 9 timestamp architecture for live OV9281 + Matek H743 Mono+IMU input.

## Common time domain

The common runtime time domain is Raspberry Pi `CLOCK_MONOTONIC`, represented in integer nanoseconds.

### Camera

The OV9281 USB UVC capture provides the native V4L2 timestamp in the monotonic/SOE domain. The raw timestamp is preserved for diagnostics.

A measured physical camera-to-IMU offset is compensated by shifting the camera timestamp forward by a fixed `+10.5 ms`:

`camera_corrected_ns = v4l2_timestamp_ns + 10,500,000 ns`

The sign was verified independently. A dedicated start/stop yaw test measured raw global offset `-10.55 ms`; applying camera `+10.5 ms` reduced the residual to approximately `-0.05 ms` in the sign verifier. A later compensated validation measured global residual `-0.150 ms` with correlation `-0.964`.

The implementation is centralized in `tools/camera_imu_timestamp_policy.hpp`.

### IMU

Matek H743 `HIGHRES_IMU.time_usec` is FC boot-relative time and is not used directly as an RPi timestamp.

MAVLink TIMESYNC samples continuously establish the affine FC-to-RPi mapping:

`t_rpi = rpi_ref + A * (t_fc - fc_ref)`

The reference-centered form is required to avoid loss of numerical precision. `A` is estimated from TIMESYNC data and must not be hardcoded because the measured FC/RPi clock-rate mismatch is approximately 2,000 ppm and changes slightly between runs.

Mapped IMU timestamps are expressed in the same RPi `CLOCK_MONOTONIC` nanosecond domain as corrected camera timestamps.

## Camera continuity policy

Before any inter-frame visual measurement is formed, consecutive source frames must pass the shared runtime continuity policy:

1. `current_sequence == previous_sequence + 1`.
2. `current_corrected_timestamp_ns > previous_corrected_timestamp_ns`.
3. Corrected timestamp delta must not exceed `20 ms` for the 120 FPS source stream.

If any check fails, the inter-frame visual pair is invalid and tracking must be reinitialized on the current frame. No visual rotation/translation increment may be formed across the discontinuity.

The sequence check is mandatory and cannot be replaced by a timestamp-gap check. Validation found a real one-frame source loss where the observed timestamp delta was only `7.998 ms`; the loss was detected only by the V4L2 sequence discontinuity.

`tools/camera_timestamp_policy_check.cpp` validated the shared policy on the captured yaw dataset: 3620 frames, 1 source frame missing, 1 rejected pair, 0 corrected timestamp mismatches, `RESULT: PASS`.

## Transport and arrival timestamps

Camera receive time and IMU serial receive time are diagnostics only. They must not replace source timestamps for VIO synchronization.

A 120 s disk-backed diagnostic run demonstrated why: blocking MJPEG file I/O produced simultaneous camera and UART servicing stalls of approximately 1.6 s while the IMU source timestamp stream itself remained continuous. Repeating the run with MJPEG output in `/dev/shm` reduced camera delivery max to `12.304 ms` and IMU transport max to `4.501 ms`.

Therefore live VIO must use corrected/mapped source timestamps, not message arrival times.

## Runtime rules for live Kimera input

- Time unit: integer nanoseconds.
- Common domain: RPi `CLOCK_MONOTONIC`.
- Camera VIO timestamp: corrected V4L2 source timestamp (`raw + 10.5 ms`).
- IMU VIO timestamp: affine-mapped `HIGHRES_IMU.time_usec` using continuously maintained MAVLink TIMESYNC mapping.
- Preserve raw source and receive timestamps in diagnostic logs.
- Reject visual pairs across camera sequence or timestamp discontinuities and reset tracking.
- Do not perform blocking disk I/O in the camera/UART servicing path; logging must be asynchronous or otherwise isolated from sensor acquisition.
- Visual-estimator outliers that occur with valid timestamps are a frontend-quality problem and are separate from timestamp-discontinuity handling.

## Stage 9 validation summary

- Common FC/RPi clock mapping validated in 30 s and 300 s runs.
- Physical camera-to-IMU offset measured by yaw correlation.
- Offset sign verified and `+10.5 ms` camera correction implemented.
- Compensated residual measured near zero.
- Camera and IMU jitter measured over 120 s.
- Large disk-I/O-induced diagnostic stalls isolated from source timestamp quality.
- Runtime camera continuity policy implemented and independently validated with `RESULT: PASS`.

This scheme is the timestamp contract to be used when implementing Stage 12 live Mono+IMU input.