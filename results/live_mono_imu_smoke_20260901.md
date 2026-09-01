# First live Mono + IMU smoke test — 2026-09-01

## Result

**PASS**

The first native live JT-Zero Mono+IMU pipeline on Raspberry Pi 5 ran successfully with the real OV9281 camera and Matek H743 `HIGHRES_IMU` stream feeding Kimera-VIO.

This is a smoke/integration result. It proves that the live data path, timestamp mapping and Kimera backend operate together; it is not yet the final stand-still drift qualification or a complete accuracy validation.

## Runtime configuration

- Camera: OV9281 USB UVC, `/dev/video0`
- Source camera mode: 640x480 MJPEG @ 120 FPS
- Selected VIO camera stream: approximately 30 FPS, preserving actual corrected V4L2 source timestamps
- Camera timestamp correction: JT-Zero validated USB UVC policy (`+10.5 ms` physical offset compensation)
- IMU: Matek H743 `HIGHRES_IMU`, requested 200 Hz over `/dev/ttyAMA0`
- IMU timestamps: `HIGHRES_IMU.time_usec` mapped into Raspberry Pi `CLOCK_MONOTONIC` by online MAVLink TIMESYNC affine mapping
- Kimera internal IMU-rate time alignment: disabled
- Kimera initialization: automatic IMU initialization (`autoInitialize: 1`)
- Camera/IMU extrinsics: `params/JTZeroMono/LeftCameraParams.yaml`, based on accepted Stage 10 intrinsics and Stage 11 `T_BC`

## Final counters

```text
raw camera frames:      3618
rejected raw pairs:     1
selected camera frames: 847
decoded frames:         847
IMU received:           6231
IMU fed to Kimera:      5606
IMU skipped mapping:    625
TIMESYNC samples:       297
mapping valid:          yes
mapping drift ppm:      2034.997
backend outputs:        124
RESULT: PASS
```

The 625 skipped IMU samples occurred while the online TIMESYNC mapping was not yet valid. The runtime deliberately does not fall back to UART receive time.

The single rejected raw camera pair is consistent with the source-continuity safeguard: every raw 120 FPS V4L2 frame is checked before temporal selection.

## Live backend observation

Kimera produced backend states continuously from keyframe `kf=0` through `kf=123` during the run.

First state:

```text
kf=0
P=[0.0000,0.0000,0.0000]
V=[0.0000,0.0000,0.0000]
RPYdeg=[-174.1984,1.2621,-24.5266]
```

Last state:

```text
kf=123
P=[0.0040,-0.0013,-0.0000]
V=[-0.0003,-0.0008,-0.0005]
RPYdeg=[-174.2857,1.2688,-25.8033]
```

Over this approximately 30 s smoke run, the final position displacement from the initialized origin was about `4.21 mm`. This number is useful as an initial sanity check only; a dedicated longer stand-still logger/test is required before claiming a drift metric.

The absolute Euler angles are not interpreted as an external attitude reference because the live world frame is initialized by Kimera from IMU rather than from ground truth. A dedicated yaw/attitude-change test remains required.

## Non-fatal visualization messages

The run emitted `QueueSynchronizer` timeouts for `visualizer_mesher_queue` and VTK edge-extractor informational messages. Kimera explicitly switched from unavailable mesh visualization to point-cloud visualization. These messages did not stop the pipeline and the run completed with `RESULT: PASS`.

For subsequent quantitative/headless tests the visualizer should be disabled to remove unnecessary VTK/visualization work and log noise.

## What this validates

Validated by this run:

- live OV9281 frames are accepted by Kimera-VIO;
- live `HIGHRES_IMU` measurements are accepted by Kimera-VIO;
- the previously defined common-clock timestamp contract operates in the live pipeline;
- Kimera performs IMU initialization without EuRoC ground truth;
- the backend produces continuous live pose and velocity outputs;
- accepted camera/IMU orientation/extrinsics do not cause an immediate catastrophic live-pipeline failure.

Still open:

- dedicated 60–120 s stand-still drift statistics;
- explicit quantitative end-to-end extrinsics motion validation;
- input/output logging suitable for offline trajectory analysis;
- known-distance translation test;
- return-to-origin test;
- yaw-only test;
- combined-motion test;
- final Matek/ArduPilot IMU noise-density/random-walk characterization.
