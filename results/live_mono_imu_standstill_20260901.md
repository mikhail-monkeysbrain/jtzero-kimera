# Live Mono + IMU stand-still test — 2026-09-01

## Result

**PIPELINE RESULT: PASS**

A dedicated 120 s motionless bench run was completed using the native live OV9281 + Matek H743 HIGHRES_IMU → Kimera-VIO pipeline.

This establishes the first quantitative stand-still baseline for the current JT-Zero live VIO configuration. Drift thresholds were intentionally not frozen before this measurement.

## Runtime counters

```text
requested duration:    120.0000 s
raw camera frames:     14475
rejected raw pairs:    3
selected frames:       3558
decoded frames:        3558
IMU received:          24195
IMU fed to Kimera:     23551
IMU skipped mapping:   644
TIMESYNC samples:      1195
mapping valid:         yes
mapping drift ppm:     2029.142
standstill states:     517
backend duration:      117.901538 s
```

The timestamp mapping remained valid. IMU samples observed before valid mapping were deliberately skipped; UART receive-time fallback was not used.

## Stand-still trajectory metrics

```text
final dP:              [0.010165,-0.012522,-0.002306] m
final |dP|:            16.293034 mm
position RMS origin:   5.556669 mm
position max origin:   18.385700 mm
velocity RMS:          1.501836 mm/s
velocity max:          10.631633 mm/s
roll drift:            +0.173704 deg
pitch drift:           -0.044186 deg
yaw drift:             -1.043353 deg
```

Backend CSV produced by the run:

```text
/home/vio/jtzero_live_standstill.csv
```

## Interpretation

Position behavior is stable at the centimeter scale over this approximately 118 s backend interval: final displacement is 16.3 mm, RMS displacement from the initialized origin is 5.56 mm, and maximum excursion is 18.39 mm.

Velocity remains close to zero: RMS 1.50 mm/s. The 10.63 mm/s maximum is retained as an observed transient and should not be hidden by averaging.

Roll and pitch remain comparatively stable. Yaw shows a monotonic-scale drift of about -1.04 deg over the run. This is the main stand-still attitude effect to investigate when the dedicated yaw test is performed. The absolute initial Euler angles are not treated as an external attitude reference because Kimera initializes its world frame from the IMU.

Camera source continuity remained healthy: only 3 raw pairs were rejected out of 14475 raw frames. The selected stream produced 3558 decoded frames, approximately the intended 30 FPS class.

## What this validates

- live pipeline remains operational for a two-minute-class stationary run;
- camera and IMU common-clock mapping remains valid throughout the run;
- live pose does not exhibit catastrophic translational divergence while stationary;
- backend state logging is sufficient to compute quantitative stand-still position, velocity and attitude drift metrics;
- the stand-still test item in Stage 12 is complete for this configuration.

## Remaining Stage 12 tests

The next primary experiment is a known linear displacement. Use a mechanically measured 500 mm horizontal trajectory, with pauses before and after motion, and preserve the same camera/IMU configuration. Subsequent tests remain return-to-origin, yaw-only, and combined motion.

Do not interpret this stand-still result as a known-distance scale validation: the rig did not move during this test.
