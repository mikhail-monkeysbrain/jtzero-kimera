# Camera ↔ IMU extrinsics calibration

## Stage 11 status

This document records the experimentally verified coordinate-frame convention and the current Camera↔IMU extrinsic state for OV9281 ↔ Matek H743.

After the mechanical stand rebuild, the pre-rebuild rotation became obsolete. The rebuilt geometry has now been independently recalibrated and validated. The current Camera↔IMU rotation is fixed. The mechanical camera position relative to the FC center is measured as approximately 55 mm below the FC, with no intentional X/Y offset. Experimental validation of the translation and the complete 6-DoF transform is still open.

Kimera `T_BS` serialization direction has now been checked directly against the exact Kimera-VIO commit used by this project (`ce8c59b7b273ab5ac29db7e5572e1623760e19c7`).

## IMU / body frame B

Source: Matek H743 / ArduPilot `HIGHRES_IMU` (MAVLink message 105).

The correspondence between physical vehicle rotations and `xgyro/ygyro/zgyro` was verified experimentally with isolated roll, pitch and yaw motions.

Axis mapping test:

- physical ROLL → `xgyro` dominant, RMS 0.2456 rad/s; parasitic `ygyro` 0.0263, `zgyro` 0.0720 rad/s
- physical PITCH → `ygyro` dominant, RMS 0.3584 rad/s; parasitic `xgyro` 0.0408, `zgyro` 0.0791 rad/s
- physical YAW → `zgyro` dominant, RMS 0.3071 rad/s; parasitic `xgyro` 0.0200, `ygyro` 0.0285 rad/s

No axis permutation was observed.

Sign tests:

- right side down → `+xgyro`; integrated angle `+12.77 deg`
- nose down → `-ygyro`; integrated angle `-13.02 deg`
- nose right → `+zgyro`; dedicated yaw verification integrated `+37.94 deg`

Therefore the body/IMU frame used by the received `HIGHRES_IMU` data is fixed as right-handed FRD:

- `+B_X` = Forward
- `+B_Y` = Right
- `+B_Z` = Down

The corresponding positive rotations follow the right-hand rule:

- `+ω_Bx` = right side down
- `+ω_By` = nose up
- `+ω_Bz` = nose right

These statements describe the actual MAVLink `HIGHRES_IMU` stream in the tested Matek H743 / ArduPilot configuration; they are not assumptions based only on board silkscreen or nominal sensor orientation.

## Camera frame C

The OV9281 camera frame is fixed to the standard OpenCV camera convention:

- `+C_X` = right in the image
- `+C_Y` = down in the image
- `+C_Z` = forward along the optical axis, away from the camera

The camera is physically mounted looking downward, so `+C_Z` is expected to be approximately aligned with vehicle/body down (`+B_Z`). The post-rebuild calibration confirms this.

## Transform convention

For Stage 11:

- `B` denotes the experimentally verified IMU/body FRD frame above.
- `C` denotes the OpenCV OV9281 camera frame above.
- `R_CB` maps a body-frame angular-velocity vector into the camera frame: `ω_C = R_CB ω_B`.
- `R_BC = R_CB^T` maps camera-frame vectors into the body frame.
- `t_BC` is the camera-origin position expressed in the body frame.
- The homogeneous transform `T_BC = [R_BC | t_BC]` maps camera-frame points into the body frame: `p_B = R_BC p_C + t_BC`.

## Kimera `T_BS` direction verification

The exact Kimera-VIO source used by this project parses camera `T_BS` through `CameraParams::parseBodyPoseCam(...)` into `body_Pose_cam_`. The source comment is `Camera pose wrt body.`

The shipped Kimera parameter files label `T_BS` as `Sensor extrinsics wrt. the body-frame.` The EuRoC `LeftCameraParams.yaml` contains the standard EuRoC body-to-camera extrinsic in the same matrix slot.

Therefore for camera sensor `S=C`, Kimera expects the camera pose in the body frame:

`T_BS = T_BC`

with

`p_B = R_BC p_C + t_BC`.

For this project the rotation to serialize into camera `T_BS` is therefore `R_BC`, not `R_CB`.

## Timing configuration used during post-rebuild rotation calibration

Both post-rebuild rotation runs used the Stage 9 synchronized logger configuration:

- OV9281 USB UVC 640x480 MJPEG @ 120 FPS
- camera timestamp correction `+10.5 ms`
- `HIGHRES_IMU` requested @ 200 Hz
- FC clock mapped to RPi `CLOCK_MONOTONIC` by MAVLink TIMESYNC affine mapping
- camera MJPEG and camera-index output written to `/dev/shm`
- raw camera discontinuities explicitly rejected before visual-pair formation

The independent validation run additionally used the state-driven guided logger with a 2 s stable attitude-zero acquisition. Roll, pitch and yaw targets were evaluated relative to the physical stand zero instead of absolute ArduPilot ATTITUDE angles.

## Post-rebuild calibration run

Logger quality:

- camera frames: 7238
- camera source drops: 2
- camera delivery median / p95 / p99 / max: 8.029 / 8.061 / 8.081 / 8.116 ms
- IMU samples: 11977
- IMU transport median / p95 / p99 / max: 0.698 / 1.216 / 1.349 / 2.465 ms
- TIMESYNC good: 596/596
- TIMESYNC affine clock ratio `A = 1.002023254479`
- drift: 2023.254 ppm

Rotation-calibrator statistics:

- raw camera frames: 7238
- raw discontinuities: 2
- pose attempts: 1810
- valid ChArUco poses: 1085
- detection failures: 725
- reprojection rejects: 0
- candidate pairs: 949
- robust inliers: 915
- continuity rejects: 23
- speed rejects: 112
- gyro-vector residual median / p95: 0.049816667 / 0.120717404 rad/s
- direction error median / p95: 4.380252371 / 14.477215060 deg

Measured rotation:

```text
R_CB =
 0.007363966 -0.999546758  0.029189919
 0.999967841  0.007268069 -0.003390018
 0.003176327  0.029213944  0.999568135

RPY_CB = (1.674082316, -0.181990427, 89.578069893) deg
```

Inverse:

```text
R_BC =
 0.007363966  0.999967841  0.003176327
-0.999546758  0.007268069  0.029213944
 0.029189919 -0.003390018  0.999568135

RPY_BC = (-0.194316878, -1.672696736, -89.577892151) deg
```

`det(R_CB) = 1.0`; orthogonality error was 0 at printed precision.

## Independent post-rebuild validation run

This run used the guided state-driven logger after fixing its zero-reference logic.

Logger quality:

- camera frames: 11797
- camera source drops: 3
- camera delivery median / p95 / p99 / max: 8.032 / 8.350 / 9.156 / 12.859 ms
- IMU samples: 19521
- IMU transport median / p95 / p99 / max: 0.638 / 1.208 / 2.119 / 9.739 ms
- TIMESYNC good: 972/972
- TIMESYNC RTT median / p95: 2.523 / 3.393 ms
- TIMESYNC affine clock ratio `A = 1.002036512495`
- drift: 2036.512 ppm

Rotation-calibrator statistics:

- raw camera frames: 11797
- raw discontinuities: 3
- pose attempts: 2950
- valid ChArUco poses: 2656
- decode failures: 0
- detection failures: 294
- reprojection rejects: 0
- pose reprojection median / p95: 0.613590566 / 0.741702464 px
- candidate pairs: 685
- robust inliers: 599
- continuity rejects: 19
- dt rejects: 0
- IMU coverage rejects: 1
- speed rejects: 1950
- gyro-vector residual median / p95: 0.044848426 / 0.099041568 rad/s
- direction error median / p95: 4.039750296 / 15.866769103 deg

Independent measured rotation:

```text
R_CB(validation) =
 0.009367371 -0.999326771  0.035471918
 0.999954838  0.009418381  0.001271215
-0.001604448  0.035458408  0.999369865

R_BC(validation) =
 0.009367371  0.999954838 -0.001604448
-0.999326771  0.009418381  0.035458408
 0.035471918  0.001271215  0.999369865

RPY_BC(validation) = (0.072881157, -2.032817638, -89.462943333) deg
```

The geodesic relative rotation between the two post-rebuild `R_BC` estimates is approximately `0.465 deg`.

This is well inside the Stage 11 acceptance target of approximately 1–2 degrees for an independently repeated calibration.

## Superseded pre-rebuild calibration

The previous stand geometry produced a Camera↔IMU rotation approximately 180 degrees away in yaw from the rebuilt stand. That matrix was valid only for the previous mechanical assembly and must not be used for the current hardware geometry.

The post-rebuild calibration changed the yaw by approximately 180 degrees, which is consistent with the actual mechanical rebuild. All pre-rebuild `R_BC` values in repository history are therefore superseded.

## Accepted Camera↔IMU rotation for current stand

The independent post-rebuild validation estimate is accepted as the current Camera↔IMU rotation:

```text
R_CB =
 0.009367371 -0.999326771  0.035471918
 0.999954838  0.009418381  0.001271215
-0.001604448  0.035458408  0.999369865

R_BC =
 0.009367371  0.999954838 -0.001604448
-0.999326771  0.009418381  0.035458408
 0.035471918  0.001271215  0.999369865
```

Acceptance basis:

- two independent post-rebuild synchronized captures
- geodesic disagreement only approximately 0.465 deg
- proper rotation matrices with determinant 1
- camera optical `+C_Z` maps approximately to `[-0.0016, +0.0355, +0.9994]_B`, consistent with the downward-looking mount
- no storage-induced camera/UART stalls
- raw camera discontinuities explicitly rejected
- second run used relative attitude targets tied to the physical stand zero

## Translation geometry for current stand

After the rebuild the camera is mechanically centered below the FC.

Current measured geometry, expressed as camera origin in body FRD coordinates:

```text
t_BC = [0.000, 0.000, 0.055] m
```

Conservative mechanical uncertainty used for Stage 11 planning:

```text
X = 0 ± 5 mm
Y = 0 ± 5 mm
Z = 55 ± 3 mm
```

A centered-yaw visual lever-arm validator was also run. It did not validate translation because manual pivot/reposition error was comparable to or larger than the expected X/Y lever arm. The run produced about 7–15 mm camera repositioning when returning to nominally similar orientations, so its fitted `Y ≈ -18.9 mm` must not be interpreted as a physical sensor offset.

The mechanical `t_BC` measurement is therefore the current translation candidate, but Camera↔IMU translation remains experimentally unvalidated.

## Candidate Kimera transform for current geometry

With the accepted post-rebuild rotation and the mechanical translation candidate, the current candidate camera transform is:

```text
T_BS = T_BC =
[ 0.009367371   0.999954838  -0.001604448   0.000 ]
[-0.999326771   0.009418381   0.035458408   0.000 ]
[ 0.035471918   0.001271215   0.999369865   0.055 ]
[ 0.000000000   0.000000000   0.000000000   1.000 ]
```

This matrix is not yet promoted to the final extrinsics file because the translation and complete axis/sign transform validation are still open.

## Result

For the current rebuilt stand:

- IMU/body coordinate frame is experimentally fixed as FRD.
- OpenCV camera coordinate frame is fixed.
- Camera↔IMU rotation is recalibrated and independently validated to approximately 0.465 deg repeatability.
- The accepted current rotation is the post-rebuild validation `R_CB` / `R_BC` pair above.
- Kimera camera `T_BS` direction is verified: serialize `T_BC`, therefore use `R_BC` and camera position `t_BC` in body coordinates.
- Mechanical translation candidate is `t_BC = [0, 0, 0.055] m`.
- Translation experimental validation, full axis/sign validation of the 6-DoF transform, and final extrinsics serialization remain open.
