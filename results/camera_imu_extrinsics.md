# Camera ↔ IMU extrinsics calibration

## Stage 11 status

This document records the experimentally verified coordinate-frame convention and the calibrated Camera↔IMU rotation for OV9281 ↔ Matek H743 extrinsic calibration.

Coordinate frames and Camera↔IMU rotation are now fixed. Camera↔IMU translation is not yet calibrated. The final Kimera `T_BS` serialization direction must still be verified before writing the complete extrinsic calibration file.

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

The camera is physically mounted looking downward, so `+C_Z` is expected to be approximately aligned with vehicle/body down (`+B_Z`). The calibrated rotation below confirms this independently from synchronized visual and gyro motion.

## Transform convention

For Stage 11 rotation calibration:

- `B` denotes the experimentally verified IMU/body FRD frame above.
- `C` denotes the OpenCV OV9281 camera frame above.
- `R_CB` maps a body-frame angular-velocity vector into the camera frame: `ω_C = R_CB ω_B`.
- `R_BC = R_CB^T` is the inverse rotation and maps camera-frame vectors into the body frame.
- Camera↔IMU translation will be measured/calibrated separately.
- The final matrix direction and serialization into Kimera `T_BS` must be verified against Kimera/GTSAM conventions before the complete calibration file is written.

## Timing configuration used during rotation calibration

Both rotation runs used the Stage 9 synchronized logger configuration:

- OV9281 USB UVC 640x480 MJPEG @ 120 FPS
- camera timestamp correction `+10.5 ms`
- `HIGHRES_IMU` requested @ 200 Hz
- FC clock mapped to RPi `CLOCK_MONOTONIC` by MAVLink TIMESYNC affine mapping
- camera MJPEG and camera-index output written to `/dev/shm` to avoid storage-induced camera/UART stalls

### Calibration run

Logger quality:

- camera frames: 7238
- camera source drops: 0
- camera delivery median / max: 8.029 / 8.420 ms
- IMU samples: 11975
- IMU transport median / p99 / max: 0.675 / 1.229 / 2.274 ms
- TIMESYNC good: 596/596
- TIMESYNC affine clock ratio `A = 1.002046381109`

Rotation-calibrator statistics:

- valid ChArUco poses: 1784 / 1810 attempts
- candidate pairs: 646
- robust inliers: 579
- pose reprojection median / p95: 0.525164 / 0.664801 px
- gyro-vector residual median / p95: 0.043923 / 0.098450 rad/s
- direction error median / p95: 3.499315 / 13.634251 deg

Measured rotation:

```text
R_CB =
-0.010624404   0.995285416   0.096405718
-0.998107308  -0.016395603   0.059270445
 0.060571639  -0.095593539   0.993575841

RPY_CB = (-5.495604269, -3.472624953, -90.609864790) deg
```

Inverse:

```text
R_BC =
-0.010624404  -0.998107308   0.060571639
 0.995285416  -0.016395603  -0.095593539
 0.096405718   0.059270445   0.993575841

RPY_BC = (3.413857845, -5.532232935, 90.611593783) deg
```

`det(R_CB) = 1.0`; reported orthogonality error was 0 at printed precision.

### Independent validation run

The second 60 s run used a different motion ordering and was processed independently.

Logger quality:

- camera frames: 7238
- camera source drops: 2
- camera delivery median / max: 8.029 / 8.343 ms
- IMU samples: 11976
- IMU transport median / p99 / max: 0.499 / 1.049 / 1.870 ms
- TIMESYNC good: 592/592
- TIMESYNC affine clock ratio `A = 1.002047409444`

Rotation-calibrator statistics:

- valid ChArUco poses: 1810 / 1810 attempts
- candidate pairs: 839
- robust inliers: 759
- continuity rejects: 2
- pose reprojection median / p95: 0.542697 / 0.699334 px
- gyro-vector residual median / p95: 0.045341 / 0.099169 rad/s
- direction error median / p95: 3.837891 / 13.433409 deg

Independent measured rotation:

```text
R_CB(validation) =
-0.009841887   0.996767522   0.079734833
-0.997747339  -0.015080480   0.065366862
 0.066358004  -0.078911884   0.994670563

R_BC(validation) =
-0.009841887  -0.997747339   0.066358004
 0.996767522  -0.015080480  -0.078911884
 0.079734833   0.065366862   0.994670563

RPY_BC(validation) = (3.759905826, -4.573324098, 90.565708918) deg
```

The relative rotation between the primary and validation `R_BC` estimates,

`R_delta = R_BC(validation) * R_BC(calibration)^T`,

has rotation angle approximately `1.019 deg`.

This is within the Stage 11 acceptance target of approximately 1–2 degrees for an independently repeated run.

The physical-axis sanity check is also consistent: in the independent run camera optical `+C_Z` maps to approximately `[+0.0664, -0.0789, +0.9947]_B`, so the downward-looking optical axis is almost aligned with body `+B_Z` as expected.

## Accepted Camera↔IMU rotation

The primary calibration estimate is accepted as the Stage 11 Camera↔IMU rotation:

```text
R_CB =
-0.010624404   0.995285416   0.096405718
-0.998107308  -0.016395603   0.059270445
 0.060571639  -0.095593539   0.993575841

R_BC =
-0.010624404  -0.998107308   0.060571639
 0.995285416  -0.016395603  -0.095593539
 0.096405718   0.059270445   0.993575841
```

Acceptance basis:

- two independent 60 s captures
- independent motion ordering in the validation run
- relative 3D rotation disagreement approximately 1.019 deg
- proper rotation matrices with determinant 1
- axis orientation physically consistent with the downward camera mount
- no camera/IMU transport stalls in either accepted run
- raw camera discontinuities explicitly rejected by the calibrator

## Result

Camera and IMU coordinate systems are fixed and experimentally verified.

Camera↔IMU rotation is calibrated and independently validated. The accepted primary estimate is the `R_CB` / `R_BC` pair above.

Camera↔IMU translation remains uncalibrated. Full axis/sign validation of the complete 6-DoF extrinsic transform and final Kimera `T_BS` serialization remain open until translation is obtained.
