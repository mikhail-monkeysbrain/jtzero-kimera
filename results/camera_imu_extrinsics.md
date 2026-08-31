# Camera ↔ IMU extrinsics calibration

## Stage 11 status

This document records the experimentally verified coordinate-frame convention used for OV9281 ↔ Matek H743 extrinsic calibration.

At this point only the coordinate frames are fixed. Camera↔IMU rotation and translation are not yet calibrated and must not be inferred from the approximate physical mounting.

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

The camera is physically mounted looking downward, so `+C_Z` is approximately aligned with vehicle/body down (`+B_Z`). This approximate mounting relation is descriptive only and must not be used as the calibrated extrinsic rotation.

## Transform convention

For the remainder of Stage 11:

- `B` denotes the experimentally verified IMU/body FRD frame above.
- `C` denotes the OpenCV OV9281 camera frame above.
- Exact Camera↔IMU rotation will be estimated from synchronized camera and gyro motion.
- Exact Camera↔IMU translation will be measured/calibrated separately.
- The final matrix direction and serialization into Kimera `T_BS` must be verified against Kimera/GTSAM conventions before the calibration file is written.

No numeric extrinsic transform is claimed by this document yet.

## Timing configuration used during verification

The tests used the Stage 9 synchronized logger configuration:

- OV9281 USB UVC 640x480 MJPEG @ 120 FPS
- camera timestamp correction `+10.5 ms`
- `HIGHRES_IMU` requested @ 200 Hz
- FC clock mapped to RPi `CLOCK_MONOTONIC` by MAVLink TIMESYNC affine mapping

The 45 s axis-identification run contained 5429 camera frames, 0 camera source drops, 8982 IMU samples and 444/444 good TIMESYNC exchanges.

The dedicated sign run used the same sensor/timestamp configuration. The final dedicated nose-right yaw test produced `+0.662092 rad` (`+37.94 deg`) on `zgyro`, resolving the previously inconclusive short yaw interval.

## Result

Coordinate systems are fixed and experimentally verified sufficiently to proceed with Camera↔IMU rotation calibration.
