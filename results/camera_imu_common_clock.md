# Camera + IMU common clock validation

## Scope

Partial validation of Stage 9: camera and Matek H743 IMU timestamps are converted to one Raspberry Pi `CLOCK_MONOTONIC` time domain. This does **not** yet determine the physical camera-to-IMU time offset.

## Clock domains

Camera timestamps come from the OV9281 USB UVC V4L2 capture buffer. The tested buffers are marked `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` and `V4L2_BUF_FLAG_TSTAMP_SRC_SOE`, so the V4L2 timestamp is already expressed in the Raspberry Pi monotonic clock domain. The physical meaning of the USB/UVC SOE timestamp relative to sensor exposure is still subject to the later motion-correlation test.

The Matek H743 `HIGHRES_IMU.time_usec` timestamp belongs to the flight-controller clock and therefore requires clock-rate and offset conversion before it can be compared with the camera timestamp.

The tested affine mapping is:

```text
t_rpi = RPi_ref + A * (t_fc - FC_ref)
```

A constant offset alone is invalid because the FC and RPi clocks have a measurable rate mismatch.

## Long TIMESYNC validation

A 300 s combined TIMESYNC + HIGHRES_IMU diagnostic produced:

```text
TIMESYNC requests/responses: 3000
TIMESYNC good samples:       2997
RTT min:                     1.459 ms
RTT median:                  2.143 ms
RTT p95:                     4.041 ms
RTT max:                     5.146 ms
RPi/FC ratio:                1.002066836
clock-rate difference:       2066.836 ppm
constant-offset error / 10m: 1240.101 ms
```

This independently confirms that a constant clock offset is not sufficient.

## Native single-process C++ validation

A native C++ logger simultaneously captured:

- OV9281 `/dev/video0`, 640x480 MJPEG @ 120 FPS;
- Matek H743 `HIGHRES_IMU` @ 200 Hz over `/dev/ttyAMA0` @ 460800 baud;
- MAVLink TIMESYNC @ 10 Hz;
- Raspberry Pi receive timestamps from `CLOCK_MONOTONIC`.

30 s result:

```text
A (RPi/FC):                 1.002061378243
clock-rate difference:      2061.378 ppm
TIMESYNC good:              300 / 300
TIMESYNC RTT median:        2.492 ms
TIMESYNC RTT p95:           2.945 ms

Camera frames:              3618
Camera source drops:        1
Camera delivery median:     8.029 ms
Camera delivery p95:        8.060 ms
Camera delivery p99:        8.081 ms
Camera delivery max:        8.108 ms
Camera timestamp dt median: 8.004 ms
Camera timestamp dt p95:    11.961 ms

IMU samples:                5987
IMU transport median:       0.651 ms
IMU transport p95:          1.144 ms
IMU transport p99:          1.223 ms
IMU transport min:          0.534 ms
IMU transport max:          1.700 ms
```

The 30 s native estimate of 2061.378 ppm agrees closely with the 300 s estimate of 2066.836 ppm. The two independent measurements therefore support the affine FC-to-RPi clock conversion.

The native logger output is `/home/vio/camera_imu_timesync.csv` on the test Raspberry Pi.

## Conclusion

The camera and IMU timestamps can now be represented on a common Raspberry Pi `CLOCK_MONOTONIC` time scale. The Stage 9 checklist item `Привести camera и IMU timestamps к общей временной шкале` is therefore complete.

Stage 9 as a whole remains incomplete. In particular, the approximately 8.03 ms camera userspace delivery delay and approximately 0.65 ms IMU userspace transport delay must **not** be subtracted and treated as the physical camera-to-IMU offset. The physical offset must be measured separately by correlating camera-observed angular motion with IMU gyro data during a controlled yaw test.
