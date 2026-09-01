# Live Mono + IMU 300 mm HUD — trial 1 — 2026-09-01

## Result

**PIPELINE PASS / MEASUREMENT INVALID FOR SCALE CALIBRATION**

The integrated low-load HUD remained live and the Kimera backend continued updating throughout the run. However, the measured displacement is grossly inconsistent with the mechanically commanded 300 mm translation.

## Runtime counters

```text
raw camera frames:     4227
rejected raw pairs:    4
selected frames:       995
decoded frames:        995
IMU received:          7298
IMU fed to Kimera:     6580
TIMESYNC samples:      344
mapping valid:         yes
mapping drift ppm:     2027.0075
backend states:        144
START avg states:      22
END avg states:        22
```

## Measurement

```text
measured dP:            [0.445640,-0.246368,0.000614] m
3D measured distance:   509.207659 mm
expected distance:      300.000000 mm
absolute error:         +209.207659 mm
relative error:         +69.735886 %
scale measured/true:    1.697359
dRoll:                  -0.897349 deg
dPitch:                 -1.838475 deg
dYaw:                   +7.885955 deg
END mean |V|:           1.832756 mm/s
PIPELINE RESULT:        PASS
```

No explicit VIO jump event was reported. The estimate increased smoothly enough to avoid the 80 mm/keyframe jump detector, but accumulated a very large translational over-estimate while attitude changed substantially, especially yaw.

## Interpretation

This result is not compatible with a simple constant scale-factor error. Earlier valid 500 mm trials produced both ~0.91 and ~0.98 scale, while this 300 mm run produces ~1.70. The dominant new feature is the relatively large orientation change during the manual translation, particularly +7.89 deg yaw and -1.84 deg pitch.

The integrated HUD itself is now verified to remain responsive: backend states reached 144 and continued incrementing through the run, unlike the first high-load HUD build that stalled around KF 19.

The result strengthens the hypothesis that the current live Mono+IMU estimate is strongly coupled to rotational/manual motion and/or to the camera-IMU rotational/extrinsic/IMU model under combined translation and rotation. Do not apply a global scale multiplier from this run.

## Next diagnostic

Before another scale run, use the HUD with flight-controller attitude deltas displayed directly from MAVLink ATTITUDE/HIGHRES_IMU rather than relying only on backend RPY. The operator should minimize raw platform rotation during motion. The next known-distance test should be repeated at 300 mm with a deliberately constrained yaw/pitch envelope and the run should be rejected automatically if rotation exceeds a documented threshold.
