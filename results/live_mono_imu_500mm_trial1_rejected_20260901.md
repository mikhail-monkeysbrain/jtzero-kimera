# Live Mono + IMU 500 mm — trial 1, rejected — 2026-09-01

## Pipeline result

**PASS**

The live camera/IMU/Kimera pipeline operated normally during the complete guided run.

```text
raw camera frames:     4221
rejected raw pairs:    1
selected frames:       998
decoded frames:        998
IMU received:          7230
IMU fed to Kimera:     6605
IMU skipped mapping:   625
TIMESYNC samples:      349
mapping valid:         yes
mapping drift ppm:     2033.564
backend states:        145
PIPELINE RESULT:        PASS
```

## Reported measurement

```text
START avg states:       22
END avg states:         22
START mean P:           [0.016950,-0.001530,-0.001094] m
END mean P:             [0.484976,-0.151364,0.013021] m
measured dP:            [0.468026,-0.149834,0.014114] m
horizontal distance:    491.424848 mm
3D measured distance:   491.627499 mm
expected distance:      500.000000 mm
absolute error:         -8.372501 mm
relative error:         -1.674500 %
scale measured/true:    0.983255
```

## Why the scale result is rejected

The START reference window was contaminated by real motion before the program emitted the `MOVE NOW` marker.

The live trace still showed the START countdown while pose had already changed materially:

```text
kf=20 P=[0.0002,0.0002,-0.0017]   START countdown
kf=30 P=[0.0223,0.0004,-0.0003]   START countdown, 3 s remaining
```

Therefore the computed START mean already contained approximately 17 mm of displacement on X (`START mean P.x = 0.016950 m`). The resulting `491.627 mm`, `-1.6745 %`, and `scale=0.983255` must not be used as a scale qualification result.

This is not a failure of the VIO pipeline. It is a protocol/reference-window failure.

## Other observations

The END phase was stationary enough for a clean average (`END mean |V| = 0.576850 mm/s`). Camera continuity and TIMESYNC were also healthy.

The pose trace reached roughly 0.48 m X and -0.15 m Y, showing that the physical hand motion contained a substantial lateral component in the Kimera world frame. This is acceptable for distance-magnitude validation if the real physical path between marks is exactly 500 mm, but the START reference must be uncontaminated.

Orientation changed during the hand-carried move (`dRoll=0.324 deg`, `dPitch=-1.935 deg`, `dYaw=-1.168 deg`). This is recorded but does not by itself invalidate a 3D displacement-magnitude test.

## Required repeat

Repeat the same 500 mm one-way test. Keep the rig completely motionless until the terminal prints the exact `MOVE NOW` line. Do not begin during the countdown. After reaching the 500 mm mark, stop as early as practical and remain motionless through the END phase.

Stage 12 `known linear displacement` remains open until a clean trial is completed.
