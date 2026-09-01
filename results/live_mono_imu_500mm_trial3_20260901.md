# Live Mono + IMU 500 mm — trial 3 — 2026-09-01

## Result

**VALID TRIAL / PIPELINE PASS**

The rig remained stationary through the START reference window and motion began after the explicit MOVE command. This trial is valid for scale-repeatability analysis.

## Runtime counters

```text
raw camera frames:     4222
rejected raw pairs:    0
selected frames:       998
decoded frames:        998
IMU received:          7228
IMU fed to Kimera:     6605
IMU skipped mapping:   623
TIMESYNC samples:      347
mapping valid:         yes
mapping drift ppm:     2040.399
backend states:        145
START avg states:      21
END avg states:        22
```

## Measurement

```text
START mean P:           [-0.000332,0.000323,-0.000005] m
END mean P:             [0.474149,-0.123664,0.011360] m
measured dP:            [0.474481,-0.123987,0.011365] m
horizontal distance:    490.412833 mm
3D measured distance:   490.544512 mm
expected distance:      500.000000 mm
absolute error:         -9.455488 mm
relative error:         -1.891098 %
scale measured/true:    0.981089
dRoll:                  -0.094313 deg
dPitch:                 -1.960003 deg
dYaw:                   -3.445114 deg
END mean |V|:           0.752439 mm/s
PIPELINE RESULT:         PASS
```

## Interpretation

Trial 3 measures 490.54 mm for the mechanically commanded 500 mm translation, an under-estimate of 9.46 mm (-1.89%). The inferred scale factor is 0.98109.

This differs materially from valid trial 2, which measured 455.47 mm (scale 0.91094). Therefore the current data do not support applying a single constant global scale correction.

The run also had substantially less yaw change than trial 2: -3.45 deg versus -6.07 deg. Because the rig is moved by hand and cannot be held at perfectly constant attitude, coupling between translation and rotational/manual motion remains a candidate contributor to run-to-run scale variation. More repeated trials are required before changing calibration or estimator parameters.

Source continuity and timing were clean: zero rejected raw pairs, valid TIMESYNC mapping, and a stable final state with mean speed 0.75 mm/s.

## Next action

Run at least one more identical valid 500 mm trial without changing parameters. Compare scale against trials 2 and 3 before any tuning. If the scale remains variable, move to controlled-motion diagnostics rather than introducing a fixed scale multiplier.
