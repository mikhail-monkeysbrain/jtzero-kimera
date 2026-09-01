# Live Mono + IMU 500 mm — trial 4 — 2026-09-01

## Result

**PIPELINE PASS / SCALE MEASUREMENT REJECTED**

The data path remained healthy, but the VIO trajectory contains a large non-physical transient during the motion phase. This trial must not be used as a scale measurement.

## Runtime counters

```text
raw camera frames:     4222
rejected raw pairs:    0
selected frames:       998
decoded frames:        998
IMU received:          7227
IMU fed to Kimera:     6605
IMU skipped mapping:   622
TIMESYNC samples:      348
mapping valid:         yes
mapping drift ppm:     2033.367
backend states:        145
START avg states:      22
END avg states:        22
PIPELINE RESULT:       PASS
```

## Reported endpoint measurement

```text
START mean P:           [0.000431,0.000588,0.001186] m
END mean P:             [0.290822,-0.071300,0.028161] m
measured dP:            [0.290391,-0.071887,0.026975] m
horizontal distance:    299.156577 mm
3D measured distance:   300.370305 mm
expected distance:      500.000000 mm
absolute error:         -199.629695 mm
relative error:         -39.925939 %
scale measured/true:    0.600741
dRoll:                  -0.102793 deg
dPitch:                 -2.081162 deg
dYaw:                   -2.466327 deg
END mean |V|:           0.880627 mm/s
```

## Why this trial is rejected for scale

During what should be one continuous forward translation, the estimated position jumps strongly backward between backend states around kf=50 and kf=60:

```text
kf=50 P=[0.2214,-0.0919,0.0030] V=[ 0.0311,-0.0278,0.0016]
kf=60 P=[0.1212, 0.0076,0.0276] V=[-0.2557, 0.0432,0.0072]
```

The position change over this interval is approximately:

```text
dP ~= [-0.1002,+0.0995,+0.0246] m
|dP| ~= 143 mm
```

and the reported velocity briefly reaches about 0.26 m/s in the opposite longitudinal direction. This is inconsistent with the commanded single straight 500 mm motion and dominates the final 300 mm endpoint result.

Therefore the 0.600741 scale value is not evidence of a global scale factor. It is evidence of a VIO tracking/state-estimation failure during the motion interval.

## Interpretation

Input continuity and synchronization do not explain the event at the coarse runtime level: there were zero rejected raw camera pairs, TIMESYNC stayed valid, and the clock drift remained in the previously observed ~2030 ppm range. The failure is therefore more likely inside visual tracking / visual-inertial update consistency than in gross camera-frame transport continuity.

The next diagnostic should record frontend/backend quality around every motion frame/keyframe rather than repeating endpoint-only scale trials blindly. In particular, capture feature count, tracked feature count, stereo/mono tracking status available from Kimera frontend, RANSAC/inlier counts or related frontend status if exposed, backend state increments, and enough timestamped IMU/camera diagnostics to identify the kf=50→60 failure interval.

Do not mark the known-linear-movement checklist item complete from this trial.
