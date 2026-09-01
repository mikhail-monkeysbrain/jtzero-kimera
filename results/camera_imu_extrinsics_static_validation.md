# Camera ↔ IMU full extrinsics static validation

## Input

Calibration file:

`calibration/ov9281_extrinsics_candidate.yaml`

Validator:

`tools/camera_imu_extrinsics_validator.cpp`

Current transform:

```text
R_BC =
 0.009367371  0.999954838 -0.001604448
-0.999326771  0.009418381  0.035458408
 0.035471918  0.001271215  0.999369865

t_BC = [0.000, 0.000, 0.055] m
```

## Validation result

The static full-transform validator returned PASS for every implemented check:

- `R_BC` orthonormal: PASS
- `det(R_BC)=+1`: PASS
- `R_CB == R_BC^T`: PASS
- homogeneous `T_BS` bottom row: PASS
- duplicated `t_BC` field matches `T_BS`: PASS
- camera optical `+C_Z` maps to body down: PASS; `[-0.001604, 0.035458, 0.999370]_B`
- camera `+C_X` maps approximately to body `-Y`: PASS; `[0.009367, -0.999327, 0.035472]_B`
- camera `+C_Y` maps approximately to body `+X`: PASS; `[0.999955, 0.009418, 0.001271]_B`
- mechanical XY offset <=10 mm: PASS; configured XY=0 mm
- mechanical Z within 55 +/-3 mm: PASS; configured Z=55 mm

Static result:

`PASS`

## Interpretation

This test verifies the mathematical structure, transform direction, axis mapping, signs, and consistency of the serialized `T_BS` against the measured mechanical geometry.

It does not independently estimate the 55 mm camera-to-FC lever arm from visual/IMU motion. The translation source remains direct mechanical measurement:

```text
X = 0 +/- 5 mm
Y = 0 +/- 5 mm
Z = 55 +/- 3 mm
```

The current rotation is independently experimentally validated by two post-rebuild synchronized ChArUco + HIGHRES_IMU captures with approximately 0.465 deg geodesic disagreement.

For the current rigid assembly, the transform is therefore suitable to promote to the project calibration file. A further live Mono+IMU motion test in Stage 12 remains the end-to-end validation of the complete VIO use of these extrinsics.
