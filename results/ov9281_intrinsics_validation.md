# OV9281 intrinsic calibration validation

Configuration validated in this stage:

- Camera: OV9281 USB UVC
- Calibration image size: 640x480
- Calibration model: pinhole + radtan_5
- Calibration target: ChArUco 7x5, DICT_4X4_50
- squareLength: 27.324 mm
- markerLength: 20.043 mm

## Calibration dataset

63 independent calibration views were accepted, with no detection rejects.

Final calibration:

- fx = 568.53170752165227
- fy = 569.68005562865858
- cx = 315.98271077441063
- cy = 239.88148589100641
- k1 = 0.073569192194028493
- k2 = -0.095253893789117
- p1 = -0.010810530757187299
- p2 = -0.0022843373576970235
- k3 = 0.082177400802757483
- calibration RMS = 0.34732173182114762 px

The final 4x3 calibration coverage grid was:

```text
32   92   85   23
108 268  212   55
40  108   57   16
```

All 63 per-view reprojection errors were below 0.5 px.

## Independent validation dataset

A separate 49-frame dataset, not used for fitting the calibration, was processed with `tools/ov9281_charuco_validate.cpp`.

Results:

- usable_views = 49
- rejected_detection = 0
- rejected_pnp = 0
- mean_error = 0.392143 px
- median_error = 0.379265 px
- aggregate_view_rmse = 0.407339 px
- max_error = 0.686381 px
- views_above_0.50px = 8
- views_above_0.75px = 0
- RESULT: PASS

The independent validation error remained close to the fitting error, with no view above 0.75 px.

## Undistortion visual check

The 49-frame independent validation dataset was undistorted with `tools/ov9281_charuco_undistort.cpp` using the saved calibration. All 49 images were written successfully.

A 9-pair RAW/UNDIST contact sheet was visually inspected. The correction was smooth and moderate, with the expected increase toward the image periphery. No obvious S-shaped bending, wave-like distortion, local discontinuities, or strong left/right asymmetric warping were observed. No visible over-correction was identified at the image edges or corners.

Visual undistortion check: PASS.

## Calibration artifact

The exact OpenCV YAML produced by the calibration run is stored at:

`calibration/ov9281_intrinsics.yaml`

This calibration is valid for the tested OV9281 USB UVC 640x480 image geometry with unchanged optics. Recalibration is required if the lens/focus, crop, scaling, image resolution, or camera interface changes in a way that changes image geometry.
