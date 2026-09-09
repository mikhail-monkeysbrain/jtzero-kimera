# JT-Zero P11 / V21-V25 — текущий статус

Назначение: компактная operational memory. Полная хронология остаётся в `docs/ASSISTANT_ACTION_LOG.md`.

Последнее обновление: 2026-09-07.

## Главный вопрос

Почему при стендовом горизонтальном движении возникают ложный VIO/PIM Z и нестабильный horizontal scale, и какая часть эффекта связана с реальной механикой стенда, raw FC IMU и visual-inertial observability?

## Что подтверждено

- TF-Luna показывает практически постоянную реальную высоту при корректном bench horizontal motion, тогда как VIO создаёт ложный Z.
- Ложный vertical velocity появляется уже на PIM/inertial prediction до основной backend correction.
- Backend accelerometer bias не является единственным источником directional Z.
- В FC HIGHRES_IMU наблюдается A/B-зависимый stationary acceleration signal.
- Два независимых raw position runs дали norm split:
  - 151131: B-A = -0.039198 m/s²;
  - 160342: B-A = -0.036468 m/s².
- Эта межпрогонная воспроизводимость norm split пока предварительная: n=2 independent runs.
- Paired raw-vector analysis дал acceleration-vector direction change порядка 2.2–2.4°.
- На canonical V23 все шесть legs показывают:
  - stationary raw gravity-vector change примерно 2.20–2.36°;
  - FC relative tilt примерно 2.18–2.48°;
  - VIO tilt того же порядка;
  - gyro-integrated rigid rotation, предсказывающий endpoint gravity-vector orientation с ошибкой примерно 0.07–0.25°.
- Signed raw-gyro rotation vector стабильно меняет знак при A↔B. Типичные direction medians в FC FRD:
  - A->B ≈ [-1.782, +1.651, -0.654]°;
  - B->A ≈ [+1.555, -1.590, +0.658]°.
- Внешний world-fixed 2-D cross-marker A→B→A имеет хорошее closure и projective-invariant test отвергает pure fixed-orientation translation как объяснение B deformation. Это поддерживает реальное изменение orientation жёсткой marker/body structure, а не только перспективный slope artifact.
- Поэтому реальная обратимая механическая rotation во время V23 translation сейчас поддерживается несколькими разными сигналами: raw gyro, raw gravity, FC attitude, VIO attitude и внешний 2-D marker.
- При этом ранний Z drift существует до существенного gyro rotation/VIO tilt. Следовательно mechanical rotation не является единственной причиной vertical error.
- Endpoint rotation magnitude и простой rotation exposure не объясняют horizontal scale: при близких ~2.3–2.6° endpoint rotations scale V23 сильно меняется.
- На V25 `MANUAL_PROFILE_01_CLEAN` raw FC motion profile показал, что B->A legs были физически сильнее возбуждены:
  - A->B mean: scale=1.0196, duration=9.36 s, hAcc RMS=0.3222 m/s², p90=0.4783 m/s², dyn RMS=0.3332 m/s², yaw span=0.321°;
  - B->A mean: scale=1.0963, duration=7.69 s, hAcc RMS=0.4331 m/s², p90=0.6661 m/s², dyn RMS=0.4549 m/s², yaw span=1.140°.
- Поэтому этот V25 run не изолирует pure direction-dependent scale defect от motion-profile dependence.
- Backend BA common-axis analysis показал persistent/drifting BA одного знака; прежний A->B/B->A sign flip был projection artifact. BA остаётся estimator symptom, но сам по себе не объясняет scale.
- V25 ARW=0.0003 causal test ухудшил scale/Z; простая гипотеза «BA слишком свободен, tighten ARW исправит distance» отвергнута.

## Важная количественная сверка

Не смешивать разные величины:

1. Stationary norm split ~0.037–0.039 m/s²:
   - ~0.0038 g;
   - грубый g-equivalent angle ~0.22°.
   - Сам по себе не объясняет ~2° attitude effect.

2. Stationary vector-direction / rigid-rotation effect:
   - raw acceleration-vector change ~2.2–2.4°;
   - gyro-integrated rotation того же порядка;
   - FC/VIO attitude того же порядка.
   - Эти величины согласуются по масштабу и времени на V23.

Это поддерживает реальную rotation branch, но не превращает её автоматически в объяснение horizontal scale или раннего Z drift.

## Что НЕ установлено

- Механизм раннего pre-motion Z/Vz drift до существенной rotation.
- Почему physical A/B/handling state меняет stationary FC acceleration norm/vector в P11 raw position tests.
- Как именно реальная mechanical rotation передаётся в horizontal scale error и какую долю scale variance она объясняет.
- Есть ли независимый direction-dependent visual/estimator defect после matching по raw physical excitation.
- Почему backend BA drift/persistence возникает при raw FC acceleration, близком к симметричному вдоль общей оси.
- Насколько bench A/B effects переносятся на свободный полёт.
- Какая часть наблюдаемой rotation относится к whole-rig rigid motion, а какая потенциально к local flex между marker/body/FC mount. Внешний 2-D marker сильно ослабил pure local-FC-only branch, но метрическая 3-D ось/угол внешней камеры ещё не восстановлены.

## Заблокированные / закрытые ветки

- Pure monocular homography → physical tilt: закрыто как неразделимое translation/plane explanation.
- Fixed-target onboard stereo stability: закрыто физическим ограничением стенда.
- ChArUco pose из старого P11 stereo run: невалидно для dataset.
- Backend-speed-as-physical-motion-evidence: запрещено как circular metric.
- BA along-leg sign reversal: закрыто как projection artifact.
- «Tighten accelerometer random walk = fix scale»: отвергнуто ARW=0.0003 causal test.
- Pure perspective translation как объяснение внешнего cross-marker B state: отвергнуто projective-invariant test в одном внешнем run.

## Предварительные результаты

- raw A/B norm split: n=2 independent runs.
- external cross-marker orientation discriminator: один физический внешний A→B→A video run; сильный, но single-run.
- Некоторые geometry/stereo results остаются n=1 и не должны использоваться как окончательное causal proof.

## Следующие наиболее ценные действия

1. **Не делать новый physical run до использования существующего архива.**
   Сопоставить V25 legs между архивными baseline runs только по независимым raw-input descriptors:
   operator duration + raw FC accel/gyro + FC attitude spans. Не использовать backend speed или backend BA для matching.

2. **Новый cross-run discriminator:** `tools/analyze_v25_raw_profile_crossrun_match.py`.
   Он ранжирует opposite-direction legs по близости raw physical excitation и отдельно same-direction pairs как repeatability control.
   Scale не входит в matching score и используется только как проверяемый output.

3. Если среди существующих runs найдутся близкие raw-matched opposite-direction пары:
   - большой сохраняющийся scale delta ослабит pure motion-profile confounding и усилит direction/visual-estimator branch;
   - уменьшение scale delta при хорошем raw matching усилит motion-excitation explanation.

4. Только если архив не содержит достаточно близких raw-matched legs, собирать дополнительные baseline V25 runs тем же untimed Space/Enter protocol, немедленно архивируя каждый run. Принимать causal comparison только после post-run raw matching.

5. Early Z branch вести отдельно от horizontal scale: rotation уже не может быть единственным объяснением Z из-за pre-motion drift.

## Обязательный preflight перед новым кодом/тестом

- Прочитать этот STATUS и релевантный участок ACTION_LOG.
- Проверить, не повторяется ли действие.
- Не использовать backend/VIO output как доказательство physical input, если доступен raw FC signal.
- Для geometry/calibration code сначала открыть фактический calibration/config и проверить sensor order, stereo axis/type, frames, units, K/D/R/T/P.
- Перед причинным выводом выполнить order-of-magnitude reconciliation.
- n<3 independent physical runs = предварительный результат, если нет отдельного независимого discriminator.


## 2026-09-09 — V43 camera-only scale branch

User correction on status semantics: eliminating a hypothesis without identifying a more likely cause is **НА МЕСТЕ**, not **ПРОДВИНУЛИСЬ**. Use ПРОДВИНУЛИСЬ only when the new result materially narrows the cause toward an actionable mechanism or improves the project solution.

Current V43 camera-only facts:
- real 500 mm motion -> camera-only 565.93 mm with logged dynamic h (mean 193.16 mm);
- recomputing the same real tx/ty with fixed h=180 mm still gives 526.45 mm (+5.29%);
- fixed h=185 mm gives 541.07 mm (+8.21%);
- exact fixed h needed to force 500 mm is 170.96 mm;
- affine-origin vs inlier-centroid parameterization changes accumulated pixel norm by only ~0.013%;
- affine scale itself is ~1.0006, so it is not the 1.13 metric factor;
- lens distortion alone and tested small projective perturbations were insufficient in prior screens.

Current leading hypotheses for the residual camera-only scale error:
1. **Effective focal length / runtime camera geometry mismatch**: V43 uses hard-coded fx/fy from calibration. Since metric distance is proportional to h/f, a 5–8% residual can be produced by a 5–8% mismatch between actual effective focal length in the runtime 640x480 stream and the calibration value. Possible causes: stale intrinsics, different crop/ROI/scaler mode, camera mode mismatch, calibration file not matching the actual USB stream.
2. **Plane/camera geometry mismatch**: h/f translation formula assumes a fronto-parallel planar ground and correct optical-center-to-plane perpendicular height. Camera tilt, non-fronto-parallel plane, or using the wrong reference height can bias scale. Current height uncertainty explains a substantial part but not all of the error.
3. **Residual model mismatch in optical-flow/affine translation** not captured by the tested scale/rotation centroid checks: spatially nonuniform flow from projective geometry or calibration mismatch can bias tx/ty even when fitted affine scale is near 1.
4. **Physical 500-mm reference / endpoint protocol error** is currently lower priority because Kimera has produced runs near 500 mm (e.g. 495.79 mm) on the same bench, but it remains a generic external possibility and should not be declared impossible without an independent ruler/marker check.

Next discriminator: derive the effective focal length required by the real V43 tx/ty at fixed physically plausible heights, compare it to calibration fx/fy, and inspect whether the runtime USB camera mode/crop is consistent with the calibration mode.


## 2026-09-09 — V44 one-pass camera sweep result

Latest one-pass archive shows:
- affine and median inlier flow agree almost exactly: median-flow/affine = 0.9998, so affine fitting itself is not the source of +5..8% residual scale;
- left/right median flow ratio = 1.0293 and top/bottom = 0.9737, so large spatial/projective nonuniformity is not evident;
- with fixed h=180 mm the same real flow gives 527.40 mm (+5.48%); h=185 mm gives 542.05 mm (+8.41%);
- runtime stream is 640x480 and both Kimera params and camera-only use the same calibrated intrinsics 568.53/569.68, weakening a simple config-number mismatch;
- the calibration itself is a real 640x480 OV9281 calibration with RMS 0.347 px / 63 usable views.

A stronger remaining camera-only hypothesis is now **physical attitude rotation counted as translation**. The rig changes roll/pitch by degrees during bench translation; camera-only integrates 2-D flow without IMU rotation compensation. Pitch/roll rotation can appear mainly as translational optical flow even when affine in-plane rotation/scale are near identity. This mechanism was not ruled out by the previous affine-scale/centroid tests.

Added tools/analyze_v44_fc_rotation_compensation.py. It aligns the 220 forensic intervals uniformly over the exact START/END wall-time window, interpolates FC ATTITUDE, maps FRD->FLU, transforms the active camera extrinsic R_BC, predicts rotation-only flow at each real inlier centroid, subtracts it from the real median flow, and recomputes metric distance for h=180/185 mm.

Status semantics: root cause not yet identified; **НА МЕСТЕ** until this mechanism is supported/rejected quantitatively.


## 2026-09-09 — V44 physical height measurement

Physical measurement: working surface -> OV9281 sensor plane = **185–186 mm**.
This rejects the ~173.2 mm height required to close the V44 camera-only scale after FC-rotation compensation.
At h=185 mm the remaining corrected camera-only excess is ~+6.8%, so do not tune height to ~173 mm.
Next discriminator: static runtime V4L2/camera-geometry audit, then independent effective-focal measurement if runtime mode is consistent.


## 2026-09-09 — V44.3 runtime geometry audit

Static audit on the active OV9281/rp1-cfe capture node:
- active capture geometry is 640x480, matching the 640x480 calibration resolution;
- active raw format is pRAA (10-bit Bayer);
- VIDIOC_G_PARM is unsupported on this rp1-cfe node and is not treated as a geometry failure;
- therefore a simple runtime-resolution mismatch is rejected as the source of the remaining camera scale error.

Physical sensor-plane height remains measured at 185-186 mm, so the ~173.2 mm height closure remains rejected.
Next discriminator: independent static effective-focal / projection-scale measurement at the real 185-186 mm sensor-plane height. No A->B run is required.


## 2026-09-09 — V44.4 static OV9281 grid capture ready

A correct OV9281 USB capture was obtained from /dev/video11: 20 MJPEG frames, 640x480, camera remained enumerated after capture.

Physical target measurement supplied by user: 80.77 mm over the marked two-large-square span, therefore one large checker square is provisionally 40.385 mm. Physical working-plane to OV9281 sensor-plane height remains 185-186 mm (use 185.5 mm nominal).

Added `tools/analyze_v44_grid_effective_focal.py`. It uses the same 20 static OV9281 frames, auto-detects the largest checker-grid inner-corner pattern, solves planar pose with the stored K/D, compares recovered perpendicular camera-to-plane distance against the independently measured 185.5 mm, and converts the discrepancy into an independent effective focal multiplier. This directly tests the remaining ~1.068 focal-scale hypothesis without a new A->B run.

Caution: if 80.77 mm was not exactly a two-large-square physical span, rerun with the correct one-square size; do not interpret focal scale until target geometry is confirmed.


## 2026-09-09 — V44.4 first analyzer result invalid; false checker-grid detections

The first V44.4 analyzer run produced physically impossible plane heights, 34-89 deg board tilts and reprojection RMS ~20-106 px across nominally static frames. This is not a camera-calibration result. It demonstrates that generic findChessboardCornersSB was locking onto false sub-patterns in the ArUco/checker artwork.

Do **not** use the reported focal_k~1.998 or any individual PnP height from that run.

Analyzer updated to V44.4b:
- ranks candidate checker grids by adjacent-spacing regularity;
- rejects irregular false detections;
- avoids interpreting bad PnP;
- estimates local effective fx/fy from robust adjacent physical-square spacing;
- recomputes spacing after K/D undistortion;
- reports homography reprojection RMS as a geometry sanity check.

If the artwork is not detectable as one regular checker grid, next discriminator will use explicit ArUco/ChArUco geometry or explicit endpoints of the physically measured 80.77 mm span.


## 2026-09-09 — V44.4b rejects checker interpretation

V44.4b rejected all 20 frames (regularity 0.345-1.742, threshold 0.20). Therefore the printed ArUco/checker artwork must not be treated as a conventional checkerboard. This confirms the first V44.4 focal result was an artifact of false checker detections.

Next static discriminator is V44.5: detect ArUco markers directly. The analyzer sweeps common OpenCV ArUco dictionaries only because the target dictionary has not yet been recorded; dictionary selection is based on detection consistency across the 20 existing frames, never proximity to the desired focal. A physical measurement of one marker's outer black-square side is required. No new A->B motion run is required.


## 2026-09-09 — V44.5 ArUco detection succeeds; metric interpretation pending real marker size

Direct ArUco detection is robust: 20/20 frames, 274 detections, 14 unique IDs. Common DICT_4X4 variants decode the same first 50 IDs, so detection consistency is strong but dictionary family is not uniquely identified; this does not affect corner geometry.

The run used `--marker-mm 24.35`, which had previously been given only as an example and was not independently confirmed as the real outer black-square side. Therefore the reported effective focal ~487/486 px and k~0.855 MUST NOT yet be interpreted as camera geometry.

The analyzer now reports the physical marker side implied by two competing hypotheses using the observed pixel geometry:
- side required for stored focal k=1.000;
- side required for the V44 motion residual focal hypothesis k=1.068.

Next discriminator: caliper-measure the actual outer black-square side of one marker and compare to those two predicted physical sizes. No new camera capture or A->B run is needed.


## 2026-09-09 — V44.5 physical marker-side discriminator targets

Using the 20 static OV9281 frames, direct ArUco detection is stable (20/20 frames, 274 marker detections). The provisional `--marker-mm 24.35` was only an example and is not a physical measurement, so the corresponding k~0.855 remains non-interpretable.

The observed pixel geometry predicts the real OUTER black-square side that would be required by two competing hypotheses:
- stored focal correct (k=1.000): **20.822 mm**;
- motion residual effective-focal hypothesis (k=1.068): **19.496 mm**.

Next discriminator is purely physical and requires no new image or A->B run: measure the outer black-square side of at least three printed markers (center/left/right if possible) with calipers. Use the mean and spread to compare against 20.822 vs 19.496 mm. This also checks printer-scale uniformity across the sheet.
