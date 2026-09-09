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


## 2026-09-09 — V44.5 physical marker measurement = 26.47 mm

User measured the outer black-square side on multiple target squares/markers as **26.47 mm**, with the same value at the sampled positions. This is materially larger than both inverse predictions from V44.5 (20.822 mm for stored K and 19.496 mm for k=1.068).

With the same detected pixel geometry and h=185.5 mm, replacing the provisional 24.35 mm by the measured 26.47 mm scales the inferred focal by 24.35/26.47: approximately fx=448.4 px, fy=446.9 px, mean k≈0.787. This does NOT support the prior +6.8% effective-focal hypothesis; it creates a much larger opposite-sign discrepancy.

Do not tune K to ~447 px yet. V44.5 is a local fronto-parallel scale approximation and the target/image show perspective/tilt. Next step must use full planar ArUco geometry / homography (or solvePnP) across many markers, jointly checking plane pose, metric scale, stored K, and the independently measured 185.5 mm sensor-plane distance on the SAME existing images. No new A->B run is required.


## 2026-09-09 — V44.6 full-board planar geometry sweep added

V44.5 with the real 26.47 mm marker side gives a naive local-scale k~0.787, opposite in sign and far larger than the +6.8% motion residual hypothesis. Because the target plane is visibly projective and the local-scale formula assumes fronto-parallel geometry, this is not a valid calibration replacement.

Added `tools/analyze_v44_fullboard_geometry.py` to use the SAME 20 static OV9281 frames and the independently measured physical geometry:
- large checker-cell pitch = 40.385 mm (from measured 80.77 mm over two cells);
- ArUco outer black-square side = 26.47 mm;
- sensor-plane to working-plane height = 185.5 mm nominal.

The analyzer derives a consistent marker-center lattice from stable ArUco IDs, builds full 3-D planar marker-corner coordinates, and sweeps shared focal multiplier k plus distortion-strength scale. For every model it solves board pose and reports reprojection RMS, recovered perpendicular camera-plane height, and tilt. It explicitly compares stored K (k=1), motion hypothesis (k=1.068), and the naive local-scale k~0.7866. No new image or A->B run is needed.


## 2026-09-09 — V44.6 full-board lattice inference invalid

V44.6 produced ~66.5 px reprojection RMS and recovered plane heights 381-611 mm for all focal hypotheses, despite the independently measured 185.5 mm physical height. The inferred marker lattice coordinates are therefore wrong; this is not evidence about focal length. Do not use V44.6 to tune K.

A stronger same-data discriminator avoids board-layout inference entirely: every detected ArUco marker is itself an independent metric square of measured outer side 26.47 mm. Added `tools/analyze_v44_marker_pose_height.py` (V44.7), which solves a planar pose independently for every marker across the same 20 frames while sweeping focal multiplier and distortion strength. It compares median recovered camera-to-plane distance and marker-to-marker height MAD against the physical 185.5 mm. This checks stored K vs k=1.068 without requiring any board-ID layout, new image, or A->B run.


## 2026-09-09 — V44.8 known ChArUco board geometry

The printed target PDF is explicitly: ChArUco 7x5, nominal square 30.0 mm, marker 22.0 mm, DICT_4X4_50, print 100%/actual-size reference. User physically measured the printed square as 26.47 mm. Therefore the printed marker outer black-square side implied by the board design ratio is 26.47*(22/30)=**19.411 mm**. This is close to V44.5's independently predicted 19.496 mm under the k=1.068 motion-residual hypothesis.

The earlier V44.7 run is both computationally inefficient and was launched with marker-mm=26.47 (square size mistakenly used as marker size). Stop/ignore that run.

Added `tools/analyze_v44_charuco_known_board.py` (V44.8):
- uses the actual 7x5 ChArUco board definition;
- uses actual printed square 26.47 mm and marker 19.411 mm by default;
- detects/interpolates ChArUco corners once per frame, then median-aggregates stable corners across the existing 20 images;
- sweeps focal multiplier and distortion strength on the cached median geometry, so runtime is fast;
- compares reprojection RMS and recovered camera-to-plane distance against the independent 185.5 mm physical height;
- explicitly reports stored K (1.0), motion k (1.068), and local-naive k.

No new image or A->B run is required.


## 2026-09-09 — V44.8 result: static geometry requires k ~= 1.11

V44.8 used the known ChArUco 7x5 layout and the physically measured printed square 26.47 mm. The marker side was correctly derived from the board ratio 22/30 as 19.411 mm. Detection was strong: all 20 frames produced 12–16 ChArUco corners from 13–14 markers, with 16 stable corners used for pose fitting.

Key results:
- stored K (k=1.000, distortion scale 0): reprojection RMS 0.909 px, recovered height 167.13 mm, height error -9.90%;
- motion hypothesis (nearest grid k=1.0675, distortion scale 0): RMS 0.906 px, height 178.31 mm, error -3.88%;
- best joint low-RMS + physical-height candidate: k=1.1100, distortion scale 0.10, RMS 0.913 px, height 185.52 mm, error +0.01%, tilt 4.53 deg;
- with distortion disabled, k=1.1100 still gives RMS 0.905 px and height 185.34 mm (-0.09%), so the conclusion does not depend on tuning distortion.

Interpretation:
1. The old stored focal scale is independently inconsistent with the measured 185.5 mm camera-to-plane height by about 10%.
2. The earlier k~=1.068 motion-scale hypothesis moves in the correct direction but is insufficient; static target geometry points to k~=1.11.
3. Reprojection RMS alone is almost flat versus focal scale (~0.90 px), so focal scale is weakly observable from a single near-frontoparallel planar view. The independent physical height is what discriminates k.
4. Do not overwrite production intrinsics yet. Next discriminator must use the same camera/format with several independently measured camera-to-board heights and preferably deliberate board tilt, fitting one common focal scale across all captures. This separates focal scale from distance/planar-pose ambiguity.
5. Existing V44.4 frames are near-static duplicates and therefore must not be treated as 20 independent geometry experiments.

No new A->B motion run should be performed before the multi-height static validation.


## 2026-09-09 — V44.9 multi-height static validation

V44.8 prefers k~1.11 only after using one independently measured planar distance. Because focal length and planar pose are coupled in a near-frontoparallel single-view test, production intrinsics must not be changed from one height.

Added V44.9:
- tools/run_v44_9_static_capture.sh <height_mm> <label> auto-discovers the OV9281 V4L2 node and captures 20 MJPEG 640x480 frames plus metadata;
- tools/analyze_v44_9_multiheight.py fits one common focal multiplier and distortion-strength scale across all captured heights and reports the per-height implied k with distortion disabled.

Protocol: capture at least three independently measured sensor-plane-to-board distances spanning roughly 150-260 mm, board flat for the first three captures. No A->B motion run until common-k stability is established.


## 2026-09-09 — V44.10 adapted to board only at start of 500 mm path

Constraint: the ChArUco board exists only at A/start and cannot cover the full 500 mm route. V44.9 multi-height protocol is not feasible and is superseded for the current stand.

Added V44.10 start-board anchored one-pass protocol:
- capture 20 static OV9281 ChArUco frames at A only;
- derive a start focal-scale anchor from the known 7x5 board, actual square 26.47 mm, marker ratio 22/30, and physical sensor-plane height 185.5 mm;
- then perform exactly one standard V43 A->B 500 mm pass; the board may disappear immediately after motion begins;
- offline analysis applies FC-attitude rotation compensation and compares the full 500 mm camera flow using stored focal vs the independently derived start-anchor focal;
- also compares affine-net and median-flow estimates under the same anchor to test whether the correction transfers from static board geometry to real translation.

This directly tests whether the start-board projection-scale discrepancy is causal for the 500 mm motion error without requiring the board along the route or changing stand height.


## 2026-09-09 — V44.10 result separates camera-scale improvement from Kimera regression

Current V44.10 physical pass archive: `20260909_133245_v43_CAMERA_AFFINE_FORENSIC`.

Observed:
- Kimera horizontal = 417.59 mm (scale 0.8352), worse than the recent ~500 mm-class runs;
- camera-only in the same pass remains high with stored focal (~538.5 mm raw / ~532.9 mm after FC rotation);
- applying the independently measured start ChArUco anchor k=1.1025 brings camera-only to ~488.5 mm raw, ~483.3 mm after rotation, and affine net ~490.6 mm.

Therefore the 417.6 mm regression is NOT caused by applying the ChArUco focal correction to Kimera: production Kimera parameters were not changed by V44.10. The camera-only correction actually moves the independent visual displacement toward 500 mm, while Kimera independently underestimates the same pass.

Added `tools/analyze_v44_11_kimera_regression_crossrun.py` to compare the current bad run against a prior reference run using the already archived data only. It jointly screens camera pixel motion, frontend validity/inliers/tracks, raw IMU excitation/attitude span, backend endpoint components, and final Kimera scale. No new physical run is required.


## 2026-09-09 — V44.12 reveals non-monotonic current backend trajectory

V44.12 shows the 417.6 mm run is not a simple constant scale underestimate. Relative to the reference, the current backend starts behind, overtakes strongly in the middle (+59.5 mm horizontal at 60% normalized progress), then loses distance rapidly late in the run and finishes -68.9 mm behind. Current horizontal displacement peaks around 85% (~439 mm) and then falls to 417.6 mm while the reference continues toward ~486.5 mm.

The V44.12 frontend columns were empty because frontend and backend keyframe IDs are not directly shared. Added V44.13 to fix that methodological error: frontend rows are now matched to backend rows by nearest timestamp. The new analyzer prints backend per-step motion from the late run, identifies negative along-track steps and post-peak loss, and correlates them with timestamp-matched frontend status/inlier/tracked/mono-pose quality. No new physical run is required.


## 2026-09-09 — V44.13 localizes the 417 mm regression before LOW_DISPARITY

V44.13 shows the reference remains monotonic to 486.9 mm (post-peak loss 0.4 mm), while the current run peaks at 439.9 mm at 84% and loses 22.3 mm. Crucially, reversal starts at kf=88 on a VALID row (inlier 0.891, 276 tracked), and the largest following negative step at kf=89 is also VALID (inlier 0.929, 312 tracked). LOW_DISPARITY begins only at kf=90, after approximately 10.8 mm of reversal has already accumulated. Therefore the V44.13 generic conclusion that visual rejection remains the primary branch is too coarse.

V44.14 explicitly tests causal order: it separates negative backend displacement accumulated on VALID pose rows from displacement accumulated after non-VALID/LOW_DISPARITY, and prints the archived monocular translation vector plus backend along-track velocity around the first reversal. No new physical run is required.


## 2026-09-09 — V44.15 confirms a current-only accepted visual-pose discontinuity

V44.15: reference contains zero VALID large-geometry events. Current contains exactly one: at ~90% / backend kf89, the accepted monocular body translation jumps by ~63.2 deg, from ~5 deg out-of-plane to 56.5 deg, while status remains VALID, inlier=0.929 and tracked=312. Backend simultaneously moves -9.0 mm. LOW_DISPARITY begins only on the next backend state.

This strongly shifts the diagnosis from generic frontend quality to a specific accepted monocular translation-direction degeneracy near the end of motion. Added V44.16 to cross-check that event against the already logged PIM delta-position/delta-velocity/delta-rotation and to counterfactually screen a conservative gate: VALID + pose_valid + translation direction jump >=30 deg + out-of-plane tilt >=30 deg. The gate is not yet applied to production; V44.16 first measures reference false positives and current selectivity. No new physical run is required.


## 2026-09-09 — V44.16 passes selective pre-fusion gate screen

V44.16 result:
- reference: 33 keyframes, 28 VALID, gate fires = 0;
- current: 32 keyframes, 24 VALID, gate fires = 1;
- the single fire is the known ~90% anomaly: mono direction jump 63.2 deg, out-of-plane tilt 56.5 deg, inlier 0.929, tracked 312;
- PIM rotation at that same interval is only 0.007 deg, while neighboring intervals are also small, so physical camera rotation cannot explain the accepted visual direction jump;
- PIM delta-position magnitude at the event is ordinary for the current run, so the anomaly is isolated to the monocular visual translation geometry rather than a simultaneous inertial impulse.

Added V44.17 as an opt-in Kimera pre-fusion diagnostic patch. Default behavior remains unchanged. When JTZERO_MONO_POSE_GATE=1, a VALID mono keyframe is converted to LOW_DISPARITY before BackendInput only if both body-frame translation direction jump >=30 deg and out-of-plane tilt >=30 deg. The previous-good direction is not updated by a rejected pose. PIM/IMU propagation continues through the backend exactly as with a natural LOW_DISPARITY update.

Files:
- patches/kimera_v44_17_mono_pose_gate.patch
- tools/install_v44_17_mono_pose_gate.sh
- tools/run_v44_17_mono_pose_gate_single.sh

Next physical test is one A->B pass with the gate enabled. Do not change focal/intrinsics for this test.


### V44.17 installer correction

The first V44.17 installer failed before touching Kimera because the hand-written unified diff was malformed (`git apply: corrupt patch at line 18`). This is an artifact-generation error, not a diagnostic result.

V44.17b replaces the fragile unified-diff step with guarded exact source-block replacement. It:
- verifies both source anchors before editing;
- creates `MonoImuPipeline.cpp.v44_17_pre_gate.bak`;
- installs the same default-OFF gate logic;
- prints source markers;
- rebuilds Kimera with `cmake --build /home/vio/Kimera-VIO/build -j2`.


### V44.17b source-anchor mismatch

The guarded installer correctly refused to modify Kimera because the local /home/vio/Kimera-VIO/src/pipeline/MonoImuPipeline.cpp layout does not match the upstream source text used to generate the installer anchors. No source change was applied.

Added V44.17c source-layout snapshot helper to print the exact include block and registerOutputCallback context from the local Kimera checkout. Next step is to derive the patch from the user's actual local source layout, not from upstream assumptions.


### V44.17d — adapted to the actual local MonoImuPipeline callback

V44.17c showed the local Kimera source already contains a JT-Zero `JTZERO_DIAG_IMU_ONLY` branch. This also exposed an important backend semantic: in this tree `LOW_DISPARITY` causes `ZeroVelocityPrior` and `NoMotionFactor`. Therefore the earlier proposal to convert a gated anomaly to LOW_DISPARITY would be methodologically wrong and could itself create a stop/reversal artifact.

V44.17d now patches the exact local callback. For a gated anomalous VALID monocular pose it:
- copies the status/measurements;
- clears smart landmark measurements for that one interval;
- sets tracking status to `INVALID` (not LOW_DISPARITY);
- resets the mono pose in the copied diagnostic packet;
- leaves the PIM/IMU input intact;
- does not update the previous-good translation direction with the rejected pose;
- remains OFF by default and is enabled only by `JTZERO_MONO_POSE_GATE=1`.

This preserves the existing `JTZERO_DIAG_IMU_ONLY` diagnostic path and avoids adding backend no-motion constraints during the pose-gate test.


### V44.18 — first gated run must be verified before another physical run

First V44.17d gated A->B archive: `20260909_135833_v43_CAMERA_AFFINE_FORENSIC`.
Endpoint improved from the prior 417.6 mm regression to 457.63 mm (scale 0.915251), but the terminal output contains no
`[JTZERO-MONO-POSE-GATE]` rejection line. This is not sufficient evidence that the gate caused the improvement.
Camera-only was 548.32 mm while Kimera ended at 457.63 mm, with dz=45.80 mm.

V44.18 adds a no-new-run comparison of the archived reference, ungated regression, and first gated run. The immediate
question is whether the current-only large VALID translation-direction discontinuity identified by V44.15 is actually absent
from the gated archive and whether V44.16 would have fired on that archive. Do not perform another 500 mm physical pass
until this archive-only discriminator is evaluated.


### V44.19 — V44.18 does not prove the runtime gate fired

Archive-only V44.18 shows that the first V44.17d run has no VALID >=30deg translation-geometry event, unlike the
ungated 417.6mm regression (63.2deg direction jump / 56.5deg tilt). However, the captured V44.17d terminal output contains
no `[JTZERO-MONO-POSE-GATE]` rejection marker. The gated archive still enters LOW_DISPARITY at ~87% progress and loses
~12.4mm after its ~469.8mm peak, ending near 457.4mm. Thus the large anomaly disappeared, but gate causality is unproven.

Do not run another physical 500mm pass yet. V44.19 records the archive-only causal distinction. The next runtime change
must add persistent gate counters/summary (evaluated/rejected/max jump/max tilt), so a future run can prove whether the gate
actually evaluated and rejected a pose even if no transient rejection line is noticed.


### V44.20 — persistent runtime gate evidence

V44.19 confirms that archive geometry alone cannot prove that the V44.17d runtime gate fired. The first gated archive has no
large VALID pose discontinuity, but it also has no captured `[JTZERO-MONO-POSE-GATE]` rejection line and still develops a
LOW_DISPARITY tail.

Added persistent gate instrumentation to the local Kimera patch path:
- `evaluated`: number of VALID mono poses examined while the gate is enabled;
- `rejected`: number actually converted to INVALID before backend fusion;
- `max_jump_deg`: largest previous-good to current translation-direction jump observed;
- `max_tilt_deg`: largest out-of-plane translation tilt observed.

A shared stats object prints one `[JTZERO-MONO-POSE-GATE-SUMMARY]` line when the callback is destroyed, including when
`rejected=0`. The V44.20 runner captures the full terminal log and refuses to treat a run as gate evidence if this summary
is absent.

Do not run another physical pass until `tools/install_v44_20_gate_counters.sh` builds successfully.


### V44.21 — V44.20 aborted before the physical pass; endpoint is invalid

The first V44.20 attempt aborted during startup/warmup immediately after JT-IMU-INIT with
`terminate called without an active exception`. No A->B pass occurred. Therefore the archived camera forensic rows are
leftovers/partial runtime data and MUST NOT be interpreted as a 500 mm result. The missing shutdown summary is explained by
abnormal termination: destructors are not guaranteed to run under `std::terminate`/abort.

V44.21 adds crash-safe gate telemetry via `JTZERO_MONO_POSE_GATE_STATS_FILE`. The file is synchronously rewritten and flushed
at gate initialization, every evaluated VALID pose, and every rejection. This provides evidence even if the process aborts.
Also added a stationary startup/warmup smoke wrapper. Do not perform another physical 500 mm movement until this smoke test
survives startup/warmup with the gate enabled.


### V44.22 — stationary smoke reached READY; abort is teardown, not startup

V44.21 stationary smoke reached the GUI READY state. The user did not move and exited with Q/ESC. Crash-safe telemetry then
persisted `phase=shutdown enabled=1 evaluated=0 rejected=0`. The terminal's `terminate called without an active exception`
therefore occurred during teardown after the smoke had reached READY, not during JT-IMU initialization/startup.

However, `evaluated=0` is ambiguous on a stationary scene because the gate only counted VALID poses. V44.22 extends telemetry
with `keyframes_seen`, `valid_seen`, `low_disparity_seen`, and `invalid_seen`, persisted on every keyframe. This provides
direct proof that the runtime callback containing the gate is executing even when no VALID translation exists.

Also fixed `run_v43_camera_affine_forensic_single.sh` to propagate the underlying VIO process exit code after archiving.
Previously an abort could be archived and still appear as wrapper RC=0.

Next: build V44.22 and run one stationary callback smoke. No physical 500 mm movement until `keyframes_seen>0` is confirmed.


### V44.23 result — gate path proven, but gate did not fire; current endpoint ~482 mm

The V44.23 physical pass produced crash-safe telemetry:
`keyframes_seen=107 valid_seen=26 low_disparity_seen=80 invalid_seen=1 evaluated=26 rejected=0 max_jump_deg=3.58629 max_tilt_deg=7.12048`.

Thus the modified callback and gate evaluator are proven active during real motion, and every VALID pose was evaluated. The gate did **not** fire because this run never approached either 30 deg threshold. The user's observed Kimera endpoint was ~482 mm. Therefore the improvement relative to the 417.6 mm and 457.6 mm runs cannot be attributed to pose rejection by the gate; it is run-to-run/estimator behavior unless another archived discriminator identifies a consistent cause.

The V44.23 shell summary printed `valid_seen=1` because its sed pattern matched the suffix of `invalid_seen=1`. This was a reporting bug only; the telemetry file itself correctly says `valid_seen=26`. The parser is fixed to exact key=value token matching.

Added V44.24 to reuse the four archived runs (reference, 417.6 mm regression, first gated ~457.6 mm run, current ~482 mm run) and run the cross-run, pose-discontinuity, late-reversal, and PIM/gate screens in one analysis. No new physical pass is required.


### V44.24 result — 482 mm is a clean run; 417 mm failure is not a persistent focal/gate mode

Reference and current are close: Kimera 486.07 vs 482.11 mm, camera-only net 557.79 vs 557.39 mm, VALID fraction 0.848 vs 0.839, and mono tilt 4.90 vs 4.56 deg. The current run has no >=30 deg VALID geometry event, gate telemetry reports evaluated=26/rejected=0, and late post-peak loss is only 1.1 mm (0.7 mm more than reference). Therefore the ~482 mm result is not caused by the diagnostic gate and does not reproduce the 417.6 mm failure.

The 417.6 mm run remains a transient accepted-pose/front-end/backend failure case already localized by V44.15/V44.16; it must not be used to tune focal scale. The remaining clean-run endpoint residual is about -14 to -18 mm (~3%). V44.25 compares cumulative along-track shape for reference, bad, gated1, and current archives before deciding whether that clean residual is sufficiently repeatable for a production correction.


### V44.25 result — endpoint residual repeats, but keyframe-progress shape does not

Reference/current endpoints remain close (486.07/482.11 mm; mean 484.09 mm, spread 3.97 mm), confirming a repeatable clean-run endpoint shortfall near 3%. However their keyframe-count-normalized cumulative trajectories are not close through the middle: current-reference reaches roughly +35 mm around 45–65% before converging to -4 mm at the endpoint. Therefore V44.25 does **not** justify applying 500/484 as a production scale coefficient. The progress axis is confounded by different keyframe timing/count (33 vs 31 backend rows and different motion duration).

V44.26 re-normalizes the two clean runs by observed camera-only cumulative progress and interpolates backend displacement at equal camera progress. This tests whether the mid-run shape disagreement is mostly a time/keyframe-normalization artifact and whether the remaining endpoint residual is genuinely downstream of raw image displacement.


## 2026-09-09 — New direction: OF navigation improvement, step 1/8

Stopped extending the V44 forensic chain as the primary development loop. Started an explicit 8-step improvement plan aimed at a potentially production-capable local navigation architecture:

OV9281 optical flow + TF-Luna height + FC gyro/attitude -> metric Vx,Vy -> integration/fusion.

Step counter is fixed in `docs/OF_NAV_IMPROVEMENT_PLAN.md`. Step 1/8 audits whether existing archived runs contain enough data for deterministic offline replay, so the first algorithm iterations can avoid new physical passes. The plan has a stop-rule at step 5: if repeatable improvement is not demonstrated, switch to additional hardware rather than continuing tuning indefinitely.


### OF navigation improvement — step 1/8 PASS, step 2/8 started

Step 1 confirms the clean 20260909_141831 archive contains camera forensic motion, logged height, frontend PIM/rotation evidence, backend and leg bounds. Raw OV9281 frame references are not archived, so the first offline estimator will reuse the already-computed median pixel flow rather than pretend to re-run a new pixel tracker.

Step 2 adds a production-candidate offline estimator independent of Kimera backend state:
median OV9281 flow -> subtract FC-attitude-predicted rotational image flow -> scale every sample by TF-Luna height -> integrate metric XY.

The same script also reports affine, uncorrected median-flow, rotation-corrected dynamic-height, and fixed physical-height variants in one execution. This follows the project rule of testing maximum relevant hypotheses per run. No empirical 500-mm correction coefficient is applied.


### OF navigation improvement — step 2/8 complete, step 3/8 started

Step 2 candidate result on clean run 20260909_141831:
- affine + logged height: 557.39 mm;
- median flow + logged height: 554.73 mm;
- median flow + FC rotation + logged height: 548.84 mm;
- median flow + FC rotation + fixed physical 185.5 mm: 531.42 mm.

This is an improvement over raw camera-only, but still not sufficient.

Before changing scale, Step 3 found a concrete implementation risk: the Step-2 script inherited the older hard-coded R_BC used by V44 analyzers, while current `params/JTZeroMonoFLU/LeftCameraParams.yaml` contains a materially different camera-to-body rotation. Step 3 now compares legacy and current extrinsics on the SAME archive, with both logged and fixed height, and quantifies the rotation disagreement. No new physical run and no empirical scale correction.


### OF navigation improvement — step 3/8 result

Same-run geometry audit:
- legacy vs current camera extrinsic rotation differs by 5.701 deg;
- nevertheless logged-height endpoint changes only 548.84 -> 548.75 mm;
- fixed-185.5-mm endpoint changes only 531.42 -> 531.33 mm.

Conclusion: stale R_BC was a real code hygiene defect but is not the source of the remaining metric bias. Current fixed-height residual is +31.33 mm (+6.27%). The exact-height diagnostic gives 174.56 mm, about 10.94 mm below the supplied 185.5-mm physical height; this must not be adopted as calibration without resolving what each height represents geometrically.

Step 4/8: audit height semantics and planar projection geometry on existing data before any scale tuning or new physical pass.


### OF navigation improvement — step 4/8 implementation

Step 4 now tests a physically explicit planar model instead of another scalar-height sweep. For each logged median-flow correspondence it:
1. undistorts both pixel endpoints using current OV9281 radtan calibration;
2. rotates camera rays into world using current T_BS and FC attitude at both frame boundaries;
3. intersects each ray with a horizontal ground plane using either fixed 185.5-mm camera height or logged height;
4. obtains camera translation from the difference between the two ground-intersection vectors.

This tests in one pass: small-angle projection vs exact projective geometry, distortion OFF/ON, fixed physical height vs logged height, and the maximum possible slant-range correction from body tilt. No Kimera state and no empirical 500-mm scale coefficient are used.


### OF navigation improvement — step 4/8 result

STEP 4 CLOSED on archive 20260909_141831_v43_CAMERA_AFFINE_FORENSIC.

Exact planar ray/ground intersection did not remove the residual. Best branch was fixed 185.5-mm height with distortion OFF: 534.50 mm for 500 mm truth (+6.90%). Distortion ON produced 534.90 mm (+6.98%), so current radtan correction changes the endpoint by only +0.40 mm. Logged height worsened the result to 550.33–550.74 mm (+10.1%). FC body tilt was <=2.32 deg; slant-range cos(tilt) correction is only ~0.034% on average and cannot explain the residual.

Conclusion: do not spend another iteration on planar projection, radtan distortion, or slant-range correction for this archive. The remaining +6.9% is not explained by those branches. Step 5 must test whether the residual is repeatable across independent clean physical passes before any production calibration coefficient or hardware change is justified.
