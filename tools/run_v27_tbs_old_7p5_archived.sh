#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
TMP="/tmp/JTZeroMonoFLU_TBS_Rold_7p5"
LABEL="${1:-TBS_ROLD_7P5}"

rm -rf "$TMP"
cp -a "$BASE" "$TMP"

cat > "$TMP/LeftCameraParams.yaml" <<'YAML'
%YAML:1.0
# JT-Zero T_BS forensic candidate.
# Recovered R_old from:
#   R_new = Rx(-1.5 deg) * Ry(-5.5 deg) * R_old
# Therefore:
#   R_old = Ry(+5.5 deg) * Rx(+1.5 deg) * R_new
# Translation and all camera intrinsics are unchanged.
camera_id: left_cam

T_BS:
  cols: 4
  rows: 4
  data: [ 0.009367370,  0.999954830, -0.001604440,  0.000,
          0.999326770, -0.009418380, -0.035458410,  0.000,
         -0.035471920, -0.001271210, -0.999369870, -0.055,
          0.000000000,  0.000000000,  0.000000000,  1.000 ]

rate_hz: 30
resolution: [640, 480]
camera_model: pinhole
intrinsics: [568.53170752165227, 569.68005562865858, 315.98271077441063, 239.88148589100641]
distortion_model: radial-tangential
distortion_coefficients: [0.073569192194028493, -0.095253893789117, -0.010810530757187299, -0.0022843373576970235, 0.082177400802757483]
YAML

export JTZERO_V25_PARAMS="$TMP"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_V25_MODE_DESC="V27 T_BS R_old forensic: 4 controlled passes, target 7.5 s"

rm -f "$HOME"/jtzero_500mm_v26_[1-4]AB.csv

echo "[V27-TBS] BASE=$BASE"
echo "[V27-TBS] PARAMS=$TMP"
echo "[V27-TBS] ONLY CHANGE: LeftCameraParams.yaml T_BS rotation -> recovered R_old"
echo "[V27-TBS] target 7.5 s; accepted 6.5..8.5 s"
echo "[V27-TBS] rebuilding binary..."
bash "$ROOT/tools/build_v25.sh" /tmp/live_mono_imu_500mm_repeat_hud_v25

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
rc=$?
set -e

echo
echo "[V27-TBS] run exit code=$rc"
echo "[V27-TBS] archiving outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$TMP")"
echo "[V27-TBS] archive=$archive_path"

cp "$TMP/LeftCameraParams.yaml" "$archive_path/LeftCameraParams_TBS_ROLD.yaml" 2>/dev/null || true
for f in "$HOME"/jtzero_500mm_v26_[1-4]AB.csv; do
  [[ -e "$f" ]] && echo "[V27-TBS] comparison CSV: $f"
done

exit "$rc"
