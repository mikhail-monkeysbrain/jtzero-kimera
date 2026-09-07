#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
TMP="/tmp/jtzero_v25_frontend_5pt_params"
BIN="/tmp/live_mono_imu_500mm_repeat_hud_v25_5pt"
LABEL="${1:-FRONTEND_5PT_DIAG}"

rm -rf "$TMP"
cp -a "$BASE" "$TMP"
sed -i 's/^ransac_use_2point_mono:.*/ransac_use_2point_mono: 0/' "$TMP/FrontendParams.yaml"

export JTZERO_V25_PARAMS="$TMP"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V25_BIN="$BIN"

echo "[5PT] params=$TMP"
grep -nE 'ransac_use_2point_mono|optical_flow_predictor_type' "$TMP/FrontendParams.yaml"
echo "[5PT] rebuilding dedicated V25 binary..."
bash "$ROOT/tools/build_v25.sh" "$BIN"

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh"
rc=$?
set -e

echo
echo "[5PT] run exit code=$rc"
echo "[5PT] archiving V25 outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$TMP")"
echo "[5PT] archive=$archive_path"
exit "$rc"
