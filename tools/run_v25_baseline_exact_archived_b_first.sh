#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_repeat_hud_v25_bfirst"
LABEL="${1:-BASELINE_ARW_003_EXACT_B_FIRST}"

export JTZERO_V25_PARAMS="$PARAMS"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V25_BIN="$BIN"

echo "[B-FIRST] params=$PARAMS"
grep -nE 'accelerometer_random_walk|accelerometer_noise_density|gyroscope_random_walk' "$PARAMS/ImuParams.yaml"

echo "[B-FIRST] rebuilding dedicated V25 binary..."
bash "$ROOT/tools/build_v25.sh" "$BIN"

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh" B_FIRST
rc=$?
set -e

echo
echo "[B-FIRST] run exit code=$rc"
echo "[B-FIRST] archiving V25 outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$PARAMS")"
echo "[B-FIRST] archive=$archive_path"

exit "$rc"
