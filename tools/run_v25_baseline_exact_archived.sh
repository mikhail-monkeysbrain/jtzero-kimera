#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
LABEL="${1:-BASELINE_ARW_003_EXACT}"

export JTZERO_V25_PARAMS="$PARAMS"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"

echo "[BASELINE] params=$PARAMS"
grep -nE 'accelerometer_random_walk|accelerometer_noise_density|gyroscope_random_walk'   "$PARAMS/ImuParams.yaml"

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh"
rc=$?
set -e

echo
echo "[BASELINE] run exit code=$rc"
echo "[BASELINE] archiving V25 outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$PARAMS")"
echo "[BASELINE] archive=$archive_path"

exit "$rc"
