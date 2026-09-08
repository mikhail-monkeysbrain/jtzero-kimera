#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
LABEL="${1:-CONTROLLED_AB4_FAST_5S}"

export JTZERO_V25_PARAMS="$PARAMS"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"

export JTZERO_V26_TARGET_SEC=5.0
export JTZERO_V26_TOLERANCE_SEC=0.5
export JTZERO_V25_MODE_DESC="V26 CONTROLLED_AB4 FAST: four measured A->B passes; target 5.0 s; accepted 4.5..5.5 s"

rm -f "$HOME"/jtzero_500mm_v26_[1-4]AB.csv

echo "[V26-FAST] params=$PARAMS"
echo "[V26-FAST] mode=CONTROLLED_AB4"
echo "[V26-FAST] fixed target for ALL passes: 5.0 s; accepted window: 4.5..5.5 s"
echo "[V26-FAST] rebuilding binary..."
bash "$ROOT/tools/build_v25.sh" /tmp/live_mono_imu_500mm_repeat_hud_v25

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
rc=$?
set -e

echo
echo "[V26-FAST] run exit code=$rc"
echo "[V26-FAST] archiving outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$PARAMS")"
echo "[V26-FAST] archive=$archive_path"

for f in "$HOME"/jtzero_500mm_v26_[1-4]AB.csv; do
  [[ -e "$f" ]] && echo "[V26-FAST] comparison CSV: $f"
done

exit "$rc"
