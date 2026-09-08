#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
LABEL="${1:-CONTROLLED_AB4_REPEATABILITY}"

export JTZERO_V25_PARAMS="$PARAMS"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V25_MODE_DESC="V26 CONTROLLED_AB4: four measured A->B passes; B->A returns unmeasured; SPACE starts/stops motion; pass 1 timing reference; tolerance +/-1.5 s"

rm -f "$HOME"/jtzero_500mm_v26_[1-4]AB.csv

echo "[V26] params=$PARAMS"
echo "[V26] mode=CONTROLLED_AB4"
echo "[V26] 1AB defines timing reference; 2AB..4AB should finish within +/-1.5 s"
echo "[V26] rebuilding binary..."
bash "$ROOT/tools/build_v25.sh" /tmp/live_mono_imu_500mm_repeat_hud_v25

set +e
bash "$ROOT/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
rc=$?
set -e

echo
echo "[V26] run exit code=$rc"
echo "[V26] archiving outputs..."
archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$LABEL" "$PARAMS")"
echo "[V26] archive=$archive_path"

for f in "$HOME"/jtzero_500mm_v26_[1-4]AB.csv; do
  [[ -e "$f" ]] && echo "[V26] comparison CSV: $f"
done

exit "$rc"
