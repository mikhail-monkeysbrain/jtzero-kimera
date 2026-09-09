#!/usr/bin/env bash
set -euo pipefail
LABEL="${1:-ABA_X2}"
PARAMS_DIR="${2:-${JTZERO_V25_PARAMS:-${HOME}/jtzero-kimera-sync/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003}}"
ROOT="${HOME}/jtzero-kimera-sync"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${HOME}/jtzero_runs/${STAMP}_v25_${LABEL}"
mkdir -p "$OUT"
copy_if_exists(){ local f="$1"; [[ -e "$f" ]] && cp -a "$f" "$OUT/"; }
for f in \
  "$HOME/jtzero_500mm_v25.csv" \
  "$HOME/jtzero_500mm_v25_camera.csv" \
  "$HOME/jtzero_500mm_v25_attitude.csv" \
  "$HOME/jtzero_500mm_v25_range.csv" \
  "$HOME/jtzero_500mm_v25_legs.csv" \
  "$HOME/jtzero_500mm_v25_backend.csv" \
  "$HOME/jtzero_500mm_v25_frontend.csv" \
  "$HOME/jtzero_500mm_v25_events.csv"; do copy_if_exists "$f"; done
for f in "$HOME"/jtzero_500mm_v26_[1-4]AB.csv; do
  [[ -e "$f" ]] && copy_if_exists "$f"
done

BIN="${JTZERO_V25_BIN:-/tmp/live_mono_imu_500mm_v43_camera_forensic}"
KIMERA="${KIMERA_SOURCE:-/home/vio/Kimera-VIO}"
{
  echo "label=$LABEL"
  echo "timestamp=$STAMP"
  cd "$ROOT"
  echo "jtzero_branch=$(git branch --show-current)"
  echo "jtzero_head=$(git rev-parse HEAD)"
  echo "jtzero_dirty_count=$(git status --porcelain | wc -l)"
  echo "mode=${JTZERO_V25_MODE_DESC:-V25 A->B->A x2 closure; strict START; stall >500ms invalid; FRD->FLU only; ZXY OFF; gravity feedback OFF}"
  echo "params_dir=$PARAMS_DIR"
  if [[ -x "$BIN" ]]; then
    echo "binary_path=$BIN"
    echo "binary_sha256=$(sha256sum "$BIN" | awk '{print $1}')"
  else
    echo "binary_path=$BIN"
    echo "binary_sha256=MISSING"
  fi
  if [[ -d "$KIMERA/.git" ]]; then
    echo "kimera_path=$KIMERA"
    echo "kimera_branch=$(git -C "$KIMERA" branch --show-current)"
    echo "kimera_head=$(git -C "$KIMERA" rev-parse HEAD)"
    echo "kimera_dirty_count=$(git -C "$KIMERA" status --porcelain | wc -l)"
  else
    echo "kimera_path=$KIMERA"
    echo "kimera_head=MISSING"
  fi
  echo "jtzero_status_begin"; git status --short; echo "jtzero_status_end"
  if [[ -d "$KIMERA/.git" ]]; then
    echo "kimera_status_begin"; git -C "$KIMERA" status --short; echo "kimera_status_end"
  fi
} > "$OUT/METADATA.txt"

if [[ -d "$PARAMS_DIR" ]]; then
  cp -a "$PARAMS_DIR" "$OUT/params"
  (cd "$OUT" && find params -type f -print0 | sort -z | xargs -0 sha256sum) > "$OUT/PARAMS_SHA256.txt"
else
  echo "WARNING: params dir not found: $PARAMS_DIR" >&2
fi
echo "$OUT"
