#!/usr/bin/env bash
set -euo pipefail
LABEL="${1:-run}"
ROOT="${HOME}/jtzero-kimera-sync"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${HOME}/jtzero_runs/${STAMP}_v22_${LABEL}"
mkdir -p "$OUT"
copy_if_exists(){ local f="$1"; [[ -e "$f" ]] && cp -a "$f" "$OUT/"; }
for f in \
  "$HOME/jtzero_500mm_v18.csv" \
  "$HOME/jtzero_500mm_v18_camera.csv" \
  "$HOME/jtzero_500mm_v18_attitude.csv" \
  "$HOME/jtzero_500mm_v18_range.csv" \
  "$HOME/jtzero_500mm_v18_legs.csv" \
  "$HOME/jtzero_500mm_v22_backend.csv" \
  "$HOME/jtzero_500mm_v22_frontend.csv" \
  "$HOME/jtzero_500mm_v22_events.csv"; do copy_if_exists "$f"; done
{
  echo "label=$LABEL"
  echo "timestamp=$STAMP"
  cd "$ROOT"
  echo "jtzero_branch=$(git branch --show-current)"
  echo "jtzero_head=$(git rev-parse HEAD)"
  echo "mode=V22 pure FRD->FLU; ZXY OFF; gravity feedback OFF"
  echo "jtzero_status_begin"; git status --short; echo "jtzero_status_end"
} > "$OUT/METADATA.txt"
cp -a "$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003" "$OUT/params"
echo "$OUT"
