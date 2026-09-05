#!/usr/bin/env bash
set -euo pipefail

LABEL="${1:-run}"
ROOT="${HOME}/jtzero-kimera-sync"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${HOME}/jtzero_runs/${STAMP}_v21_${LABEL}"
mkdir -p "$OUT"

copy_if_exists() {
  local f="$1"
  [[ -e "$f" ]] && cp -a "$f" "$OUT/"
}

for f in   "$HOME/jtzero_500mm_v18.csv"   "$HOME/jtzero_500mm_v18_camera.csv"   "$HOME/jtzero_500mm_v18_attitude.csv"   "$HOME/jtzero_500mm_v18_range.csv"   "$HOME/jtzero_500mm_v18_legs.csv"   "$HOME/jtzero_500mm_v21_backend.csv"   "$HOME/jtzero_500mm_v21_frontend.csv"   "$HOME/jtzero_500mm_v21_events.csv"; do
  copy_if_exists "$f"
done

{
  echo "label=$LABEL"
  echo "timestamp=$STAMP"
  echo "repo=$ROOT"
  cd "$ROOT"
  echo "jtzero_branch=$(git branch --show-current)"
  echo "jtzero_head=$(git rev-parse HEAD)"
  echo "jtzero_status_begin"
  git status --short
  echo "jtzero_status_end"
} > "$OUT/METADATA.txt"

if [[ -d "$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003" ]]; then
  cp -a "$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003" "$OUT/params"
fi

echo "$OUT"
