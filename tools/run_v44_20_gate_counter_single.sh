#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/jtzero-kimera-sync"

export JTZERO_MONO_POSE_GATE=1
export JTZERO_MONO_POSE_GATE_JUMP_DEG=30
export JTZERO_MONO_POSE_GATE_TILT_DEG=30

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/jtzero_v44_20_gate_${STAMP}.log"

echo "======================================================================"
echo "V44.20 — ONE A->B WITH PERSISTENT GATE COUNTERS"
echo "======================================================================"
echo "log=$LOG"
echo "gate=$JTZERO_MONO_POSE_GATE jump=$JTZERO_MONO_POSE_GATE_JUMP_DEG tilt=$JTZERO_MONO_POSE_GATE_TILT_DEG"
echo

set +e
bash tools/run_v43_camera_affine_forensic_single.sh 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
set -e

echo
echo "===== GATE EVIDENCE ====="
grep -E 'JTZERO-MONO-POSE-GATE(\]|-SUMMARY\])' "$LOG" || true

echo
echo "===== REQUIRED SUMMARY ====="
SUMMARY=$(grep 'JTZERO-MONO-POSE-GATE-SUMMARY' "$LOG" | tail -1 || true)
if [[ -z "$SUMMARY" ]]; then
  echo "ERROR: no persistent gate summary found."
  echo "Do not interpret the endpoint as a gate result."
  exit 3
fi
echo "$SUMMARY"

exit "$RC"
