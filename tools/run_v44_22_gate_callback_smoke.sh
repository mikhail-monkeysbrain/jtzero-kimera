#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/jtzero-kimera-sync"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/jtzero_v44_22_smoke_${STAMP}.log"
STATS="$HOME/jtzero_v44_22_gate_stats_${STAMP}.txt"

export JTZERO_MONO_POSE_GATE=1
export JTZERO_MONO_POSE_GATE_JUMP_DEG=30
export JTZERO_MONO_POSE_GATE_TILT_DEG=30
export JTZERO_MONO_POSE_GATE_STATS_FILE="$STATS"

echo "======================================================================"
echo "V44.22 — STATIONARY CALLBACK/STATUS SMOKE"
echo "======================================================================"
echo "DO NOT MOVE."
echo "Wait until the GUI reaches the ready state, then Q/ESC."
echo "stats=$STATS"
echo

set +e
bash tools/run_v43_camera_affine_forensic_single.sh 2>&1 | tee "$LOG"
WRAPPER_RC=${PIPESTATUS[0]}
set -e

echo
echo "===== WRAPPER RC ====="
echo "$WRAPPER_RC"

echo
echo "===== CRASH-SAFE STATS ====="
if [[ -f "$STATS" ]]; then
  cat "$STATS"
else
  echo "STATS FILE MISSING"
fi

echo
echo "===== CALLBACK PROOF ====="
if [[ -f "$STATS" ]] && grep -Eq 'keyframes_seen=[1-9][0-9]*' "$STATS"; then
  echo "PASS: frontend callback processed keyframes with gate enabled."
else
  echo "FAIL: no keyframe callback evidence."
fi

echo
echo "===== TEARDOWN ====="
grep -E 'terminate called|Aborted|JTZERO-MONO-POSE-GATE-SUMMARY' "$LOG" || true

exit 0
