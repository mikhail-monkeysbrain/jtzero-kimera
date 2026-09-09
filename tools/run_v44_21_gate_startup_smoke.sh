#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/jtzero-kimera-sync"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/jtzero_v44_21_smoke_${STAMP}.log"
STATS="$HOME/jtzero_v44_21_gate_stats_${STAMP}.txt"

export JTZERO_MONO_POSE_GATE=1
export JTZERO_MONO_POSE_GATE_JUMP_DEG=30
export JTZERO_MONO_POSE_GATE_TILT_DEG=30
export JTZERO_MONO_POSE_GATE_STATS_FILE="$STATS"

echo "======================================================================"
echo "V44.21 — GATE STARTUP/WARMUP SMOKE TEST"
echo "======================================================================"
echo "Do NOT move the stand."
echo "This is NOT a 500 mm pass."
echo "log=$LOG"
echo "stats=$STATS"
echo

set +e
bash tools/run_v43_camera_affine_forensic_single.sh 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
set -e

echo
echo "===== PROCESS RC ====="
echo "$RC"

echo
echo "===== CRASH-SAFE GATE STATS ====="
if [[ -f "$STATS" ]]; then
  cat "$STATS"
else
  echo "STATS FILE MISSING"
fi

echo
echo "===== TERMINAL GATE LINES ====="
grep -E 'JTZERO-MONO-POSE-GATE|terminate called|Aborted|what\(\)|exception' "$LOG" || true

echo
echo "===== RECENT KERNEL SIGNAL/SEGFAULT LINES ====="
sudo dmesg -T | grep -iE 'segfault|general protection|trap|oom|killed process' | tail -30 || true

exit 0
