#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/jtzero-kimera-sync"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/jtzero_v44_23_gate_${STAMP}.log"
STATS="$HOME/jtzero_v44_23_gate_stats_${STAMP}.txt"

export JTZERO_MONO_POSE_GATE=1
export JTZERO_MONO_POSE_GATE_JUMP_DEG=30
export JTZERO_MONO_POSE_GATE_TILT_DEG=30
export JTZERO_MONO_POSE_GATE_STATS_FILE="$STATS"

echo "======================================================================"
echo "V44.23 — ONE A->B / LIVE GATE-EVIDENCE RUN"
echo "======================================================================"
echo "Exactly ONE physical A->B 500 mm pass."
echo "Gate evidence is read from crash-safe stats, not destructor output."
echo "stats=$STATS"

set +e
bash tools/run_v43_camera_affine_forensic_single.sh 2>&1 | tee "$LOG"
WRAPPER_RC=${PIPESTATUS[0]}
set -e

echo; echo "===== WRAPPER RC ====="; echo "$WRAPPER_RC"
echo; echo "===== CRASH-SAFE GATE STATS ====="
if [[ -f "$STATS" ]]; then cat "$STATS"; else echo "ERROR: STATS FILE MISSING"; exit 2; fi

line=$(tail -n 1 "$STATS")
get_field() {
  tr ' ' '\n' <<<"$line" | awk -F= -v key="$1" '$1 == key {print $2; exit}'
}
keyframes=$(get_field keyframes_seen)
valid=$(get_field valid_seen)
evaluated=$(get_field evaluated)
rejected=$(get_field rejected)
max_jump=$(get_field max_jump_deg)
max_tilt=$(get_field max_tilt_deg)

echo; echo "===== V44.23 DISCRIMINATOR ====="
echo "keyframes_seen=${keyframes:-?} valid_seen=${valid:-?} evaluated=${evaluated:-?} rejected=${rejected:-?}"
echo "max_jump_deg=${max_jump:-?} max_tilt_deg=${max_tilt:-?}"

if [[ -z "${keyframes:-}" || "$keyframes" -eq 0 ]]; then echo "FAIL: callback did not process keyframes."; exit 3; fi
if [[ -z "${valid:-}" || "$valid" -eq 0 || -z "${evaluated:-}" || "$evaluated" -eq 0 ]]; then
  echo "FAIL: no VALID mono pose was evaluated; do NOT interpret endpoint as a gate test."; exit 4
fi
if [[ "$evaluated" -ne "$valid" ]]; then
  echo "WARNING: evaluated != valid_seen; send complete output before causal interpretation."
else
  echo "PASS: every observed VALID keyframe reached the gate evaluator."
fi
if [[ -n "${rejected:-}" && "$rejected" -gt 0 ]]; then
  echo "GATE FIRED: anomalous VALID pose(s) rejected before backend fusion."
else
  echo "GATE DID NOT FIRE: no pose crossed both configured thresholds in this run."
fi

echo; echo "===== GATE / TEARDOWN LINES ====="
grep -E 'JTZERO-MONO-POSE-GATE|terminate called|Aborted' "$LOG" || true
echo; echo "Send the complete terminal output. Do not perform a second physical pass."
