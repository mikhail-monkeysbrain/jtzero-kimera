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
keyframes=$(sed -n 's/.*keyframes_seen=\([0-9][0-9]*\).*/\1/p' <<<"$line")
valid=$(sed -n 's/.*valid_seen=\([0-9][0-9]*\).*/\1/p' <<<"$line")
evaluated=$(sed -n 's/.*evaluated=\([0-9][0-9]*\).*/\1/p' <<<"$line")
rejected=$(sed -n 's/.*rejected=\([0-9][0-9]*\).*/\1/p' <<<"$line")
max_jump=$(sed -n 's/.*max_jump_deg=\([^ ]*\).*/\1/p' <<<"$line")
max_tilt=$(sed -n 's/.*max_tilt_deg=\([^ ]*\).*/\1/p' <<<"$line")

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
