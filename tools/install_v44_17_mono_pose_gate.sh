#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
REPO="$HOME/jtzero-kimera-sync"
PATCH="$REPO/patches/kimera_v44_17_mono_pose_gate.patch"

echo "======================================================================"
echo "V44.17 — INSTALL OPT-IN MONO POSE PRE-FUSION GATE"
echo "======================================================================"
echo "Kimera source: $ROOT"
echo "Patch:         $PATCH"
echo

if [[ ! -d "$ROOT/.git" ]]; then
  echo "ERROR: Kimera source git tree not found: $ROOT"
  exit 1
fi

cd "$ROOT"

if grep -q 'JTZERO-MONO-POSE-GATE' src/pipeline/MonoImuPipeline.cpp; then
  echo "Patch already present."
else
  echo "===== PATCH CHECK ====="
  git apply --check "$PATCH"
  echo "PATCH CHECK PASS"
  git apply "$PATCH"
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'JTZERO_MONO_POSE_GATE|JTZERO-MONO-POSE-GATE|status_for_backend'   src/pipeline/MonoImuPipeline.cpp

echo
echo "===== BUILD ====="
cmake --build build -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with gate support."
echo "Default remains OFF."
echo
echo "For the next diagnostic run:"
echo "  export JTZERO_MONO_POSE_GATE=1"
echo "  export JTZERO_MONO_POSE_GATE_JUMP_DEG=30"
echo "  export JTZERO_MONO_POSE_GATE_TILT_DEG=30"
echo
echo "To return to production behavior without rebuilding:"
echo "  unset JTZERO_MONO_POSE_GATE"
