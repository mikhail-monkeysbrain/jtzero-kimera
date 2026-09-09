#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/jtzero-kimera-sync"

export JTZERO_MONO_POSE_GATE=1
export JTZERO_MONO_POSE_GATE_JUMP_DEG=30
export JTZERO_MONO_POSE_GATE_TILT_DEG=30

echo "======================================================================"
echo "V44.17 — ONE A->B WITH OPT-IN MONO POSE GATE"
echo "======================================================================"
echo "gate=$JTZERO_MONO_POSE_GATE jump=$JTZERO_MONO_POSE_GATE_JUMP_DEG tilt=$JTZERO_MONO_POSE_GATE_TILT_DEG"
echo "This changes only the diagnostic run. Stored camera intrinsics are unchanged."
echo

bash tools/run_v43_camera_affine_forensic_single.sh
