#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export JTZERO_FLOW_CAMERA_YAML="$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml"
export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.1060}"
export JTZERO_FLOW_TARGET_MM="${JTZERO_FLOW_TARGET_MM:-300}"
export JTZERO_FLOW_GUIDED_MODE="guided"
export JTZERO_FLOW_PRE_STATIC_SEC="${JTZERO_FLOW_PRE_STATIC_SEC:-5}"
export JTZERO_FLOW_POST_STATIC_SEC="${JTZERO_FLOW_POST_STATIC_SEC:-8}"
export JTZERO_FLOW_FEATURE_ROI="${JTZERO_FLOW_FEATURE_ROI:-0.20 0.32 0.80 0.90}"

cat <<EOF
======================================================================
JT-ZERO — CURRENT-MOUNT TABLE DISTANCE TEST
======================================================================
Purpose:
  validate the CURRENT physical OV9281 orientation and ArduPilot native
  FLOW_POS / RNGFND1_POS lever-arm compensation before the new housing.

No synthetic 0.60 m height.
TF-Luna publishes its real measured distance.
Current-mount camera YAML:
  $JTZERO_FLOW_CAMERA_YAML

Nominal movement instruction: $JTZERO_FLOW_TARGET_MM mm.
Measure the ACTUAL movement with a ruler and report it together with terminal output.

Temporary stand-occlusion feature ROI:
  $JTZERO_FLOW_FEATURE_ROI
  (excludes the visible upper stand/arch and left edge; final housing should not need this)

Keep vehicle level:
  - translate only;
  - no yaw;
  - no roll/pitch;
  - no lift.
======================================================================
EOF

read -r RX0 RY0 RX1 RY1 <<< "$JTZERO_FLOW_FEATURE_ROI"
export JTZERO_FLOW_EXTRA_ARGS="--feature-roi $RX0 $RY0 $RX1 $RY1"
exec bash tools/run_optical_flow_mavlink_bench.sh
