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

Keep vehicle level:
  - translate only;
  - no yaw;
  - no roll/pitch;
  - no lift.
======================================================================
EOF

exec bash tools/run_optical_flow_mavlink_bench.sh
