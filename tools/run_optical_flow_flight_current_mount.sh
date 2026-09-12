#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Validated on 2026-09-12 current temporary mount:
# 500 mm physical -> RAW camera-height 501.2 mm; EKF 489.3 mm.
export JTZERO_FLOW_CAMERA_YAML="$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml"
export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-0.931}"
export JTZERO_FLOW_FEATURE_ROI="${JTZERO_FLOW_FEATURE_ROI:-0.20 0.32 0.80 0.90}"
export JTZERO_FLOW_DIAG_CAMERA_Z_M="${JTZERO_FLOW_DIAG_CAMERA_Z_M:-0.050}"
export JTZERO_FLOW_DIAG_RANGE_Z_M="${JTZERO_FLOW_DIAG_RANGE_Z_M:-0.026}"

bash "$ROOT/tools/audit_optical_flow_current_mount_geometry.sh"

echo
echo "CURRENT-MOUNT PROFILE:"
echo "  focal_scale = $JTZERO_FLOW_FOCAL_SCALE"
echo "  feature ROI = $JTZERO_FLOW_FEATURE_ROI"
echo "  camera YAML = $JTZERO_FLOW_CAMERA_YAML"
echo
echo "ВАЖНО: этот профиль относится только к текущему временному монтажу."
echo "После установки нового корпуса геометрию и offsets нужно переснять."
echo

exec bash "$ROOT/tools/run_optical_flow_flight.sh"
