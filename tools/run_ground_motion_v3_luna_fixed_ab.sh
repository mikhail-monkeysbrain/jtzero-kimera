#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/jtzero_ground_motion_v3_luna_fixed_ab"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${JTZERO_GM_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_V3_LUNA_FIXED_AB.csv}"
mkdir -p "$(dirname "$OUT")"
if [[ ! -x "$BIN" || "$ROOT/tools/ground_motion_live_v3_luna_fixed_ab.cpp" -nt "$BIN" || "$ROOT/tools/ground_motion_live_v3_lever_ab.cpp" -nt "$BIN" || "$ROOT/tools/ground_motion_live_v2_v3_ab.cpp" -nt "$BIN" ]]; then
  bash "$ROOT/tools/build_ground_motion_v3_luna_fixed_ab.sh"
fi
echo "======================================================================"
echo "JT-ZERO V3 LUNA vs FIXED HEIGHT A/B"
echo "Обе ветки включают CAD lever arm: X=${JTZERO_LEVER_X_MM:-49.16} Y=${JTZERO_LEVER_Y_MM:-0.22} Z=${JTZERO_LEVER_Z_MM:-0} mm"
echo "FIXED H автоматически фиксируется по медиане H камеры перед стартом."
echo "Камера: $CAMERA"
echo "TF-Luna: $LUNA"
echo "FC: $FC"
echo "CSV: $OUT"
echo "======================================================================"
"$BIN" "$CAMERA" "$LUNA" "$FC" "$OUT" "$PARAMS" "$OFFSET"
RC=$?
echo "CSV: $OUT"
exit "$RC"
