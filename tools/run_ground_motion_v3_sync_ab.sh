#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/jtzero_ground_motion_v3_sync_ab"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${JTZERO_GM_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_V3_SYNC_AB.csv}"
mkdir -p "$(dirname "$OUT")"
if [[ ! -x "$BIN" || "$ROOT/tools/ground_motion_live_v3_sync_ab.cpp" -nt "$BIN" ]]; then
  bash "$ROOT/tools/build_ground_motion_v3_sync_ab.sh" "$BIN"
fi
echo "======================================================================"
echo "JT-ZERO GROUND MOTION V3 TIMING A/B"
echo "LATEST: текущий последний ATTITUDE"
echo "SYNC: ATTITUDE интерполирован к V4L2 timestamp кадра"
echo "Luna одинакова в обеих ветках — изолируем timing ATTITUDE"
echo "Камера: $CAMERA"
echo "TF-Luna: $LUNA"
echo "FC: $FC"
echo "CSV: $OUT"
echo "======================================================================"
"$BIN" "$CAMERA" "$LUNA" "$FC" "$OUT" "$PARAMS" "$OFFSET"
RC=$?
echo "CSV: $OUT"
exit "$RC"
