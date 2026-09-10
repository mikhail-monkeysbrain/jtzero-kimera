#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${JTZERO_GM_BIN:-/tmp/jtzero_ground_motion_mvp}"

find_ov9281_camera() {
  local byid="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
  if [[ -e "$byid" ]]; then
    readlink -f "$byid"
    return 0
  fi

  local dev
  while read -r dev; do
    [[ -n "$dev" ]] || continue
    if v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
      echo "$dev"
      return 0
    fi
  done < <(
    v4l2-ctl --list-devices 2>/dev/null |
      awk '/Arducam OV9281 USB Camera/{found=1; next} found && /^[[:space:]]*\/dev\/video/{print $1} found && NF==0{exit}'
  )

  return 1
}

if [[ -n "${JTZERO_GM_CAMERA:-}" ]]; then
  CAM="$JTZERO_GM_CAMERA"
else
  CAM="$(find_ov9281_camera)" || {
    echo "ОШИБКА: OV9281 capture node не найден" >&2
    exit 1
  }
fi

LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
YAML="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
RUN_DIR="${JTZERO_RUN_DIR:-/home/vio/jtzero_runs}"
mkdir -p "$RUN_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV="$RUN_DIR/${STAMP}_GROUND_MOTION_MVP.csv"
[[ -x "$BIN" ]] || "$ROOT/tools/build_ground_motion_mvp.sh" "$BIN"
echo "JT-ZERO Ground Motion MVP"
echo "camera=$CAM luna=$LUNA fc=$FC"
echo "csv=$CSV"
exec "$BIN" "$CAM" "$LUNA" "$FC" "$CSV" "$YAML" "$OFFSET"
