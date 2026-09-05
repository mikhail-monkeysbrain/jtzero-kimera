#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${JTZERO_V25_BIN:-/tmp/live_mono_imu_500mm_repeat_hud_v25}"
PARAMS="${JTZERO_V25_PARAMS:-$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003}"
KIMERA_BUILD="${KIMERA_BUILD:-/home/vio/Kimera-VIO/build}"

find_camera() {
  local d real card

  # Prefer stable udev symlink when present. video-index0 is the real image
  # stream; video-index1 is normally UVC metadata.
  if [ -d /dev/v4l/by-id ]; then
    for d in /dev/v4l/by-id/*Arducam*video-index0 /dev/v4l/by-id/*OV9281*video-index0; do
      [ -e "$d" ] || continue
      real="$(readlink -f "$d")"
      if v4l2-ctl -d "$real" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
        printf '%s\n' "$real"
        return 0
      fi
    done
  fi

  # Fallback: find the UVC image node by card name/capabilities.
  for d in /dev/video*; do
    [ -e "$d" ] || continue
    card="$(v4l2-ctl -d "$d" --all 2>/dev/null | sed -n 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*//p' | head -1)"
    case "$card" in
      *Arducam*OV9281*|*OV9281*USB*|*Arducam*)
        if v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -q "'MJPG'" &&
           v4l2-ctl -d "$d" --all 2>/dev/null | grep -A20 'Device Caps' | grep -q 'Video Capture'; then
          printf '%s\n' "$d"
          return 0
        fi
        ;;
    esac
  done
  return 1
}

CAMERA="$(find_camera || true)"
if [ -z "$CAMERA" ]; then
  echo "ERROR: Arducam OV9281 image stream not found." >&2
  echo "Detected V4L2 devices:" >&2
  v4l2-ctl --list-devices >&2 || true
  exit 2
fi

echo "[AUTO-CAM] selected $CAMERA"
v4l2-ctl -d "$CAMERA" --all 2>/dev/null |   grep -E 'Driver name|Card type|Video input' | head -3 || true

if [ ! -x "$BIN" ]; then
  echo "[AUTO-CAM] V25 binary missing, rebuilding..."
  "$ROOT/tools/build_v25.sh" "$BIN"
fi

export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export LD_LIBRARY_PATH="$KIMERA_BUILD:/usr/local/lib:${LD_LIBRARY_PATH:-}"

exec "$BIN" "$PARAMS" "$CAMERA" "$@"
