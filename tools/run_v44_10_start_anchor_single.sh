#!/usr/bin/env bash
set -euo pipefail
ROOT="$HOME/jtzero-kimera-sync"
STAMP=$(date +%Y%m%d_%H%M%S)
ANCHOR="$HOME/jtzero_runs/${STAMP}_v44_10_START_ANCHOR"
mkdir -p "$ANCHOR"

NODE="$(v4l2-ctl --list-devices | awk '
  /Arducam OV9281 USB Camera/ {f=1; next}
  f && /\/dev\/video[0-9]+/ {print $1; exit}
')"
if [[ -z "$NODE" ]]; then
  echo "ERROR: OV9281 not found"
  exit 1
fi

echo "======================================================================"
echo "V44.10 — START-BOARD ANCHORED ONE-PASS TEST"
echo "ФИЗИЧЕСКИХ ПРОХОДОВ A->B: РОВНО 1"
echo
echo "ВАЖНО: ChArUco нужна ТОЛЬКО в точке A."
echo "После начала движения доска может полностью исчезнуть из кадра."
echo "Высоту стенда не менять."
echo "======================================================================"
echo
echo "1) Стенд в A, ChArUco лежит в начале пути и видна OV9281."
echo "2) Ничего не двигать во время 20 стартовых кадров."
read -r -p "ENTER — снять стартовый ChArUco anchor: " _

v4l2-ctl -d "$NODE" --set-fmt-video=width=640,height=480,pixelformat=MJPG >/dev/null
ffmpeg -hide_banner -loglevel warning   -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 100   -i "$NODE" -frames:v 20 -q:v 2 "$ANCHOR/frame_%02d.jpg"

echo "[V44.10] start-anchor frames: $(find "$ANCHOR" -name 'frame_*.jpg' | wc -l)"
lsusb | grep '0c45:6366' >/dev/null || { echo "ERROR: OV9281 lost after anchor capture"; exit 1; }

BEFORE="$(find "$HOME/jtzero_runs" -maxdepth 1 -type d -name '*_v43_CAMERA_AFFINE_FORENSIC' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"

echo
echo "3) Доску НЕ переносить: она остаётся только у A."
echo "4) Сейчас будет стандартный ОДИН проход A->B 500 мм."
echo
bash "$ROOT/tools/run_v43_camera_affine_forensic_single.sh"

AFTER="$(find "$HOME/jtzero_runs" -maxdepth 1 -type d -name '*_v43_CAMERA_AFFINE_FORENSIC' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -z "$AFTER" || "$AFTER" == "$BEFORE" ]]; then
  echo "ERROR: new V43 archive not found"
  exit 1
fi

echo
echo "===== V44.10 ANALYSIS ====="
/usr/bin/python3 "$ROOT/tools/analyze_v44_10_start_anchor.py"   --run "$AFTER"   --start-glob "$ANCHOR/frame_*.jpg"   --square-mm 26.47   --height-mm 185.5   --truth-mm 500

echo
echo "Anchor: $ANCHOR"
echo "Run:    $AFTER"
