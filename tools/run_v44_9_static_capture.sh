#!/usr/bin/env bash
set -euo pipefail

HEIGHT_MM="${1:-}"
LABEL="${2:-}"
if [[ -z "$HEIGHT_MM" || -z "$LABEL" ]]; then
  echo "Usage: $0 <physical_height_mm> <label>"
  exit 2
fi

NODE="$(v4l2-ctl --list-devices | awk '
  /Arducam OV9281 USB Camera/ {f=1; next}
  f && /\/dev\/video[0-9]+/ {print $1; exit}
')"

if [[ -z "$NODE" ]]; then
  echo "ERROR: OV9281 not found"
  exit 1
fi

OUT="$HOME/jtzero_runs/v44_9_${LABEL}_${HEIGHT_MM}mm"
mkdir -p "$OUT"

cat > "$OUT/METADATA.txt" <<EOF
label=$LABEL
physical_height_mm=$HEIGHT_MM
device=$NODE
width=640
height=480
pixelformat=MJPG
frames=20
EOF

echo "V44.9 static capture"
echo "node=$NODE"
echo "height_mm=$HEIGHT_MM"
echo "out=$OUT"

v4l2-ctl -d "$NODE" --set-fmt-video=width=640,height=480,pixelformat=MJPG >/dev/null
rm -f "$OUT"/frame_*.jpg

ffmpeg -hide_banner -loglevel warning   -f v4l2   -input_format mjpeg   -video_size 640x480   -framerate 100   -i "$NODE"   -frames:v 20   -q:v 2   "$OUT/frame_%02d.jpg"

echo "frames=$(find "$OUT" -maxdepth 1 -name 'frame_*.jpg' | wc -l)"
lsusb | grep '0c45:6366' || echo "WARNING: OV9281 USB LOST AFTER CAPTURE"
echo "DONE: $OUT"
