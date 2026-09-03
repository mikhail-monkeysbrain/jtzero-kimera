#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
SRC="${ROOT}/tools/analyze_yaw_rotation_parallax_v15_45.cpp"
BIN="/tmp/jtzero_analyze_yaw_rotation_parallax_v15_45"
OUT="/home/vio/jtzero_rotation_parallax_v15_45.csv"

cd "$ROOT"
for f in "$CAM" "$MJPG" "$ATT" "$SRC"; do
  [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }
done

echo "============================================================"
echo "JT-ZERO v15.45 OFFLINE ROTATION/PARALLAX DIAGNOSTIC"
echo "Dataset: v15.42"
echo "No physical test. No production source/parameter changes."
echo "============================================================"

g++ -std=c++17 -O2 \
  $(pkg-config --cflags opencv4) \
  "$SRC" -o "$BIN" \
  $(pkg-config --libs opencv4)

"$BIN" "$CAM" "$MJPG" "$ATT" "$OUT"
