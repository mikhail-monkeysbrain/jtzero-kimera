#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/jtzero-kimera-sync"
SRC="${ROOT}/tools/analyze_yaw_homography_pure_rotation_v15_47.cpp"
BIN="/tmp/jtzero_v15_47"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
OUT="/home/vio/jtzero_homography_purerot_v15_47.csv"
cd "$ROOT"
for f in "$SRC" "$CAM" "$MJPG" "$ATT"; do [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }; done
echo "============================================================"
echo "JT-ZERO v15.47 HOMOGRAPHY vs CLOSEST PURE ROTATION"
echo "Dataset: v15.42"
echo "No physical test. No production changes."
echo "============================================================"
g++ -std=c++17 -O2 $(pkg-config --cflags opencv4) "$SRC" -o "$BIN" $(pkg-config --libs opencv4)
"$BIN" "$CAM" "$MJPG" "$ATT" "$OUT"
