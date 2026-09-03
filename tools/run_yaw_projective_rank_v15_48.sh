#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/jtzero-kimera-sync"
SRC="${ROOT}/tools/analyze_yaw_projective_rank_v15_48.cpp"
BIN="/tmp/analyze_yaw_projective_rank_v15_48"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
OUT="/home/vio/jtzero_projective_rank_v15_48.csv"
cd "$ROOT"
for f in "$SRC" "$CAM" "$MJPG" "$ATT"; do [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }; done

echo "============================================================"
echo "JT-ZERO v15.48 PROJECTIVE RANK TEST"
echo "Dataset: v15.42"
echo "Tests whether H - closest pure rotation is approximately rank-1"
echo "No physical test. No production changes."
echo "============================================================"

g++ -std=c++17 -O2 "$SRC" -o "$BIN" $(pkg-config --cflags --libs opencv4)
"$BIN" "$CAM" "$MJPG" "$ATT" "$OUT"
