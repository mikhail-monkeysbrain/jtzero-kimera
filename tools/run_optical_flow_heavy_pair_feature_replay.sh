#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
DATASET="${1:-/home/vio/jtzero_runs/20260913_102506_OF_FEATURE_REPLAY}"
YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml}"
SCALE="${JTZERO_FLOW_FOCAL_SCALE:-0.931}"
TOPN="${2:-30}"
BIN=/tmp/jtzero_of_heavy_feature_replay
MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }
[[ -f "$DATASET/frames.csv" && -f "$DATASET/frames.mjpgbin" ]] || { echo "ОШИБКА: dataset не найден: $DATASET" >&2; exit 2; }

echo "Собираю heavy-pair replay..."
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member  $(pkg-config --cflags opencv4) $MAVLINK_INC  "$ROOT/tools/optical_flow_heavy_pair_feature_replay.cpp"  -o "$BIN" $(pkg-config --libs opencv4) -lpthread

"$BIN" "$DATASET" "$YAML" "$SCALE" "$TOPN"
