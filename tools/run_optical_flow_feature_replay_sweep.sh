#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml}"
SCALE="${JTZERO_FLOW_FOCAL_SCALE:-0.931}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${1:-/home/vio/jtzero_runs/${STAMP}_OF_FEATURE_REPLAY}"
REC=/tmp/jtzero_of_feature_record
REP=/tmp/jtzero_of_feature_replay
MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }

echo "Собираю recorder/replay..."
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member $(pkg-config --cflags opencv4) $MAVLINK_INC  "$ROOT/tools/optical_flow_feature_replay_record.cpp" -o "$REC" $(pkg-config --libs opencv4) -lpthread
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member $(pkg-config --cflags opencv4) $MAVLINK_INC  "$ROOT/tools/optical_flow_feature_replay_sweep.cpp" -o "$REP" $(pkg-config --libs opencv4) -lpthread

cat <<EOF
======================================================================
JT-ZERO — REAL-MOTION FEATURE CAP REPLAY
======================================================================
FC и TF-Luna не нужны.
Будет записано 20 секунд РЕАЛЬНОГО видео OV9281.

ПРОТОКОЛ:
  первые 3 с — аппарат спокойно;
  затем 10-12 с — естественные переносы/наклоны, включая несколько быстрых;
  последние 3-5 с — спокойно.

Маршрут повторять не нужно: ВСЕ cap будут проверены на одной записи.
Dataset: $OUT
======================================================================
EOF

mkdir -p "$OUT"
"$REC" "$CAMERA" "$OUT" 20

echo
for DT in 60 80 100; do
  echo "======================================================================"
  echo "REPLAY target dt = $DT ms"
  echo "======================================================================"
  "$REP" "$OUT" "$YAML" "$SCALE" "$DT"
  echo
done
