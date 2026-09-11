#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
OUT="/tmp/jtzero_fc_rangefinder_ingress_diag"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены"; exit 2; }

echo "Сборка..."
g++ -std=c++17 -O2 -DNDEBUG -pthread $MAVLINK_INC "$ROOT/tools/fc_rangefinder_ingress_diag.cpp" -o "$OUT"
echo "Сборка OK: $OUT"
cat <<EOF

======================================================================
JT-ZERO — FC RANGEFINDER MAVLink INGRESS DIAG
======================================================================
Камера и TF-Luna НЕ используются.
Параметры FC НЕ меняются.

8 секунд:
  0..4 c  -> отправляется synthetic DISTANCE_SENSOR = 0.60 м
  4..8 c  -> отправляется synthetic DISTANCE_SENSOR = 0.70 м

Одновременно запрашивается DISTANCE_SENSOR, который публикует САМ FC.
Если FC telemetry повторит ступень 0.60 -> 0.70 м, вход
RPi -> MAVLink -> AP_RangeFinder_MAVLink доказан.

Аппарат может быть DISARMED. Ничего двигать не нужно.
Тест завершится сам.
======================================================================
EOF
exec "$OUT" "$FC"
