#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
LUNA="${JTZERO_FLOW_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
OUT="/tmp/jtzero_real_luna_fc_rangefinder_diag"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены"; exit 2; }

echo "Сборка..."
g++ -std=c++17 -O2 -DNDEBUG -pthread $MAVLINK_INC "$ROOT/tools/real_luna_fc_rangefinder_diag.cpp" -o "$OUT"
echo "Сборка OK: $OUT"
cat <<EOF

======================================================================
JT-ZERO — REAL TF-LUNA -> FC RANGEFINDER DIAG
======================================================================
Камера не используется. Параметры FC не меняются.
10 секунд читает реальный TF-Luna на $LUNA, публикует его как DISTANCE_SENSOR
и сравнивает с DISTANCE_SENSOR, который возвращает сам FC.

Аппарат DISARMED и неподвижен.
Ничего двигать не нужно. Тест завершится сам.
======================================================================
EOF
exec "$OUT" "$LUNA" "$FC"
