#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
DURATION="${JTZERO_FLOW_AIDING_DIAG_SEC:-60}"
OUT="${JTZERO_FLOW_AIDING_DIAG_BIN:-/tmp/jtzero_optical_flow_relative_aiding_diag}"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: ardupilotmega/mavlink.h не найден"; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC port не найден: $FC"; exit 2; }

echo "Собираю OpticalFlow relative-aiding diagnostic..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_relative_aiding_diag.cpp" \
  -o "$OUT"

echo "Сборка OK: $OUT"
echo
cat <<EOF
======================================================================
JT-ZERO — OPTICAL FLOW AID_RELATIVE DIAG
======================================================================
Камера и TF-Luna НЕ используются.

Тест:
  - runtime принудительно выбирает EKF source set 1 (PRIMARY);
  - 60 секунд отправляет synthetic OPTICAL_FLOW 50 Гц;
  - ждёт переход EKF из constPos/AID_NONE в relative aiding;
  - печатает все EK3_SRC1/2/3 source-параметры;
  - слушает EKF STATUSTEXT и LOCAL_POSITION_NED.

Параметры FC НЕ записываются.
Аппарат DISARMED и неподвижен. Ctrl-C не требуется.
FC: $FC
duration: $DURATION s
======================================================================
EOF

exec "$OUT" "$FC" "$DURATION"
