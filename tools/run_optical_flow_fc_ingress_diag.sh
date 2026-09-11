#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
OUT="${JTZERO_FLOW_INGRESS_BIN:-/tmp/jtzero_optical_flow_fc_ingress_diag}"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: ardupilotmega/mavlink.h не найден"; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC port не найден: $FC"; exit 2; }

echo "Собираю OpticalFlow FC ingress diagnostic..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_fc_ingress_diag.cpp" \
  -o "$OUT"

echo "Сборка OK: $OUT"
echo
cat <<EOF
======================================================================
JT-ZERO — FC OPTICAL FLOW INGRESS DIAG
======================================================================
Камера и TF-Luna здесь НЕ используются.
Тест 7 секунд отправляет почти нулевой synthetic OPTICAL_FLOW и проверяет,
возвращает ли сам ArduPilot OPTICAL_FLOW telemetry от sysid FC.

Это позволяет доказать отдельно:
  RPi UART -> MAVLink -> AP_OpticalFlow_MAV -> healthy frontend

Аппарат DISARMED и неподвижен. Ничего не двигать.
FC: $FC
======================================================================
EOF

exec "$OUT" "$FC"
