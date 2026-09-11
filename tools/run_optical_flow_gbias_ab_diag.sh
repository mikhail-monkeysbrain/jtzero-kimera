#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
TEST_VALUE="${JTZERO_GBIAS_TEST_VALUE:-0.0001}"
TEST_DURATION="${JTZERO_GBIAS_TEST_DURATION:-90}"
OUT="${JTZERO_GBIAS_AB_BIN:-/tmp/jtzero_optical_flow_gbias_ab_diag}"

MAVLINK_INC=""
for d in \
  "$KIMERA_ROOT/third_party/mavlink" \
  "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" \
  "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done

[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: ardupilotmega/mavlink.h не найден"; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC port не найден: $FC"; exit 2; }

echo "Собираю обратимый OpticalFlow GBIAS A/B diagnostic..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_gbias_ab_diag.cpp" \
  -o "$OUT"

echo "Сборка OK: $OUT"
echo
cat <<EOF
======================================================================
JT-ZERO — OPTICAL FLOW GBIAS A/B DIAG
======================================================================
Камера и TF-Luna НЕ используются.

Контрольный эксперимент:
  1. 10 секунд baseline при текущем EK3_GBIAS_P_NSE;
  2. временно EK3_GBIAS_P_NSE = $TEST_VALUE;
  3. до $TEST_DURATION секунд synthetic OPTICAL_FLOW 50 Гц;
  4. проверка перехода AID_NONE -> AID_RELATIVE;
  5. автоматическое восстановление исходного EK3_GBIAS_P_NSE.

Это ТОЛЬКО bench-диагностика. Тестовое значение не является flight-настройкой.
Ожидается EK3_SRC1_YAW=0, аппарат DISARMED и полностью неподвижен.

Не закрывай терминал и не отключай питание FC во время временного изменения.
При обычном завершении и Ctrl-C программа пытается вернуть исходный параметр.
FC: $FC
======================================================================
EOF

exec "$OUT" "$FC" "$TEST_VALUE" "$TEST_DURATION"
