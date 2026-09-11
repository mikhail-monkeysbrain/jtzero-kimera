#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
DURATION="${JTZERO_REMOTE_LOG_DURATION:-45}"
OUT_BIN="${JTZERO_REMOTE_LOG_BIN:-/tmp/jtzero_remote_log_xkv_diag}"
RUN_ROOT="${JTZERO_RUN_ROOT:-/home/vio/jtzero_runs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RUN_ROOT/${STAMP}_REMOTE_LOG_XKV_DIAG"
BIN_FILE="$RUN_DIR/remote_ekf.bin"

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
mkdir -p "$RUN_DIR"

echo "Собираю MAVLink remote-log XKV diagnostic..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/remote_log_xkv_diag.cpp" \
  -o "$OUT_BIN"

echo "Сборка OK: $OUT_BIN"
echo
cat <<EOF
======================================================================
JT-ZERO — REMOTE EKF LOG OVER MAVLINK
======================================================================
SD-карта FC НЕ используется. DataFlash stream сохраняется на RPi.

Перед первым запуском должны быть:
  LOG_BACKEND_TYPE = 2   # MAVLink logger backend
  LOG_DISARMED     = 1   # логировать DISARMED bench
  EK3_LOG_LEVEL    = 0   # ALL, включая XKV1/XKV2/XKT
  EK3_SRC1_YAW     = 0   # только для текущего gyro-bias gate теста

ВАЖНО: после изменения LOG_BACKEND_TYPE нужен reboot FC.
Программа сама проверит параметры и ничего из них не записывает.

Тест ${DURATION} секунд:
  - выбирает SRC1 runtime;
  - держит synthetic OPTICAL_FLOW 50 Гц;
  - запускает AP_Logger_MAVLink;
  - сохраняет удалённый DataFlash BIN на RPi.

Аппарат DISARMED и полностью неподвижен.
FC: $FC
BIN: $BIN_FILE
======================================================================
EOF

exec "$OUT_BIN" "$FC" "$BIN_FILE" "$DURATION"
