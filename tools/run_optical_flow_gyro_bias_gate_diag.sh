#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
DURATION="${JTZERO_GYRO_GATE_DURATION:-30}"
OUT="${JTZERO_GYRO_GATE_BIN:-/tmp/jtzero_optical_flow_gyro_bias_gate_diag}"

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

echo "Собираю OpticalFlow gyro-bias gate diagnostic..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_gyro_bias_gate_diag.cpp" \
  -o "$OUT"

echo "Сборка OK: $OUT"
echo
cat <<EOF
======================================================================
JT-ZERO — OPTICAL FLOW GYRO-BIAS GATE DIAG
======================================================================
Камера и TF-Luna НЕ используются.

Тест ${DURATION} секунд:
  - runtime выбирает SRC1;
  - держит synthetic OPTICAL_FLOW свежим на 50 Гц;
  - контролирует EKF AID_RELATIVE;
  - читает EK3_GBIAS_P_NSE / EK3_GYRO_P_NSE / EK3_IMU_MASK;
  - собирает stationary gyro mean/std для SCALED_IMU1/2/3.

Параметры НЕ записываются.
Для чистого теста сейчас ожидается EK3_SRC1_YAW=0.
Аппарат DISARMED и полностью неподвижен.
FC: $FC
======================================================================
EOF

exec "$OUT" "$FC" "$DURATION"
