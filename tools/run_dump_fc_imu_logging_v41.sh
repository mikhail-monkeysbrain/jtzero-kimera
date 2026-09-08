#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/dump_fc_imu_logging_v41"
SRC="$ROOT/tools/dump_fc_imu_logging_v41.cpp"

echo "============================================================"
echo "JT-ZERO: ЧТЕНИЕ ПАРАМЕТРОВ ЛОГИРОВАНИЯ IMU v41"
echo "ТОЛЬКО ЧТЕНИЕ: параметры полётного контроллера НЕ изменяются."
echo "Физический тест НЕ нужен. Стенд не двигать."
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"

LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"
