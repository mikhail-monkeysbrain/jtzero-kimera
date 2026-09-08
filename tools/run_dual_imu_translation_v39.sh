#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/live_dual_imu_translation_v39"
SRC="$ROOT/tools/live_dual_imu_translation_v39.cpp"

echo "============================================================"
echo "JT-ZERO: A-B-A СРАВНЕНИЕ ДВУХ IMU v39"
echo "YAW ДРОНА/СТЕНДА: примерно -90°"
echo "Новые метки и ArUco НЕ НУЖНЫ."
echo "Kimera в измерении не используется."
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"

rm -f /home/vio/jtzero_dual_imu_translation_v39.csv

LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"

echo
echo "Результат:"
ls -lh /home/vio/jtzero_dual_imu_translation_v39.csv
echo
echo "Загрузите файл:"
echo "  /home/vio/jtzero_dual_imu_translation_v39.csv"
