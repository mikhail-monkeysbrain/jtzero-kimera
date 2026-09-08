#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/live_static_imu_sources_v38"
SRC="$ROOT/tools/live_static_imu_sources_v38.cpp"

echo "============================================================"
echo "JT-ZERO: СТАТИЧЕСКОЕ СРАВНЕНИЕ IMU v38"
echo "СТЕНД НЕ ДВИГАТЬ. YAW НЕ МЕНЯТЬ."
echo "Длительность записи: 30 секунд."
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"

rm -f /home/vio/jtzero_static_imu_sources_v38.csv

LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"

echo
echo "Результат:"
ls -lh /home/vio/jtzero_static_imu_sources_v38.csv
echo
echo "Загрузите файл:"
echo "  /home/vio/jtzero_static_imu_sources_v38.csv"
