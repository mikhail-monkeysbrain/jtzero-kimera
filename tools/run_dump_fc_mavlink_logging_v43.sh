#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/dump_fc_mavlink_logging_v43"
SRC="$ROOT/tools/dump_fc_mavlink_logging_v43.cpp"

echo "============================================================"
echo "JT-ZERO: ЧТЕНИЕ MAVLINK LOGGING BACKEND v43"
echo "ТОЛЬКО ЧТЕНИЕ: параметры FC НЕ изменяются."
echo "Физический тест НЕ нужен. Стенд не двигать."
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"
