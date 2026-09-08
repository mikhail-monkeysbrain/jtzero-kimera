#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/read_fc_autopilot_version_v44"
SRC="$ROOT/tools/read_fc_autopilot_version_v44.cpp"

echo "============================================================"
echo "JT-ZERO: ЧТЕНИЕ ВЕРСИИ И ИДЕНТИФИКАТОРОВ FC v44"
echo "ТОЛЬКО ЧТЕНИЕ: FC НЕ изменяется."
echo "Физический тест НЕ нужен. Стенд не двигать."
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"
