#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN=/tmp/read_p11_fc_accel_params_cpp
echo "[СБОРКА] $BIN"
g++ -std=c++17 -O2 "$ROOT/tools/read_p11_fc_accel_params.cpp" -o "$BIN" \
  -I/home/vio/Kimera-VIO/third_party/mavlink
echo "[ЗАПУСК] только чтение параметров FC"
"$BIN"
