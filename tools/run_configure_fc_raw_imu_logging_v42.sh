#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/configure_fc_raw_imu_logging_v42"
SRC="$ROOT/tools/configure_fc_raw_imu_logging_v42.cpp"

if [[ "${1:-}" != "--enable" && "${1:-}" != "--restore" ]]; then
  echo "Использование:"
  echo "  bash tools/run_configure_fc_raw_imu_logging_v42.sh --enable"
  echo "  bash tools/run_configure_fc_raw_imu_logging_v42.sh --restore"
  exit 2
fi

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN" "$1"
