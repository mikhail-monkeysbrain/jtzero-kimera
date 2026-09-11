#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${JTZERO_MAV_PYTHON:-/home/vio/venv-jtzero-mav/bin/python3}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

if ! "$PYTHON_BIN" -c 'import pymavlink, pymavlink.mavftp' >/dev/null 2>&1; then
  echo "ОШИБКА: pymavlink.mavftp не найден в $PYTHON_BIN" >&2
  echo "Укажи Python из MAVLink venv через JTZERO_MAV_PYTHON=/path/to/python3" >&2
  exit 2
fi

exec "$PYTHON_BIN" -u tools/probe_fc_flash_optflow_feature.py \
  --device "${JTZERO_FC_DEVICE:-/dev/ttyAMA0}" \
  --baud "${JTZERO_FC_BAUD:-460800}" \
  --max-bytes "${JTZERO_FLASH_SCAN_BYTES:-2097152}"
