#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-/home/vio/venv-jtzero-mav/bin/python}"
if [ ! -x "$PY" ]; then
  PY=python3
fi

exec "$PY" tools/analyze_remote_ekf3_optflow_ingress.py "$@"
