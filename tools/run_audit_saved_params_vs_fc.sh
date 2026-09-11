#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-/home/vio/venv-jtzero-mav/bin/python}"
[[ -x "$PY" ]] || PY=python3
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FC_BAUD:-460800}"

if [[ $# -lt 1 ]]; then
  echo "Использование: bash tools/run_audit_saved_params_vs_fc.sh /path/to/saved.param"
  exit 2
fi

exec "$PY" tools/audit_saved_params_vs_fc.py "$1" --device "$FC" --baud "$BAUD"
