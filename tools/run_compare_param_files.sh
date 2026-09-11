#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"

if [[ $# -ne 2 ]]; then
  echo "Использование: bash tools/run_compare_param_files.sh OLD.param NEW.param"
  exit 2
fi

exec "$PY" tools/compare_param_files.py "$1" "$2"
