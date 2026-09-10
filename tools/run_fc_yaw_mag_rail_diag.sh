#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${JTZERO_PYTHON:-}"
if [[ -z "$PY" ]]; then
  if [[ -x /home/vio/venv-jtzero-mav/bin/python ]]; then
    PY=/home/vio/venv-jtzero-mav/bin/python
  else
    PY=python3
  fi
fi
exec "$PY" "$ROOT/tools/diagnose_fc_yaw_mag_rail.py" "$@"
