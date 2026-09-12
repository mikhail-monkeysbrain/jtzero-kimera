#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
COUNT="${1:-6}"
if ! [[ "$COUNT" =~ ^[2-9][0-9]*$ ]] || (( COUNT % 2 != 0 )); then
  echo "ОШИБКА: укажите чётное число проходов 2..30"
  exit 2
fi
export JTZERO_GUI_RUNS="$COUNT"
export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.1060}"
export JTZERO_FLOW_TARGET_MM="${JTZERO_FLOW_TARGET_MM:-300}"
exec python3 tools/optical_flow_continuous_reciprocal_gui.py
