#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

COUNT="${1:-6}"
if ! [[ "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
  echo "ОШИБКА: число проходов должно быть положительным целым"
  exit 2
fi
if (( COUNT % 2 != 0 )); then
  echo "ОШИБКА: reciprocal-тест требует чётное число проходов (A→B/B→A пары)"
  exit 2
fi

export JTZERO_GUI_RUNS="$COUNT"
export JTZERO_GUI_RECIPROCAL=1
export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.0000}"
export JTZERO_FLOW_TARGET_MM="${JTZERO_FLOW_TARGET_MM:-300}"

exec python3 tools/optical_flow_bench_gui.py
