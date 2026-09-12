#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SESSION="${1:-}"
if [[ -z "$SESSION" ]]; then
  SESSION="$(find /home/vio/jtzero_runs -maxdepth 1 -type f -name '*_OPTICAL_FLOW_GUI_SERIES.json' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[[ -n "$SESSION" && -f "$SESSION" ]] || { echo "ОШИБКА: GUI session JSON не найден"; exit 2; }

echo "SESSION=$SESSION"
exec python3 tools/analyze_optical_flow_gui_series_forensic.py "$SESSION"
