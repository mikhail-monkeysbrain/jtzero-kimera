#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

COUNT="${1:-5}"
export JTZERO_GUI_RUNS="$COUNT"
export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.0000}"
# Это только приблизительный nominal для старого guided backend.
# GUI после каждого прохода попросит ФАКТИЧЕСКИ измеренное расстояние.
export JTZERO_FLOW_TARGET_MM="${JTZERO_FLOW_TARGET_MM:-300}"

if ! python3 - <<'PY'
import tkinter
PY
then
  echo "ОШИБКА: tkinter не установлен."
  echo "На Raspberry Pi/Debian обычно нужен пакет: sudo apt install python3-tk"
  exit 2
fi

exec python3 tools/optical_flow_bench_gui.py
