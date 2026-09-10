#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! python3 - <<'PY' >/dev/null 2>&1
import tkinter
PY
then
  echo "ОШИБКА: Python tkinter не установлен."
  echo "Установите: sudo apt install python3-tk"
  exit 2
fi

exec python3 "$ROOT/tools/ground_motion_test_gui.py"
