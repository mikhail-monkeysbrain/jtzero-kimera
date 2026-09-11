#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${1:-}"

if [[ -z "$BIN" ]]; then
  BIN="$(find /home/vio/jtzero_runs -maxdepth 2 -type f -path '*_REMOTE_LOG_XKV_DIAG/remote_ekf.bin' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n1 | cut -d' ' -f2-)"
fi

if [[ -z "$BIN" || ! -f "$BIN" ]]; then
  echo "ОШИБКА: remote_ekf.bin не найден" >&2
  exit 2
fi

echo "Анализирую: $BIN"
python3 "$ROOT/tools/analyze_remote_xkv_gate.py" "$BIN"
