#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN=/tmp/p11_ruler_pass_gui

echo "[СБОРКА] $BIN"
g++ -std=c++17 -O2 "$ROOT/tools/p11_ruler_pass_gui.cpp" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4)

echo "[ЗАПУСК] P11 — проход A→B вдоль рулетки-направляющей"
"$BIN" "${1:-auto}"
