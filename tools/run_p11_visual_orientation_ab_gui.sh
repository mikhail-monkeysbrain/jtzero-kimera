#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN=/tmp/p11_visual_orientation_ab_gui
echo "[СБОРКА] $BIN"
g++ -std=c++17 -O2 "$ROOT/tools/p11_visual_orientation_ab_gui.cpp" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4)
echo "[ЗАПУСК] P11 — независимая визуальная проверка A/B"
"$BIN" "${1:-0}"
