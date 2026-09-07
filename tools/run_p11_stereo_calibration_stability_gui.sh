#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN=/tmp/p11_stereo_calibration_stability_gui

echo "[СБОРКА] $BIN"

g++ -std=c++17 -O2 -pthread \
  "$ROOT/tools/p11_stereo_calibration_stability_gui.cpp" \
  -o "$BIN" \
  $(pkg-config --cflags --libs libcamera opencv4)

echo "[ЗАПУСК] P11 — стабильность стереокалибровки на неподвижной ChArUco-мишени"
exec "$BIN" "${1:-auto}"
