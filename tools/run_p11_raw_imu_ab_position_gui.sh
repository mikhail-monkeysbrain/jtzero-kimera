#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN=/tmp/p11_raw_imu_ab_position_gui

echo "[BUILD] $BIN"
g++ -std=c++17 -O2   "$ROOT/tools/p11_raw_imu_ab_position_gui.cpp"   -o "$BIN"   -I/home/vio/Kimera-VIO/third_party/mavlink   $(pkg-config --cflags --libs opencv4)

echo "[RUN] $BIN"
"$BIN"
