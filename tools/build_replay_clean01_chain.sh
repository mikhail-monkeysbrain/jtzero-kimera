#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
OUT="${1:-/tmp/replay_clean01_chain}"
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $(pkg-config --cflags opencv4) \
  -I"$KIMERA_ROOT/include" -I/usr/local/include -I/usr/include/eigen3 \
  "$ROOT/tools/replay_clean01_chain.cpp" -o "$OUT" \
  -L"$KIMERA_ROOT/build" -L/usr/local/lib \
  -Wl,-rpath,"$KIMERA_ROOT/build:/usr/local/lib" \
  -lkimera_vio -lgtsam -lgtsam_unstable -lKimeraRPGO -lgflags -lglog -lboost_system \
  $(pkg-config --libs opencv4) -ldl -lpthread
echo "ГОТОВО: $OUT"
