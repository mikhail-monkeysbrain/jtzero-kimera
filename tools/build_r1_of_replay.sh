#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/tools/r1_of_replay.cpp"
OUT=/tmp/jtzero_r1_of_replay

echo "===== R1 6/6 — STANDALONE OF BUILD ====="
rm -f "$OUT"
g++ -std=c++17 -O2 -DNDEBUG "$SRC" -o "$OUT" \
  $(pkg-config --cflags --libs opencv4)
sha256sum "$OUT"
echo "BUILD_R1_OF_REPLAY PASS"
