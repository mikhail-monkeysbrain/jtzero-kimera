#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/tools/r1_replay.cpp"
OUT=/tmp/jtzero_r1_replay

echo "===== R1 4/6 — STANDALONE REPLAY BUILD ====="
rm -f "$OUT"
g++ -std=c++17 -O2 -DNDEBUG "$SRC" -o "$OUT"
sha256sum "$OUT"
echo "BUILD_R1_REPLAY PASS"
