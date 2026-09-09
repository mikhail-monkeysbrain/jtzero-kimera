#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/r1_recorder.cpp"
OUT="${1:-/tmp/jtzero_r1_recorder}"

echo "===== R1 3/6 — STANDALONE BUILD ====="
echo "source=$SRC"
echo "output=$OUT"

rm -f "$OUT"
g++ -std=c++17 -O2 -DNDEBUG -pthread "$SRC" -o "$OUT"

test -x "$OUT"
echo
echo "===== BINARY IDENTITY ====="
sha256sum "$OUT"
stat -c 'bytes=%s mtime=%y' "$OUT"
echo
echo "BUILD_R1_RECORDER PASS"
