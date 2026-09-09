#!/usr/bin/env bash
set -euo pipefail
ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.17c — KIMERA SOURCE LAYOUT SNAPSHOT"
echo "======================================================================"
echo "source: $SRC"
echo

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: source not found"
  exit 1
fi

echo "===== TOP 60 LINES ====="
sed -n '1,60p' "$SRC"

echo
echo "===== INCLUDE MATCHES ====="
grep -nE '^#include <(gflags|glog|string|optional|cmath|cstdlib)' "$SRC" || true

echo
echo "===== FRONTEND CALLBACK ANCHORS ====="
grep -nE 'registerOutputCallback|backend_input_queue|MonoFrontendOutput|BackendInput|status_mono_measurements' "$SRC" || true

echo
echo "===== CALLBACK CONTEXT ====="
LINE=$(grep -n 'registerOutputCallback' "$SRC" | head -1 | cut -d: -f1 || true)
if [[ -n "$LINE" ]]; then
  START=$(( LINE > 25 ? LINE-25 : 1 ))
  END=$(( LINE+80 ))
  sed -n "${START},${END}p" "$SRC"
else
  echo "registerOutputCallback not found"
fi
