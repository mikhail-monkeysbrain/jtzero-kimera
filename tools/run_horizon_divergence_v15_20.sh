#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/analyze_horizon_divergence_v15_20.cpp"
BIN=/tmp/analyze_horizon_divergence_v15_20
DATA="${1:-/home/vio/jtzero_horizon_threshold_v15_19}"
COMBINED="${2:-/home/vio/jtzero_yaw_only_v13.csv}"

echo '============================================================'
echo 'JT-ZERO HORIZON DIVERGENCE RUNNER v15.20'
echo '============================================================'
echo "Data: $DATA"
echo "IMU:  $COMBINED"
for f in H28.csv H29.csv H30.csv; do
  [[ -s "$DATA/$f" ]] || { echo "[FATAL] missing $DATA/$f"; exit 2; }
done

g++ -std=c++17 -O2 "$SRC" -o "$BIN"
"$BIN" "$DATA" "$COMBINED" | tee "$DATA/divergence_v15_20.txt"

echo "Saved: $DATA/divergence_v15_20.txt"
echo "Saved: $DATA/divergence_v15_20.csv"
