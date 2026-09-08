#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/live_mechanical_translation_fc_accel_v37"
SRC="$ROOT/tools/live_mechanical_translation_fc_accel_v37.cpp"

echo "============================================================"
echo "JT-ZERO MECHANICAL TRANSLATION REFERENCE v37"
echo "DRONE/STAND YAW: approximately -90 deg"
echo "No Kimera estimator is used for the measurement."
echo "Record one continuous side-view phone video showing:"
echo "  1) marker/cross rigidly attached next to FC/IMU"
echo "  2) rigid upper platform of the stand"
echo "  3) fixed background reference"
echo "Sequence: A settle -> A->B 500 mm -> B settle -> B->A 500 mm -> A settle"
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"

rm -f /home/vio/jtzero_mechanical_translation_v37.csv

LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"

echo
echo "Result CSV:"
ls -lh /home/vio/jtzero_mechanical_translation_v37.csv
echo
echo "Upload BOTH:"
echo "  /home/vio/jtzero_mechanical_translation_v37.csv"
echo "  the continuous side-view phone video"
