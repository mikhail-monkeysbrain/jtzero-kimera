#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
BASE="$ROOT/tools/run_final_diagnostic_suite_v15_29.sh"
TMP="$ROOT/tools/.run_final_diagnostic_suite_v15_30.generated.sh"
PROVEN=/tmp/record_mono_imu_final_suite_gui_v15_29_proven
cleanup(){ rm -f "$TMP"; }
trap cleanup EXIT

[[ -f "$BASE" ]] || { echo "ERROR: missing $BASE" >&2; exit 1; }

# v15.30 guard is a separate process. Build the proven v15.29 acquisition
# executable independently so there are no nested-main/preprocessor conflicts.
echo '[V15.30] Build isolated proven v15.29 acquisition recorder'
g++ -std=c++17 -O2 "$ROOT/tools/record_mono_imu_final_suite_gui_v15_29.cpp" -o "$PROVEN" \
  -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
[[ -x "$PROVEN" ]] || { echo "ERROR: failed to build $PROVEN" >&2; exit 1; }

# Reuse the already validated v15.29 causal/production matrix. Only the
# top-level physical recorder is replaced by the v15.30 persistence/quality guard.
sed \
  -e 's#record_mono_imu_final_suite_gui_v15_29.cpp#record_mono_imu_final_suite_gui_v15_30.cpp#g' \
  -e 's#/tmp/record_mono_imu_final_suite_gui_v15_29#/tmp/record_mono_imu_final_suite_gui_v15_30#g' \
  -e 's#JT-ZERO FINAL DIAGNOSTIC SUITE v15.29#JT-ZERO FINAL DIAGNOSTIC SUITE v15.30#g' \
  "$BASE" > "$TMP"
chmod +x "$TMP"
exec "$TMP"
