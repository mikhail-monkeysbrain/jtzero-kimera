#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/tools/run_final_diagnostic_suite_v15_29.sh"
TMP="$ROOT/tools/.run_final_diagnostic_suite_v15_30.generated.sh"
cleanup(){ rm -f "$TMP"; }
trap cleanup EXIT

[[ -f "$BASE" ]] || { echo "ERROR: missing $BASE" >&2; exit 1; }

# Reuse the already validated v15.29 causal/production matrix, changing only
# the physical recorder executable/source to the guarded v15.30 wrapper.
sed \
  -e 's#record_mono_imu_final_suite_gui_v15_29.cpp#record_mono_imu_final_suite_gui_v15_30.cpp#g' \
  -e 's#/tmp/record_mono_imu_final_suite_gui_v15_29#/tmp/record_mono_imu_final_suite_gui_v15_30#g' \
  -e 's#JT-ZERO FINAL DIAGNOSTIC SUITE v15.29#JT-ZERO FINAL DIAGNOSTIC SUITE v15.30#g' \
  "$BASE" > "$TMP"
chmod +x "$TMP"
exec "$TMP"
