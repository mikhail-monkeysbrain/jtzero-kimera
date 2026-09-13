#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# REAL FLIGHT configuration.
# Explicitly remove all bench-only overrides so the FC receives the real TF-Luna range.
unset JTZERO_FLOW_BENCH_HEIGHT
unset JTZERO_FLOW_RETURN_MANUAL_TARGET

export JTZERO_FLOW_RETURN_GUI=0
export JTZERO_FLOW_ROTATION_GUI=1
export JTZERO_FLOW_MAX_FEATURES=500

exec bash "$ROOT/tools/run_optical_flow_flight_current_mount.sh"
