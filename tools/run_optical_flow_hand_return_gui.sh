#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Единственный пользовательский интерфейс этого теста — OpenCV GUI.
# Терминал остаётся только для технических ошибок/финального пути к CSV.
export JTZERO_FLOW_RETURN_GUI=1
export JTZERO_FLOW_RETURN_MANUAL_TARGET=1
export JTZERO_FLOW_BENCH_HEIGHT=0.60
export JTZERO_FLOW_MAX_FEATURES=500

exec bash "$ROOT/tools/run_optical_flow_flight_current_mount.sh"
