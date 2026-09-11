#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ $# -lt 1 ]]; then
  echo "Использование: bash tools/run_analyze_optical_flow_guided_csv.sh /path/to/optical_flow_mavlink.csv"
  exit 2
fi
exec python3 tools/analyze_optical_flow_guided_csv.py "$1"
