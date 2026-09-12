#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
CSV="${1:-}"
if [[ -z "$CSV" ]]; then
  CSV="$(find /home/vio/jtzero_runs -maxdepth 2 -type f -name optical_flow_mavlink.csv -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[[ -n "$CSV" && -f "$CSV" ]] || { echo "ОШИБКА: CSV не найден"; exit 2; }
exec python3 tools/analyze_optical_flow_pipeline_latency.py "$CSV"
