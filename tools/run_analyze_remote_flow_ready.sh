#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT"

if [[ $# -gt 0 ]]; then
  exec python3 tools/analyze_remote_flow_ready.py "$1"
else
  exec python3 tools/analyze_remote_flow_ready.py
fi
