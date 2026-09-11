#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ $# -ne 1 ]]; then
  echo "Использование: bash tools/run_analyze_remote_xkf5_optflow.sh /path/to/remote_ekf.bin"
  exit 2
fi
exec python3 tools/analyze_remote_xkf5_optflow.py "$1"
