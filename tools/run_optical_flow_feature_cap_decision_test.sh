#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# GUI is the only user-facing interface for this test.
exec python3 "$ROOT/tools/optical_flow_feature_cap_decision_gui.py"
