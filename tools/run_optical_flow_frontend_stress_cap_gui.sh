#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CAP="${1:-}"
case "$CAP" in
  500|300|200) ;;
  *)
    echo "Использование:"
    echo "  bash tools/run_optical_flow_frontend_stress_cap_gui.sh 500"
    echo "  bash tools/run_optical_flow_frontend_stress_cap_gui.sh 300"
    echo "  bash tools/run_optical_flow_frontend_stress_cap_gui.sh 200"
    exit 2
    ;;
esac

export JTZERO_FLOW_RETURN_GUI=1
export JTZERO_FLOW_RETURN_MANUAL_TARGET=1
export JTZERO_FLOW_BENCH_HEIGHT=0.60
export JTZERO_FLOW_MAX_FEATURES="$CAP"

cat <<EOF
======================================================================
JT-ZERO — LIVE FRONTEND STRESS, FEATURE CAP = $CAP
======================================================================
Это контролируемый live-тест одной переменной:
  max_features = $CAP

Остальные параметры не меняются.
Маршрут воспроизводить НЕ НУЖНО.

ПРОТОКОЛ:
  1) Дождись «СИСТЕМА ГОТОВА».
  2) Нажми ПРОБЕЛ.
  3) 5-10 секунд двигай аппарат спокойно.
  4) 10-20 секунд естественные переносы/наклоны.
  5) Несколько более быстрых движений.
  6) 3-5 секунд неподвижно.
  7) Q/ESC.

B/H не нужны.
После прогона сохрани путь к CSV.
======================================================================
EOF

exec bash "$ROOT/tools/run_optical_flow_flight_current_mount.sh"
