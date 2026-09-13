#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MODE="${1:-}"
case "$MODE" in
  translation|TRANSLATION)
    TITLE="TRANSLATION"
    INSTR=$'Двигай аппарат преимущественно поступательно.\nСтарайся держать yaw/roll/pitch небольшими.\nНе нужно повторять какую-либо траекторию.'
    ;;
  yaw|YAW)
    TITLE="YAW"
    INSTR=$'Основная нагрузка — повороты по yaw.\nXY-положение удерживать идеально НЕ нужно.\nRoll/pitch старайся не делать специально.'
    ;;
  tilt|TILT)
    TITLE="TILT"
    INSTR=$'Основная нагрузка — roll/pitch.\nYaw старайся менять мало.\nXY-положение удерживать идеально НЕ нужно.'
    ;;
  *)
    echo "Использование:"
    echo "  bash tools/run_optical_flow_frontend_mode_gui.sh translation"
    echo "  bash tools/run_optical_flow_frontend_mode_gui.sh yaw"
    echo "  bash tools/run_optical_flow_frontend_mode_gui.sh tilt"
    exit 2
    ;;
esac

export JTZERO_FLOW_RETURN_GUI=1
export JTZERO_FLOW_RETURN_MANUAL_TARGET=1
export JTZERO_FLOW_BENCH_HEIGHT=0.60

cat <<EOF
======================================================================
JT-ZERO — FRONTEND STRESS: $TITLE
======================================================================
Моторы НЕ нужны. Арминг не требуется.
Маршрут воспроизводить НЕ НУЖНО.

$INSTR

ПРОТОКОЛ:
  1) Дождись «СИСТЕМА ГОТОВА».
  2) Нажми ПРОБЕЛ.
  3) 15-20 секунд выполняй только указанный тип нагрузки.
  4) Сделай несколько более быстрых движений того же типа.
  5) Остановись на 3-5 секунд.
  6) Нажми Q/ESC.

B и H НЕ НУЖНЫ.
Смысл теста — получить статистику по фактическим dt/gyro, а не повторить путь.
======================================================================
EOF

exec bash "$ROOT/tools/run_optical_flow_flight_current_mount.sh"
