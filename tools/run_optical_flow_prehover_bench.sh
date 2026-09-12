#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# JT-Zero pre-hover bench.
# Один короткий тест целевого поведения перед переходом к моторному удержанию.
# Production код не меняется.

export JTZERO_FLOW_FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.1060}"
export JTZERO_FLOW_TARGET_MM="${JTZERO_FLOW_TARGET_MM:-75}"
export JTZERO_FLOW_GUIDED_MODE="armed-gate-open"
export JTZERO_FLOW_PRE_STATIC_SEC="${JTZERO_FLOW_PRE_STATIC_SEC:-8}"
export JTZERO_FLOW_POST_STATIC_SEC="${JTZERO_FLOW_POST_STATIC_SEC:-12}"

cat <<EOF
======================================================================
JT-ZERO — PRE-HOVER BENCH
======================================================================
Это НЕ новый тест масштаба.

Цель:
  1) 8 с покоя;
  2) один небольшой горизонтальный сдвиг около 50–100 мм;
  3) полная остановка;
  4) 12 с покоя;
  5) проверить, возвращается ли EKF velocity к ~0 и прекращается ли drift.

Условия:
  - пропеллеры сняты;
  - FC ARMED;
  - стенд не вращать, не наклонять, не приподнимать;
  - focal_scale=$JTZERO_FLOW_FOCAL_SCALE;
  - EK3_FLOW_DELAY НЕ менять;
  - synthetic bench height = 0.60 м через существующий armed-gate-open режим.

Номинальная подсказка движения: $JTZERO_FLOW_TARGET_MM мм.
Точно попадать в это расстояние НЕ нужно.
======================================================================
EOF

exec bash tools/run_optical_flow_mavlink_bench.sh
