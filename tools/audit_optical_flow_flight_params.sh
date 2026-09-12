#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PARAM_TOOL="$ROOT/tools/set_fc_param_checked.py"

pick_python() {
  local p
  if [[ -n "${PYTHON:-}" ]]; then
    if "$PYTHON" -c 'import pymavlink' >/dev/null 2>&1; then
      echo "$PYTHON"
      return 0
    fi
  fi

  for p in \
    "$HOME/venv-jtzero-mav/bin/python" \
    "$HOME/.venv-jtzero-mav/bin/python" \
    "$HOME/venv/bin/python" \
    "$HOME/.venv/bin/python" \
    python3
  do
    command -v "$p" >/dev/null 2>&1 || [[ -x "$p" ]] || continue
    if "$p" -c 'import pymavlink' >/dev/null 2>&1; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

if ! PY="$(pick_python)"; then
  echo "ОШИБКА: не найден Python с модулем pymavlink." >&2
  echo "Укажите интерпретатор через PYTHON=/path/to/python или активируйте существующий MAVLink venv." >&2
  exit 2
fi

if [[ ! -f "$PARAM_TOOL" ]]; then
  echo "ОШИБКА: не найден $PARAM_TOOL" >&2
  exit 2
fi

DEVICE="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FLOW_FC_BAUD:-460800}"

declare -A EXPECTED=(
  [FLOW_TYPE]=5
  [FLOW_OPTIONS]=0
  [FLOW_ORIENT_YAW]=0
  [FLOW_FXSCALER]=0
  [FLOW_FYSCALER]=0
  [EK3_FLOW_DELAY]=0
  [EK3_SRC1_POSXY]=0
  [EK3_SRC1_VELXY]=5
  [EK3_SRC1_POSZ]=2
  [EK3_SRC1_VELZ]=0
  [EK3_SRC1_YAW]=0
)

order=(
  FLOW_TYPE
  FLOW_OPTIONS
  FLOW_ORIENT_YAW
  FLOW_FXSCALER
  FLOW_FYSCALER
  EK3_FLOW_DELAY
  EK3_SRC1_POSXY
  EK3_SRC1_VELXY
  EK3_SRC1_POSZ
  EK3_SRC1_VELZ
  EK3_SRC1_YAW
)

echo "======================================================================"
echo "JT-ZERO — OPTICAL FLOW FLIGHT PREFLIGHT"
echo "======================================================================"
echo "FC: $DEVICE @ $BAUD"
echo "Python: $PY"
echo "Параметры только ЧИТАЮТСЯ. Ничего не изменяется."
echo

fail=0
for name in "${order[@]}"; do
  expected="${EXPECTED[$name]}"
  if ! value="$("$PY" "$PARAM_TOOL" "$name" --device "$DEVICE" --baud "$BAUD" --value-only 2>/tmp/jtzero_preflight_param.err)"; then
    echo "FAIL  $name : не прочитан"
    cat /tmp/jtzero_preflight_param.err >&2 || true
    fail=1
    continue
  fi

  if "$PY" - "$value" "$expected" <<'PY'
import math, sys
v=float(sys.argv[1]); e=float(sys.argv[2])
raise SystemExit(0 if math.isclose(v,e,rel_tol=0.0,abs_tol=max(1e-6,abs(e)*1e-5)) else 1)
PY
  then
    printf "PASS  %-18s = %s\n" "$name" "$value"
  else
    printf "FAIL  %-18s = %s  expected=%s\n" "$name" "$value" "$expected"
    fail=1
  fi
done

echo
if (( fail )); then
  echo "RESULT: FAIL"
  echo "Flight launcher НЕ запускать до устранения несоответствий."
  exit 1
fi

echo "RESULT: PASS"
echo "FC source configuration соответствует проверенному OpticalFlow flight-контуру."
