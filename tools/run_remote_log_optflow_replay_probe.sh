#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-/home/vio/venv-jtzero-mav/bin/python}"
if [[ ! -x "$PY" ]]; then PY=python3; fi
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FC_BAUD:-460800}"
DURATION="${JTZERO_REPLAY_PROBE_DURATION:-12}"

PARAM_TOOL="$ROOT/tools/set_fc_param_checked.py"
ANALYZER="$ROOT/tools/analyze_remote_ekf3_optflow_ingress.py"

OLD_REPLAY="$($PY "$PARAM_TOOL" LOG_REPLAY --device "$FC" --baud "$BAUD" --value-only)"
RESTORED=0
restore_replay() {
  if [[ "$RESTORED" -eq 0 ]]; then
    echo
    echo "Восстанавливаю LOG_REPLAY=$OLD_REPLAY ..."
    if "$PY" "$PARAM_TOOL" LOG_REPLAY "$OLD_REPLAY" --device "$FC" --baud "$BAUD"; then
      RESTORED=1
    else
      echo "ВНИМАНИЕ: автоматическое восстановление LOG_REPLAY не подтверждено." >&2
    fi
  fi
}
trap restore_replay EXIT INT TERM

echo "======================================================================"
echo "JT-ZERO — SHORT EKF3 OPTFLOW REPLAY PROBE"
echo "======================================================================"
echo "Цель: включить LOG_REPLAY только на время короткого теста и проверить ROFH."
echo "Нового физического движения не требуется. Камера и TF-Luna не используются."
echo "Synthetic OPTICAL_FLOW отправляется существующим remote-log diagnostic."
echo
echo "Исходный LOG_REPLAY=$OLD_REPLAY"
echo "Временно ставлю LOG_REPLAY=1 с read-back..."
"$PY" "$PARAM_TOOL" LOG_REPLAY 1 --device "$FC" --baud "$BAUD"
echo
echo "Запускаю remote log на ${DURATION} секунд..."

JTZERO_REMOTE_LOG_DURATION="$DURATION" JTZERO_FLOW_FC="$FC" \
  bash tools/run_remote_log_xkv_diag.sh

NEWBIN="$(find /home/vio/jtzero_runs -maxdepth 2 -type f -path '*_REMOTE_LOG_XKV_DIAG/remote_ekf.bin' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -z "$NEWBIN" || ! -f "$NEWBIN" ]]; then
  echo "ОШИБКА: новый remote_ekf.bin не найден" >&2
  exit 5
fi

echo
echo "Новый BIN: $NEWBIN"
restore_replay
trap - EXIT INT TERM

echo
echo "===== POST ANALYSIS ====="
"$PY" "$ANALYZER" "$NEWBIN" --device "$FC" --baud "$BAUD"
