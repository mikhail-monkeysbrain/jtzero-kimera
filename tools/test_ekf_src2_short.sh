#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="/tmp/jtzero_src2_short_$(date +%Y%m%d_%H%M%S).log"

cd "$ROOT"

echo "=== JT-ZERO: короткая проверка переключения EKF -> SRC2 ==="
echo "БПЛА НЕ ДВИГАТЬ. Длительность около 8 секунд."
echo "log=$LOG"
echo

# stdin estimator остаётся открытым. Через 3 с после запуска отправляем SRC2,
# затем даём FC несколько секунд на COMMAND_ACK и штатно останавливаем процесс.
{
  sleep 3
  printf 'SRC2\n'
  sleep 5
} | timeout --signal=INT --kill-after=2s 8s bash "$ROOT/tools/run_ground_motion_mvp.sh" 2>&1 | tee "$LOG"

rc=${PIPESTATUS[1]}

echo
echo "=== РЕЗУЛЬТАТ SRC2 ==="
grep -E 'FC: ArduPilot heartbeat|EKF SOURCE SET:|EKF SOURCE SET ACK:' "$LOG" || true

echo
if grep -q 'EKF SOURCE SET ACK: result=0' "$LOG"; then
  echo "PASS: FC принял MAV_CMD_SET_EKF_SOURCE_SET для SRC2."
  exit 0
elif grep -q 'EKF SOURCE SET: запрос SRC2 отправлен' "$LOG"; then
  echo "PARTIAL: команда SRC2 отправлена, но ACK result=0 не получен."
  exit 2
else
  echo "FAIL: estimator не зафиксировал отправку команды SRC2."
  exit 3
fi
