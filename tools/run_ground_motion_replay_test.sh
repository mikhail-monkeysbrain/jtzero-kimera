#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REC_BIN="/tmp/jtzero_ground_motion_replay_record_diag"
REPLAY_BIN="/tmp/jtzero_ground_motion_replay_compare"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
MOVE_MM="${JTZERO_GM_REPLAY_MOVE_MM:-500}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTDIR="${JTZERO_GM_REPLAY_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_REPLAY_DATASET}"
EVENTS="$OUTDIR/events.csv"
CONSOLE="$OUTDIR/record_console.log"
SUMMARY="$OUTDIR/replay_summary.csv"
DETAIL="$OUTDIR/replay_detail.csv"

mkdir -p "$OUTDIR"

mono_ns() {
  python3 - <<'PY'
import time
print(time.monotonic_ns())
PY
}

mark() {
  local event="$1"
  local note="${2:-}"
  printf '%s,%s,%q\n' "$(mono_ns)" "$event" "$note" >> "$EVENTS"
  echo "[EVENT] $event${note:+ — $note}"
}

find_mavlink() {
  local d
  local kimera_root="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
  for d in \
    "$kimera_root/third_party/mavlink" \
    "$kimera_root/third_party/mavlink/include/mavlink/v2.0" \
    "/usr/local/include/mavlink/v2.0"; do
    if [[ -f "$d/common/mavlink.h" ]]; then
      printf '%s' "$d"
      return 0
    fi
  done
  return 1
}

MAVLINK_DIR="$(find_mavlink || true)"
if [[ -z "$MAVLINK_DIR" ]]; then
  echo "ОШИБКА: common/mavlink.h не найден"
  exit 2
fi

echo "Собираю diagnostic recorder и deterministic replay..."
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $(pkg-config --cflags opencv4) -I"$MAVLINK_DIR" \
  "$ROOT/tools/ground_motion_replay_record_diag.cpp" \
  -o "$REC_BIN" \
  $(pkg-config --libs opencv4) -lpthread

g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $(pkg-config --cflags opencv4) -I"$MAVLINK_DIR" \
  "$ROOT/tools/ground_motion_replay_compare.cpp" \
  -o "$REPLAY_BIN" \
  $(pkg-config --libs opencv4) -lpthread

printf 'mono_ns,event,note\n' > "$EVENTS"

REC_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${REC_PID:-}" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    sleep 0.5
    kill -TERM "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

assert_alive() {
  if [[ -z "${REC_PID:-}" ]] || ! kill -0 "$REC_PID" 2>/dev/null; then
    echo "ОШИБКА: recorder завершился преждевременно."
    tail -100 "$CONSOLE" 2>/dev/null || true
    exit 1
  fi
}

echo "======================================================================"
echo "JT-ZERO — DETERMINISTIC GROUND MOTION REPLAY TEST"
echo "======================================================================"
echo "Цель: один физический прогон записать один раз, затем offline сравнить:"
echo "  1) BASELINE_CURRENT — текущее production prev-поведение;"
echo "  2) DROP_CURRENT — кадры действительно отсутствуют;"
echo "  3) REJECT_CURRENT — кадры пришли, но visual-step отвергнут и prev сдвинут;"
echo "  4) REJECT_ANCHOR — тот же reject, но опора остаётся на последнем"
echo "     реально учтённом кадре, пока dt < 0.2 s."
echo
echo "DATASET: $OUTDIR"
echo "MOVE:    ${MOVE_MM} мм"
echo
echo "Правила физического прогона:"
echo "  - камера всё время открыта;"
echo "  - один плавный линейный сдвиг ровно на ${MOVE_MM} мм вдоль рельса;"
echo "  - не вращать и не приподнимать стенд;"
echo "  - после остановки больше не двигать до завершения записи."
echo "======================================================================"

"$REC_BIN" "$CAMERA" "$LUNA" "$FC" "$OUTDIR" "$PARAMS" "$OFFSET" \
  </dev/null >"$CONSOLE" 2>&1 &
REC_PID=$!

for _ in $(seq 1 150); do
  [[ -s "$OUTDIR/frames.csv" ]] && break
  assert_alive
  sleep 0.1
done
[[ -s "$OUTDIR/frames.csv" ]] || {
  echo "ОШИБКА: frames.csv не появился"
  tail -100 "$CONSOLE" 2>/dev/null || true
  exit 1
}

mark MOVE_TARGET_MM "$MOVE_MM"

echo
read -r -p "Поставьте стенд в исходную точку. Когда готовы, нажмите Enter: " _
mark STATIC_PRE_START "исходная статика"
echo "5 секунд статики — НЕ ДВИГАТЬ."
sleep 5
mark STATIC_PRE_END
assert_alive

echo
read -r -p "Нажмите Enter непосредственно перед началом сдвига на ${MOVE_MM} мм: " _
mark MOVE_START "начало единственного измеряемого движения"
echo "СЕЙЧАС плавно переместите стенд РОВНО на ${MOVE_MM} мм вдоль рельса."
echo "Не вращать и не приподнимать."
read -r -p "Когда стенд полностью остановлен, нажмите Enter: " _
mark MOVE_END "стенд полностью остановлен"
assert_alive

echo "5 секунд финальной статики — стенд больше НЕ ДВИГАТЬ."
mark STATIC_POST_START
sleep 5
mark STATIC_POST_END
mark DONE

kill -INT "$REC_PID" 2>/dev/null || true
set +e
wait "$REC_PID"
RC=$?
set -e
REC_PID=""
if [[ "$RC" -ne 0 ]]; then
  echo "ОШИБКА: recorder завершился с кодом $RC"
  tail -100 "$CONSOLE" 2>/dev/null || true
  exit "$RC"
fi

echo
echo "Запись завершена. Запускаю offline replay одной и той же последовательности..."
"$REPLAY_BIN" "$OUTDIR" "$PARAMS" "$EVENTS" "$SUMMARY" "$DETAIL"

echo
echo "======================================================================"
echo "ГОТОВО"
echo "DATASET: $OUTDIR"
echo "EVENTS:  $EVENTS"
echo "SUMMARY: $SUMMARY"
echo "DETAIL:  $DETAIL"
echo "CONSOLE: $CONSOLE"
echo "======================================================================"
