#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/jtzero_ground_motion_loss_test"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
BLIND_MOVE_MM="${JTZERO_GM_BLIND_MOVE_MM:-150}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${JTZERO_GM_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_BLIND_MOVE_TEST.csv}"
EVENTS="${OUT%.csv}_events.csv"
CONSOLE="${OUT%.csv}_console.log"

mkdir -p "$(dirname "$OUT")"

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

build_binary() {
  local mavlink_inc=""
  local d
  local kimera_root="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"

  for d in \
    "$kimera_root/third_party/mavlink" \
    "$kimera_root/third_party/mavlink/include/mavlink/v2.0" \
    "/usr/local/include/mavlink/v2.0"; do
    if [[ -f "$d/common/mavlink.h" ]]; then
      mavlink_inc="-I$d"
      break
    fi
  done

  [[ -n "$mavlink_inc" ]] || {
    echo "ОШИБКА: common/mavlink.h не найден"
    exit 2
  }

  echo "Собираю диагностический binary из production tools/ground_motion_mvp.cpp ..."
  g++ -std=c++17 -O2 -DNDEBUG -pthread \
    $(pkg-config --cflags opencv4) \
    $mavlink_inc \
    "$ROOT/tools/ground_motion_mvp.cpp" \
    -o "$BIN" \
    $(pkg-config --libs opencv4) -lpthread
}

EST_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${EST_PID:-}" ]] && kill -0 "$EST_PID" 2>/dev/null; then
    kill -INT "$EST_PID" 2>/dev/null || true
    sleep 0.5
    kill -TERM "$EST_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

assert_alive() {
  if [[ -z "${EST_PID:-}" ]] || ! kill -0 "$EST_PID" 2>/dev/null; then
    echo
    echo "ОШИБКА: Ground Motion завершился преждевременно."
    echo "Последние строки журнала:"
    tail -80 "$CONSOLE" 2>/dev/null || true
    exit 1
  fi
}

build_binary

printf 'mono_ns,event,note\n' > "$EVENTS"

echo "======================================================================"
echo "JT-ZERO — BLIND-MOVE TEST GROUND MOTION / EXTERNALNAV"
echo "======================================================================"
echo "Цель: проверить, что происходит, если аппарат реально переместился"
echo "      во время длительной потери visual tracking."
echo
echo "CSV:      $OUT"
echo "EVENTS:   $EVENTS"
echo "CONSOLE:  $CONSOLE"
echo "CAMERA:   $CAMERA"
echo "LUNA:     $LUNA"
echo "FC:       $FC"
echo "BLIND MOVE: ${BLIND_MOVE_MM} мм"
echo
echo "КРИТИЧЕСКИЕ ПРАВИЛА:"
echo "  - до команды движения стенд НЕ двигать;"
echo "  - камера должна оставаться ПОЛНОСТЬЮ закрытой во время blind move;"
echo "  - переместить стенд ровно на ${BLIND_MOVE_MM} мм вдоль рельса;"
echo "  - не вращать, не приподнимать и специально не наклонять стенд;"
echo "  - после blind move стенд больше НЕ двигать до конца теста;"
echo "  - камеру открыть только после отдельной команды скрипта."
echo "======================================================================"

"$BIN" "$CAMERA" "$LUNA" "$FC" "$OUT" "$PARAMS" "$OFFSET" \
  </dev/null >"$CONSOLE" 2>&1 &
EST_PID=$!

for _ in $(seq 1 120); do
  if [[ -s "$OUT" ]]; then
    break
  fi
  assert_alive
  sleep 0.1
done

if [[ ! -s "$OUT" ]]; then
  echo "ОШИБКА: CSV не появился: $OUT"
  tail -80 "$CONSOLE" 2>/dev/null || true
  exit 1
fi

assert_alive
mark BLIND_MOVE_TARGET_MM "$BLIND_MOVE_MM"

echo
read -r -p "Поставьте стенд в исходную точку и НЕ ДВИГАЙТЕ. Когда готовы, нажмите Enter: " _
mark STATIC_PRE_START "исходная статика, камера открыта"
echo "5 секунд исходной статики — камера открыта, стенд НЕ ДВИГАТЬ."
sleep 5
mark STATIC_PRE_END
assert_alive

echo
echo "Теперь ПОЛНОСТЬЮ закройте OV9281 непрозрачным предметом."
echo "Не сдвигайте камеру и стенд."
read -r -p "Когда объектив полностью закрыт, нажмите Enter: " _
mark BLOCK_START "OV9281 полностью закрыта; стенд неподвижен"

echo "3 секунды ждём потери tracking. Камера ЗАКРЫТА. Стенд НЕ ДВИГАТЬ."
sleep 3
mark BLIND_SETTLE_END "3 секунды после закрытия; перед blind move"
assert_alive

echo
echo "Камеру НЕ ОТКРЫВАТЬ."
read -r -p "Нажмите Enter непосредственно перед blind move на ${BLIND_MOVE_MM} мм: " _
mark BLIND_MOVE_START "начало перемещения при полностью закрытой камере"
echo "СЕЙЧАС переместите стенд РОВНО на ${BLIND_MOVE_MM} мм вдоль рельса."
echo "Камера остаётся закрытой. Не вращать и не приподнимать стенд."
read -r -p "Когда стенд полностью остановлен в новой точке, нажмите Enter: " _
mark BLIND_MOVE_END "blind move завершён; стенд остановлен"
assert_alive

echo "После этого стенд БОЛЬШЕ НЕ ДВИГАТЬ до конца теста."
mark BLIND_STATIC_AFTER_MOVE_START "камера всё ещё закрыта; статика после blind move"
echo "3 секунды статики с закрытой камерой после blind move."
sleep 3
mark BLIND_STATIC_AFTER_MOVE_END
assert_alive

mark OPEN_START "оператор начинает открывать OV9281"
printf '\a'
echo
echo "ТЕПЕРЬ откройте OV9281. Стенд не двигать."
read -r -p "Когда объектив полностью открыт, нажмите Enter: " _
mark BLOCK_END "OV9281 полностью открыта"
assert_alive

mark RECOVERY_START "наблюдение повторного захвата ExternalNav"
echo "10 секунд после открытия — стенд НЕ ДВИГАТЬ."
for n in 10 9 8 7 6 5 4 3 2 1; do
  printf '\rОсталось %2d с ' "$n"
  sleep 1
  assert_alive
done
printf '\n'
mark RECOVERY_END
mark DONE

echo "Останавливаю Ground Motion..."
kill -INT "$EST_PID" 2>/dev/null || true
set +e
wait "$EST_PID"
RC=$?
set -e
EST_PID=""

echo
echo "======================================================================"
echo "ТЕСТ ЗАВЕРШЁН"
echo "CSV:      $OUT"
echo "EVENTS:   $EVENTS"
echo "CONSOLE:  $CONSOLE"
echo "======================================================================"

if [[ -f "$ROOT/tools/analyze_ground_motion_loss_test.py" ]]; then
  echo
  python3 "$ROOT/tools/analyze_ground_motion_loss_test.py" "$OUT" "$EVENTS" || true
fi

if [[ -f "$ROOT/tools/analyze_ground_motion_loss_timeline.py" ]]; then
  echo
  python3 "$ROOT/tools/analyze_ground_motion_loss_timeline.py" "$OUT" "$EVENTS" || true
fi

exit "$RC"
