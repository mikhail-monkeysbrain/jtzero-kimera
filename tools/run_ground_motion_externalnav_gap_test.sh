#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/jtzero_ground_motion_externalnav_gap_test"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
MOVE_MM="${JTZERO_GM_EXTNAV_GAP_MOVE_MM:-175}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${JTZERO_GM_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_EXTNAV_GAP_TEST.csv}"
EVENTS="${OUT%.csv}_events.csv"
CONSOLE="${OUT%.csv}_console.log"
GATE="/tmp/jtzero_gm_extnav_gate_${$}"

mkdir -p "$(dirname "$OUT")"
rm -f "$GATE"
export JTZERO_GM_EXTNAV_GATE_FILE="$GATE"

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

  echo "Собираю diagnostic binary с ExternalNav publication gate..."
  g++ -std=c++17 -O2 -DNDEBUG -pthread \
    $(pkg-config --cflags opencv4) \
    $mavlink_inc \
    "$ROOT/tools/ground_motion_mvp_externalnav_gate_diag.cpp" \
    -o "$BIN" \
    $(pkg-config --libs opencv4) -lpthread
}

EST_PID=""
cleanup() {
  trap - INT TERM EXIT
  rm -f "$GATE"
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
    tail -80 "$CONSOLE" 2>/dev/null || true
    exit 1
  fi
}

build_binary
printf 'mono_ns,event,note\n' > "$EVENTS"

echo "======================================================================"
echo "JT-ZERO — EXTERNALNAV GAP TEST"
echo "======================================================================"
echo "Цель: камера и Ground Motion продолжают нормально измерять движение,"
echo "      но VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE временно"
echo "      программно НЕ отправляются в FC."
echo
echo "CSV:      $OUT"
echo "EVENTS:   $EVENTS"
echo "CONSOLE:  $CONSOLE"
echo "MOVE:     ${MOVE_MM} мм"
echo
echo "КРИТИЧЕСКИЕ ПРАВИЛА:"
echo "  - OV9281 всё время ОТКРЫТА; ничего перед камерой не ставить;"
echo "  - перемещение только вдоль рельса;"
echo "  - переместить стенд ровно на ${MOVE_MM} мм;"
echo "  - не вращать и не приподнимать стенд;"
echo "  - после перемещения стенд больше не двигать до конца теста."
echo "======================================================================"

"$BIN" "$CAMERA" "$LUNA" "$FC" "$OUT" "$PARAMS" "$OFFSET" \
  </dev/null >"$CONSOLE" 2>&1 &
EST_PID=$!

for _ in $(seq 1 120); do
  [[ -s "$OUT" ]] && break
  assert_alive
  sleep 0.1
done
[[ -s "$OUT" ]] || { echo "ОШИБКА: CSV не появился: $OUT"; exit 1; }

mark MOVE_TARGET_MM "$MOVE_MM"

echo
read -r -p "Поставьте стенд в исходную точку. Камера ОТКРЫТА. Когда готовы, нажмите Enter: " _
mark STATIC_PRE_START "исходная статика; ExternalNav ON"
echo "5 секунд статики — НЕ ДВИГАТЬ."
sleep 5
mark STATIC_PRE_END
assert_alive

echo
touch "$GATE"
mark GAP_START "ExternalNav publication OFF; GM продолжает интегрировать"
echo "ExternalNav ОТКЛЮЧЁН программно. Камера остаётся открытой."
echo "3 секунды статики перед движением — НЕ ДВИГАТЬ."
sleep 3
mark GAP_SETTLE_END
assert_alive

echo
read -r -p "Нажмите Enter непосредственно перед перемещением на ${MOVE_MM} мм: " _
mark GAP_MOVE_START "начало физического движения при ExternalNav OFF"
echo "СЕЙЧАС переместите стенд РОВНО на ${MOVE_MM} мм вдоль рельса."
echo "Камера ОТКРЫТА. Не вращать и не приподнимать стенд."
read -r -p "Когда стенд полностью остановлен, нажмите Enter: " _
mark GAP_MOVE_END "физическое движение завершено"
assert_alive

echo "После этого стенд БОЛЬШЕ НЕ ДВИГАТЬ."
mark GAP_STATIC_AFTER_MOVE_START
sleep 3
mark GAP_STATIC_AFTER_MOVE_END
assert_alive

echo
mark GAP_END "ExternalNav publication ON"
rm -f "$GATE"
echo "ExternalNav снова ВКЛЮЧЁН. Стенд НЕ ДВИГАТЬ."
mark RECOVERY_START
for n in 10 9 8 7 6 5 4 3 2 1; do
  printf '\rRecovery: осталось %2d с ' "$n"
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
rm -f "$GATE"

echo
echo "======================================================================"
echo "ТЕСТ ЗАВЕРШЁН"
echo "CSV:      $OUT"
echo "EVENTS:   $EVENTS"
echo "CONSOLE:  $CONSOLE"
echo "======================================================================"

if [[ -f "$ROOT/tools/analyze_ground_motion_externalnav_gap_test.py" ]]; then
  echo
  python3 "$ROOT/tools/analyze_ground_motion_externalnav_gap_test.py" "$OUT" "$EVENTS" || true
fi

exit "$RC"
