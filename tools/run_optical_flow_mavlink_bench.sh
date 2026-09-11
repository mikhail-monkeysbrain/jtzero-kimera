#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_FLOW_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
CAMERA_YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
# Независимая ChArUco physical-height оценка текущей OV9281 дала k=1.0925.
# Можно переопределить JTZERO_FLOW_FOCAL_SCALE=1.0 для raw saved-K сравнения.
FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.0925}"
GUIDED_MODE="${JTZERO_FLOW_GUIDED_MODE:-guided-175}"
EXTRA_ARGS=(--guided-175)
if [[ "$GUIDED_MODE" == "armed" ]]; then
  EXTRA_ARGS+=(--require-armed)
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${JTZERO_FLOW_RUN_DIR:-/home/vio/jtzero_runs/${STAMP}_OPTICAL_FLOW_MAVLINK_BENCH}"
BIN="$RUN_DIR/optical_flow_mavlink_mvp"
CSV="$RUN_DIR/optical_flow_mavlink.csv"
BUILD_LOG="$RUN_DIR/build.log"
mkdir -p "$RUN_DIR"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: common/ и ardupilotmega/ MAVLink headers не найдены"; exit 2; }
[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA"; exit 2; }
[[ -e "$LUNA" ]] || { echo "ОШИБКА: TF-Luna port не найден: $LUNA"; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC port не найден: $FC"; exit 2; }
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: camera yaml не найден: $CAMERA_YAML"; exit 2; }

echo "Собираю отдельный OpticalFlow MAVLink MVP v2 (с EKF_STATUS_REPORT)..."
if ! g++ -std=c++17 -O2 -DNDEBUG -pthread \
  -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_mavlink_mvp_v2.cpp" \
  -o "$BIN" \
  $(pkg-config --libs opencv4) -lpthread \
  >"$BUILD_LOG" 2>&1; then
  echo "ОШИБКА СБОРКИ. Последние 80 строк:"
  tail -80 "$BUILD_LOG"
  exit 1
fi

echo "Сборка OK: $BIN"
echo "BUILD: $BUILD_LOG"
echo
cat <<EOF
======================================================================
JT-ZERO — MAVLink OPTICAL FLOW BENCH v2
======================================================================
Это ОТДЕЛЬНЫЙ диагностический контур. production ground_motion_mvp.cpp не меняется.

Параметры, которые должны быть выставлены перед fusion-тестом:
  FLOW_TYPE          = 5      # MAVLink; после изменения нужен reboot FC
  FLOW_OPTIONS       = 0      # камера жёстко закреплена, НЕ gimbal-stabilised
  FLOW_ORIENT_YAW    = 0      # publisher уже поворачивает flow в body FRD
  FLOW_FXSCALER      = 0
  FLOW_FYSCALER      = 0
  EK3_FLOW_DELAY     = 0      # стартовая bench-гипотеза для USB, НЕ BlueOS 150 ms
  EK3_SRC1_POSXY     = 0      # None
  EK3_SRC1_VELXY     = 5      # OpticalFlow
  EK3_SRC1_POSZ      = 2      # RangeFinder, как в текущем JT-Zero
  EK3_SRC1_VELZ      = 0
  EK3_SRC1_YAW       = 0      # None; текущий GPS-denied bench без независимого yaw
  EK3_SRC_OPTIONS    = 0

RNGFND параметры НЕ меняем: текущий TF-Luna -> DISTANCE_SENSOR уже проверен.

camera:      $CAMERA
luna:        $LUNA
fc:          $FC
camera yaml: $CAMERA_YAML
focal scale: $FOCAL_SCALE
CSV:         $CSV

Диагностика теперь печатает одновременно:
  rateFRD    — наш raw optical flow;
  EKFSTAT    — EKF_STATUS_REPORT flags;
  LOCAL      — LOCAL_POSITION_NED, если ArduPilot считает position/velocity валидными.

РЕЖИМ: guided 175 мм.

ИНСТРУКЦИЯ БУДЕТ ПОВТОРЕНА САМОЙ ПРОГРАММОЙ:
  1) сначала 5 секунд вообще не двигать аппарат;
  2) дождаться строки "ДВИГАЙТЕ";
  3) сдвинуть ВЕСЬ аппарат строго по столу на 175 мм;
  4) не вращать, не наклонять и не приподнимать;
  5) полностью остановить аппарат и только тогда нажать Enter;
  6) после Enter ещё 5 секунд ничего не трогать;
  7) тест остановится автоматически и напечатает DELTA N/E, длину и ошибку.

Не начинайте движение до явной команды программы.
======================================================================
EOF

read -r -p "Параметры проверены. EKF origin для OpticalFlow relative aiding не требуется. Запустить? [Enter] " _

exec "$BIN" "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL_SCALE" "${EXTRA_ARGS[@]}"
