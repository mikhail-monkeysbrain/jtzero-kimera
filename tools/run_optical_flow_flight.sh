#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_FLOW_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
CAMERA_YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-1.1060}"
FEATURE_ROI="${JTZERO_FLOW_FEATURE_ROI:-0.20 0.20 0.80 0.80}"
RETURN_GUI="${JTZERO_FLOW_RETURN_GUI:-0}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${JTZERO_FLOW_RUN_DIR:-/home/vio/jtzero_runs/${STAMP}_OPTICAL_FLOW_FLIGHT}"
BIN="$RUN_DIR/optical_flow_mavlink_mvp"
CSV="$RUN_DIR/optical_flow_mavlink.csv"
BUILD_LOG="$RUN_DIR/build.log"
mkdir -p "$RUN_DIR"

bash "$ROOT/tools/audit_optical_flow_flight_params.sh"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done

[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }
[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }
[[ -e "$LUNA" ]] || { echo "ОШИБКА: TF-Luna port не найден: $LUNA" >&2; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC port не найден: $FC" >&2; exit 2; }
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: camera yaml не найден: $CAMERA_YAML" >&2; exit 2; }

echo
echo "Собираю OpticalFlow flight publisher..."
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

cat <<EOF
======================================================================
JT-ZERO — OPTICAL FLOW FLIGHT
======================================================================
РЕАЛЬНЫЙ FLIGHT-КОНТУР:

  OV9281 -> OPTICAL_FLOW
  TF-Luna -> DISTANCE_SENSOR
  ArduPilot EKF3 -> horizontal velocity/relative position

КРИТИЧЕСКИ:
  - synthetic bench height НЕ используется;
  - flow НЕ масштабируется lm/0.60;
  - в FC уходит настоящий TF-Luna range;
  - focal_scale = $FOCAL_SCALE;
  - feature ROI = $FEATURE_ROI;
  - return GUI = $RETURN_GUI;
  - EK3_FLOW_DELAY должен оставаться 0;
  - publisher работает непрерывно до Ctrl+C.

camera:      $CAMERA
luna:        $LUNA
fc:          $FC
camera yaml: $CAMERA_YAML
CSV:         $CSV
BUILD:       $BUILD_LOG

Этот launcher НЕ ARM-ит FC и НЕ меняет flight mode.
======================================================================
EOF

read -r RX0 RY0 RX1 RY1 <<< "$FEATURE_ROI"
EXTRA=(--feature-roi "$RX0" "$RY0" "$RX1" "$RY1")
if [[ "$RETURN_GUI" == "1" ]]; then
  EXTRA+=(--return-gui)
fi

exec "$BIN" "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL_SCALE" "${EXTRA[@]}"
