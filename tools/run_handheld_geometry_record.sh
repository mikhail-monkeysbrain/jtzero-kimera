#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_FLOW_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
SAVE_EVERY="${JTZERO_GEOM_SAVE_EVERY:-5}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${JTZERO_GEOM_RUN_DIR:-/home/vio/jtzero_runs/${STAMP}_HANDHELD_GEOMETRY}"
BIN="$RUN_DIR/handheld_geometry_record"
BUILD_LOG="$RUN_DIR/build.log"
mkdir -p "$RUN_DIR"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены"; exit 2; }

if ! g++ -std=c++17 -O2 -DNDEBUG -pthread \
  -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) \
  $MAVLINK_INC \
  "$ROOT/tools/handheld_geometry_record.cpp" \
  -o "$BIN" \
  $(pkg-config --libs opencv4) -lpthread \
  >"$BUILD_LOG" 2>&1; then
  tail -80 "$BUILD_LOG"
  exit 1
fi

cat <<EOF
======================================================================
JT-ZERO — HANDHELD GEOMETRY IDENTIFICATION RECORD
======================================================================
Пропеллеры снять. FC НЕ требуется ARM.

Записываются:
  - OV9281 MJPG кадры (~каждый $SAVE_EVERY-й);
  - TF-Luna;
  - FC ATTITUDE;
  - FC HIGHRES_IMU (fallback RAW_IMU).

Движение руками разрешено. Шарнир не нужен.

Рекомендуемый сценарий:
  10 с неподвижно
  10 с pitch ±10..15°
  10 с roll  ±10..15°
  10 с yaw   ±30..45°
  20 с комбинированное плавное движение
  10 с неподвижно

Для последующей метрической идентификации камера должна большую часть времени
видеть неподвижную ChArUco-доску на плоскости пола/стола.

RUN_DIR=$RUN_DIR
======================================================================
EOF

read -r -p "Начать запись? [Enter] " _
exec "$BIN" "$CAMERA" "$LUNA" "$FC" "$RUN_DIR" "$SAVE_EVERY"
