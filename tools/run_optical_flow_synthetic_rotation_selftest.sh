#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
CAMERA_YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml}"
FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-0.931}"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }
[[ -e "$CAMERA" ]] || { echo "ОШИБКА: OV9281 не найдена: $CAMERA" >&2; exit 2; }
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: YAML не найден: $CAMERA_YAML" >&2; exit 2; }

BIN="/tmp/jtzero_optflow_synthetic_rotation_selftest"

echo "======================================================================"
echo "JT-ZERO — ДЕТЕРМИНИРОВАННЫЙ СИНТЕТИЧЕСКИЙ ТЕСТ ВРАЩЕНИЯ"
echo "======================================================================"
echo "Аппарат НЕ ДВИГАТЬ."
echo "FC и TF-Luna для этого теста не требуются."
echo "Берётся один реальный кадр OV9281 и программно создаются:"
echo "  крен   +5° / -5°"
echo "  тангаж +5° / -5°"
echo "  курс  +10° / -10°"
echo
echo "Затем пары проходят через тот же production KLT/RANSAC/undistort."
echo "======================================================================"

g++ -std=c++17 -O2 -DNDEBUG -pthread \
  -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) \
  $MAVLINK_INC \
  "$ROOT/tools/optical_flow_synthetic_rotation_selftest.cpp" \
  -o "$BIN" \
  $(pkg-config --libs opencv4) -lpthread

exec "$BIN" "$CAMERA" "$CAMERA_YAML" "$FOCAL_SCALE"
