#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
YAML="${JTZERO_FLOW_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams_OF_CurrentMount.yaml}"
FOCAL_SCALE="${JTZERO_FLOW_FOCAL_SCALE:-0.931}"
BIN="/tmp/jtzero_optflow_feature_sweep"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }
[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }

cat <<'EOF'
======================================================================
JT-ZERO — ДЕТЕРМИНИРОВАННЫЙ FEATURE-COUNT SWEEP
======================================================================
Аппарат НЕ ДВИГАТЬ.
FC и TF-Luna не нужны.

Берётся ОДИН реальный кадр OV9281.
Из него программно создаются одинаковые pixel-shift пары.
На каждой одной и той же паре сравниваются:
  500 / 400 / 300 / 200 / 150 features

Измеряются:
  LK runtime
  tracked / inliers
  inlier ratio
  ошибка измеренного pixel displacement

Это контролируемый A/B. Он НЕ заменяет последующий тест на реальном движении.
======================================================================
EOF

g++ -std=c++17 -O2 -DNDEBUG -pthread   -Wno-address-of-packed-member   $(pkg-config --cflags opencv4)   $MAVLINK_INC   "$ROOT/tools/optical_flow_feature_count_sweep.cpp"   -o "$BIN"   $(pkg-config --libs opencv4) -lpthread

exec "$BIN" "$CAMERA" "$YAML" "$FOCAL_SCALE"
