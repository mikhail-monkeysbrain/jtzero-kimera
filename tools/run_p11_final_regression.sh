#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
CAM_BY_ID=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0
TMP=/tmp/jtzero_p11_final_regression
BIN=/tmp/record_p11_final_regression

if [[ ! -e "$CAM_BY_ID" ]]; then
  echo "FATAL: OV9281 camera by-id not found: $CAM_BY_ID" >&2
  echo "Available V4L by-id devices:" >&2
  ls -l /dev/v4l/by-id/ 2>/dev/null || true
  exit 2
fi

REAL_CAM="$(readlink -f "$CAM_BY_ID")"
if [[ ! -c "$REAL_CAM" ]]; then
  echo "FATAL: resolved camera is not a video character device: $REAL_CAM" >&2
  exit 3
fi

echo "[CAM] $CAM_BY_ID -> $REAL_CAM"
echo "[MAV] expected serial: /dev/ttyAMA0 @ 460800"

rm -rf "$TMP"
mkdir -p "$TMP"
cp "$ROOT/tools/camera_imu_extrinsics_logger.cpp" "$TMP/"
cp "$ROOT/tools/camera_imu_timestamp_policy.hpp" "$TMP/"
cp "$ROOT/tools/record_p11_final_regression.cpp" "$TMP/"

python3 - "$TMP/camera_imu_extrinsics_logger.cpp" "$REAL_CAM" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
cam = sys.argv[2]
s = p.read_text()
old = 'constexpr const char* CAMERA_DEVICE="/dev/video0";'
new = f'constexpr const char* CAMERA_DEVICE="{cam}";'
if old not in s:
    raise SystemExit('FATAL: CAMERA_DEVICE declaration not found in temporary logger copy')
p.write_text(s.replace(old, new, 1))
PY

grep -Fq "constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";" "$TMP/camera_imu_extrinsics_logger.cpp" || {
  echo "FATAL: camera substitution verification failed" >&2
  exit 4
}

echo "[BUILD] $BIN"
g++ -std=c++17 -O2 \
  "$TMP/record_p11_final_regression.cpp" \
  -o "$BIN" \
  -I"$TMP" \
  -I"$ROOT/tools" \
  -I"$K/third_party/mavlink" \
  -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  -lpthread \
  $(pkg-config --libs opencv4)

echo "[RUN] P11 final regression"
LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN"
