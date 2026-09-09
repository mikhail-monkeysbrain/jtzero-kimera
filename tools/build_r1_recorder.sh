#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
SRC="$ROOT/tools/live_mono_imu_500mm_v43_camera_forensic.cpp"
OUT="${1:-/tmp/live_mono_imu_500mm_r1_recorder}"
CXX="${CXX:-g++}"

MAVLINK="$KIMERA_ROOT/third_party/mavlink"
if [[ ! -f "$MAVLINK/common/mavlink.h" ]]; then
  echo "ERROR: expected MAVLink header missing: $MAVLINK/common/mavlink.h" >&2
  exit 2
fi
if [[ ! -f "$SRC" ]]; then
  echo "ERROR: R1 source missing: $SRC" >&2
  exit 2
fi

OPENCV_CFLAGS="$(pkg-config --cflags opencv4)"
OPENCV_LIBS="$(pkg-config --libs opencv4)"

rm -f "$OUT"

echo "===== R1 RECORDER BUILD ====="
echo "source=$SRC"
echo "output=$OUT"
echo "kimera=$KIMERA_ROOT"
echo "mavlink=$MAVLINK"

"$CXX" -std=c++17 -O2 -DNDEBUG -pthread \
  $OPENCV_CFLAGS \
  -I"$ROOT/tools" \
  -I"$KIMERA_ROOT/include" \
  -I"$KIMERA_ROOT/build" \
  -I"$MAVLINK" \
  -I/usr/local/include \
  -I/usr/include/eigen3 \
  "$SRC" -o "$OUT" \
  -L"$KIMERA_ROOT/build" \
  -L/usr/local/lib \
  -Wl,-rpath,"$KIMERA_ROOT/build:/usr/local/lib" \
  -lkimera_vio \
  -lgtsam -lgtsam_unstable \
  -lKimeraRPGO \
  -lgflags -lglog \
  -lboost_system \
  $OPENCV_LIBS \
  -ldl -lpthread

[[ -x "$OUT" ]] || { echo "ERROR: output binary not created" >&2; exit 3; }

for marker in \
  "R1 raw MJPEG write failed" \
  "/home/vio/jtzero_500mm_v25.mjpg" \
  "Arducam_OV9281" \
  "V42-LUNA"
do
  if ! strings "$OUT" | grep -Fq "$marker"; then
    echo "ERROR: R1 marker missing from binary: $marker" >&2
    rm -f "$OUT"
    exit 4
  fi
done

echo
echo "===== R1 BINARY IDENTITY ====="
sha256sum "$OUT"
stat -c 'bytes=%s mtime=%y' "$OUT"
echo
echo "BUILD_R1_RECORDER PASS"
