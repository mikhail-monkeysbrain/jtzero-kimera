#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
OUT="${1:-/tmp/live_mono_imu_500mm_repeat_hud_v25}"

CXX="${CXX:-g++}"

OPENCV_CFLAGS="$(pkg-config --cflags opencv4)"
OPENCV_LIBS="$(pkg-config --libs opencv4)"

MAVLINK_INC=""
for d in   "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0"   "$KIMERA_ROOT/third_party/mavlink/include"   "/usr/local/include/mavlink/v2.0"   "/usr/local/include/mavlink"; do
  if [ -f "$d/common/mavlink.h" ]; then
    MAVLINK_INC="-I$d"
    break
  fi
done

if [ -z "$MAVLINK_INC" ]; then
  echo "ERROR: common/mavlink.h not found" >&2
  exit 2
fi

echo "Building: $OUT"
echo "MAVLink:  ${MAVLINK_INC#-I}"

"$CXX"   -std=c++17 -O2 -DNDEBUG -pthread   $OPENCV_CFLAGS   -I"$ROOT/tools"   -I"$KIMERA_ROOT/include"   -I/usr/local/include   -I/usr/include/eigen3   $MAVLINK_INC   "$ROOT/tools/live_mono_imu_500mm_repeat_hud_v25.cpp"   -o "$OUT"   -L"$KIMERA_ROOT/build"   -L/usr/local/lib   -Wl,-rpath,"$KIMERA_ROOT/build:/usr/local/lib"   -lkimera_vio   -lgtsam -lgtsam_unstable   -lKimeraRPGO   -lgflags -lglog   -lboost_system   $OPENCV_LIBS   -ldl -lpthread

echo "OK: $OUT"
ldd "$OUT" | grep -Ei 'kimera|gtsam|opencv' | head -30 || true
