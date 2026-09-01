#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/live_mono_imu_500mm_hud_v2.cpp"
GEN="/tmp/live_mono_imu_300mm_hud_v2.cpp"
OUT="/tmp/live_mono_imu_300mm_hud_v2"

if [[ ! -f "$SRC" ]]; then
  echo "Source not found: $SRC" >&2
  exit 1
fi

# Generate a dedicated 300 mm variant from the already-tested low-load HUD v2.
# Only experiment-specific constants/text/path are changed; camera, IMU,
# synchronization, Kimera feeding and HUD scheduling stay identical.
sed \
  -e 's#jtzero_live_500mm_hud_v2.csv#jtzero_live_300mm_hud_v2.csv#g' \
  -e 's#JT-ZERO 500 mm LIVE HUD v2#JT-ZERO 300 mm LIVE HUD v2#g' \
  -e 's#kExpectedDistanceM = 0.500#kExpectedDistanceM = 0.300#g' \
  -e 's#kDirectionLearnDistanceM = 0.050#kDirectionLearnDistanceM = 0.030#g' \
  -e 's#JT-ZERO LIVE 500 mm#JT-ZERO LIVE 300 mm#g' \
  -e 's#TRAVEL %.0f / 500 mm#TRAVEL %.0f / 300 mm#g' \
  -e 's#MOVE NOW: exactly 500 mm#MOVE NOW: exactly 300 mm#g' \
  -e 's#500 MM HUD V2 RESULT#300 MM HUD V2 RESULT#g' \
  "$SRC" > "$GEN"

g++ -std=c++17 -O3 \
  "$GEN" \
  -I/home/vio/Kimera-VIO/include \
  -I/home/vio/Kimera-VIO/third_party/mavlink \
  -I"$ROOT/tools" \
  -I/usr/include/eigen3 \
  -L/home/vio/Kimera-VIO/build \
  -lkimera_vio \
  $(pkg-config --cflags --libs opencv4) \
  -lglog -lgflags -lgtsam -lgtsam_unstable -lpthread \
  -o "$OUT"

echo "Built: $OUT"
echo "CSV:   /home/vio/jtzero_live_300mm_hud_v2.csv"
