#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"
BASE_PARAMS="$ROOT/params/JTZeroMonoFLU"
TEST_PARAMS=/tmp/JTZeroMonoFLU_v15_16_no2d2d
BIN=/tmp/replay_2d2d_ab_yaw_v15_16

COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"

rm -rf "$TEST_PARAMS"
cp -a "$BASE_PARAMS" "$TEST_PARAMS"

FRONTEND="$TEST_PARAMS/FrontendParams.yaml"
COUNT=$(grep -c '^use_2d2d_tracking: 1$' "$FRONTEND" || true)
if [[ "$COUNT" != "1" ]]; then
  echo "[FATAL] expected exactly one 'use_2d2d_tracking: 1', got $COUNT"
  exit 2
fi
sed -i 's/^use_2d2d_tracking: 1$/use_2d2d_tracking: 0/' "$FRONTEND"

# Verify this is the only parameter-file difference.
diff -ru "$BASE_PARAMS" "$TEST_PARAMS" > /tmp/jtzero_v15_16_params.diff || true
DIFF_LINES=$(grep -E '^[+-][^+-]' /tmp/jtzero_v15_16_params.diff | wc -l)
if [[ "$DIFF_LINES" != "2" ]]; then
  echo "[FATAL] expected exactly two content diff lines (+/-), got $DIFF_LINES"
  cat /tmp/jtzero_v15_16_params.diff
  exit 3
fi

echo "============================================================"
echo "JT-ZERO FRONTEND 2D-2D TRACKING A/B YAW REPLAY v15.16"
echo "============================================================"
echo "A: CURRENT  use_2d2d_tracking=1"
echo "B: NO_2D2D  use_2d2d_tracking=0"
echo "Only FrontendParams.yaml changes. IMU, camera, R_BC, t_BC, backend and gravity logic are identical."
echo
echo "Parameter diff:"
cat /tmp/jtzero_v15_16_params.diff

g++ -std=c++17 -O2 \
  -I"$ROOT/tools" \
  -I/home/vio/Kimera-VIO/include \
  -I/home/vio/Kimera-VIO/build \
  -I/home/vio/Kimera-VIO/third_party/mavlink \
  -I/usr/include/eigen3 \
  "$SRC" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4) \
  -L/home/vio/Kimera-VIO/build -L/usr/local/lib \
  -lkimera_vio -lgtsam -lglog -lgflags -lpthread

run_one() {
  local tag="$1"
  local params="$2"
  local log="/home/vio/jtzero_2d2d_ab_v15_16_${tag}.txt"
  local csv="/home/vio/jtzero_2d2d_ab_v15_16_${tag}.csv"

  echo
  echo "================ $tag ================================"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$params" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$log"
  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"
}

run_one CURRENT "$BASE_PARAMS"
run_one NO_2D2D "$TEST_PARAMS"

SUMMARY=/home/vio/jtzero_2d2d_ab_v15_16_summary.txt
{
  echo "================ v15.16 SUMMARY ========================"
  for tag in CURRENT NO_2D2D; do
    echo "$tag"
    grep -E '^(backend states:|integral FED gyro deg XYZ:|final dP mm:|final \|dP\| mm:|path length mm:|max excursion mm:|max speed mm/s:|orientation span deg|final dRPY deg:|REPLAY RESULT:)' "/home/vio/jtzero_2d2d_ab_v15_16_${tag}.txt" || true
    echo
  done
} | tee "$SUMMARY"

echo "State CSVs: /home/vio/jtzero_2d2d_ab_v15_16_{CURRENT,NO_2D2D}.csv"
echo "Summary: $SUMMARY"
echo "RESULT: COMPLETE"
