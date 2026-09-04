#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink

cd "$ROOT"

g++ -std=c++17 -O2 \
  tools/replay_mono_imu_tbs_sweep_v12.cpp \
  -I. -Itools -I"$MAVLINK" \
  -I/home/vio/Kimera-VIO/include \
  -I/home/vio/Kimera-VIO/build \
  -I/usr/local/include \
  -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  -L/home/vio/Kimera-VIO/build \
  -L/usr/local/lib \
  -Wl,-rpath,/home/vio/Kimera-VIO/build \
  -Wl,-rpath,/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread \
  $(pkg-config --libs opencv4) \
  -o "$BIN"

OUT=/home/vio/jtzero_tbs_sweep_v12_summary.txt
: > "$OUT"

run_case() {
  local tag="$1" r="$2" p="$3"
  echo "================ $tag roll=$r pitch=$p ================" | tee -a "$OUT"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" "$tag" "$r" "$p" \
      /home/vio/jtzero_500mm_v12.csv \
      /home/vio/jtzero_500mm_v12_camera.csv \
      /home/vio/jtzero_500mm_v12.mjpg \
      /home/vio/jtzero_500mm_v12_backend.csv \
      /home/vio/jtzero_500mm_v12_legs.csv \
      2>&1 | grep -E 'LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT|\[TBS\]' | tee -a "$OUT"
}

run_case R0_P0 0 0
run_case Rm4_P0 -4 0
run_case Rp4_P0 4 0
run_case Rm8_P0 -8 0
run_case Rp8_P0 8 0
run_case R0_Pm4 0 -4
run_case R0_Pp4 0 4
run_case R0_Pm8 0 -8
run_case R0_Pp8 0 8
run_case Rm6_Pm6 -6 -6
run_case Rm6_Pp6 -6 6
run_case Rp6_Pm6 6 -6
run_case Rp6_Pp6 6 6

echo
echo "Saved: $OUT"
