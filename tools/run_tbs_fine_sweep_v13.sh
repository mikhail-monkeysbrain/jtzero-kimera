#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_tbs_fine_v13_manifest.csv
SUMMARY=/home/vio/jtzero_tbs_fine_v13_console.txt

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

echo 'tag,roll_deg,pitch_deg' > "$MANIFEST"
: > "$SUMMARY"

encode_num() {
  local v="$1"
  if [[ "$v" == -* ]]; then
    printf 'm%s' "${v#-}"
  else
    printf 'p%s' "$v"
  fi
}

run_case() {
  local r="$1" p="$2"
  local rt pt tag
  rt=$(encode_num "$r")
  pt=$(encode_num "$p")
  tag="V13_R${rt}_P${pt}"
  echo "$tag,$r,$p" >> "$MANIFEST"
  echo "================ $tag roll=$r pitch=$p ================" | tee -a "$SUMMARY"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" "$tag" "$r" "$p" \
      /home/vio/jtzero_500mm_v13.csv \
      /home/vio/jtzero_500mm_v13_camera.csv \
      /home/vio/jtzero_500mm_v13.mjpg \
      /home/vio/jtzero_500mm_v13_backend.csv \
      /home/vio/jtzero_500mm_v13_legs.csv \
      2>&1 | grep -E 'LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT|\[TBS\]' | tee -a "$SUMMARY"
}

# Narrow grid around the currently validated correction (-2, -5).
# We deliberately extend pitch above -5 because previous fine sweep stopped at -5.
for r in -4 -3 -2 -1 0; do
  for p in -8 -7 -6 -5 -4 -3 -2; do
    run_case "$r" "$p"
  done
done

python3 tools/analyze_tbs_fine_sweep_v13.py | tee /home/vio/jtzero_tbs_fine_v13_top15.txt

echo
echo "Saved:"
echo "  $MANIFEST"
echo "  $SUMMARY"
echo "  /home/vio/jtzero_tbs_fine_v13_ranked.csv"
echo "  /home/vio/jtzero_tbs_fine_v13_top15.txt"
