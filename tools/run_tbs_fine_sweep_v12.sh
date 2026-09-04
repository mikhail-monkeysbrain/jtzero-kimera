#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_tbs_fine_v12_manifest.csv
SUMMARY=/home/vio/jtzero_tbs_fine_v12_console.txt

cd "$ROOT"

# Rebuild once from the current branch state.
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
  tag="F_R${rt}_P${pt}"
  echo "$tag,$r,$p" >> "$MANIFEST"
  echo "================ $tag roll=$r pitch=$p ================" | tee -a "$SUMMARY"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" "$tag" "$r" "$p" \
      /home/vio/jtzero_500mm_v12.csv \
      /home/vio/jtzero_500mm_v12_camera.csv \
      /home/vio/jtzero_500mm_v12.mjpg \
      /home/vio/jtzero_500mm_v12_backend.csv \
      /home/vio/jtzero_500mm_v12_legs.csv \
      2>&1 | grep -E 'LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT|\[TBS\]' | tee -a "$SUMMARY"
}

# Fine grid around the best useful coarse region.
# roll: -3..+3 deg, pitch: -11..-5 deg => 49 deterministic replays.
for r in -3 -2 -1 0 1 2 3; do
  for p in -11 -10 -9 -8 -7 -6 -5; do
    run_case "$r" "$p"
  done
done

python3 tools/analyze_tbs_fine_sweep_v12.py | tee /home/vio/jtzero_tbs_fine_v12_top15.txt

echo
echo "Saved:"
echo "  $MANIFEST"
echo "  $SUMMARY"
echo "  /home/vio/jtzero_tbs_fine_v12_ranked.csv"
echo "  /home/vio/jtzero_tbs_fine_v12_top15.txt"
