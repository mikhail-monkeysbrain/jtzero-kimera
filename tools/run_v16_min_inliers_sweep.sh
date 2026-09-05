#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
BASE="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5"
WORK=/tmp/jtzero_v16_frontend_sweep
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
OUT=/home/vio/jtzero_v16_min_inliers_sweep.txt

cd "$ROOT"

g++ -std=c++17 -O2 tools/replay_mono_imu_tbs_sweep_v12.cpp   -I. -Itools -I"$MAVLINK"   -I/home/vio/Kimera-VIO/include   -I/home/vio/Kimera-VIO/build   -I/usr/local/include   -I/usr/include/eigen3   $(pkg-config --cflags opencv4)   -L/home/vio/Kimera-VIO/build   -L/usr/local/lib   -Wl,-rpath,/home/vio/Kimera-VIO/build   -Wl,-rpath,/usr/local/lib   -lkimera_vio -lgtsam -lgflags -lglog -lpthread   $(pkg-config --libs opencv4)   -o "$BIN"

for f in   /home/vio/jtzero_500mm_v15.csv   /home/vio/jtzero_500mm_v15_camera.csv   /home/vio/jtzero_500mm_v15.mjpg   /home/vio/jtzero_500mm_v15_backend.csv   /home/vio/jtzero_500mm_v15_legs.csv; do
  [[ -s "$f" ]] || { echo "Missing: $f" >&2; exit 2; }
done

rm -rf "$WORK"
mkdir -p "$WORK"
: > "$OUT"

for n in 10 12 15 20 25 30; do
  P="$WORK/min${n}"
  cp -a "$BASE" "$P"
  sed -i -E "s/^minNrMonoInliers:[[:space:]]*[0-9]+/minNrMonoInliers: ${n}/" "$P/FrontendParams.yaml"
  tag="V15_MIN${n}"

  echo "================ minNrMonoInliers=$n ================" | tee -a "$OUT"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}     "$BIN" "$P" "$tag" 0 0       /home/vio/jtzero_500mm_v15.csv       /home/vio/jtzero_500mm_v15_camera.csv       /home/vio/jtzero_500mm_v15.mjpg       /home/vio/jtzero_500mm_v15_backend.csv       /home/vio/jtzero_500mm_v15_legs.csv       2>&1 | tee "/home/vio/jtzero_v16_min${n}.log" |       grep -E '(^LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT|\[TBS\])' | tee -a "$OUT"
done

python3 tools/analyze_v16_min_inliers_sweep.py | tee /home/vio/jtzero_v16_min_inliers_ranked.txt

echo
echo "Saved:"
echo "  $OUT"
echo "  /home/vio/jtzero_v16_min_inliers_ranked.csv"
echo "  /home/vio/jtzero_v16_min_inliers_ranked.txt"
