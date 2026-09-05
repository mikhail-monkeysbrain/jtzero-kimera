#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
BASE="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5"
WORK=/tmp/jtzero_v21_acc_rw_sweep
OUT=/home/vio/jtzero_v21_acc_rw_sweep.txt

cd "$ROOT"

g++ -std=c++17 -O2 tools/replay_mono_imu_tbs_sweep_v12.cpp   -I. -Itools   -I/home/vio/Kimera-VIO/include   -I/home/vio/Kimera-VIO/build   -I/home/vio/Kimera-VIO/third_party/mavlink   -I/usr/include/eigen3   $(pkg-config --cflags opencv4)   -L/home/vio/Kimera-VIO/build   -L/usr/local/lib   -Wl,-rpath,/home/vio/Kimera-VIO/build   -Wl,-rpath,/usr/local/lib   -lkimera_vio -lgtsam -lgflags -lglog -lpthread   $(pkg-config --libs opencv4)   -o "$BIN"

for f in   /home/vio/jtzero_500mm_v18.csv   /home/vio/jtzero_500mm_v18_camera.csv   /home/vio/jtzero_500mm_v18_backend.csv   /home/vio/jtzero_500mm_v18_legs.csv; do
  [[ -s "$f" ]] || { echo "Missing: $f" >&2; exit 2; }
done

rm -rf "$WORK"
mkdir -p "$WORK"
: > "$OUT"

# Raw MJPEG is intentionally empty in V18. Replay source accepts the path;
# camera timing/data are taken from the saved camera stream used by the harness.
MJPG=/home/vio/jtzero_500mm_v18.mjpg

for rw in 0.030 0.015 0.010 0.005 0.003 0.001; do
  tag=$(echo "$rw" | tr '.' 'p')
  P="$WORK/rw_${tag}"
  cp -a "$BASE" "$P"
  sed -i -E "s/^accelerometer_random_walk:[[:space:]]*[^[:space:]]+/accelerometer_random_walk: ${rw}/" "$P/ImuParams.yaml"

  NAME="V18_ARW_${tag}"
  echo "================ accelerometer_random_walk=$rw ================" | tee -a "$OUT"

  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}     "$BIN" "$P" "$NAME" 0 0       /home/vio/jtzero_500mm_v18.csv       /home/vio/jtzero_500mm_v18_camera.csv       "$MJPG"       /home/vio/jtzero_500mm_v18_backend.csv       /home/vio/jtzero_500mm_v18_legs.csv       2>&1 | tee "/home/vio/jtzero_v21_arw_${tag}.log" |       grep -E '(^LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT)' | tee -a "$OUT"
done

python3 tools/analyze_v21_acc_rw_sweep.py | tee /home/vio/jtzero_v21_acc_rw_ranked.txt

echo
echo "Saved:"
echo "  $OUT"
echo "  /home/vio/jtzero_v21_acc_rw_ranked.csv"
echo "  /home/vio/jtzero_v21_acc_rw_ranked.txt"
