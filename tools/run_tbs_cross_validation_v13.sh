#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_tbs_cross_validation_v13_manifest.csv
CONSOLE=/home/vio/jtzero_tbs_cross_validation_v13_console.txt

cd "$ROOT"

need() { [[ -s "$1" ]] || { echo "MISSING_OR_EMPTY: $1" >&2; exit 2; }; }

for ds in v11 v12 v13; do
  for suffix in .csv _camera.csv .mjpg _backend.csv _legs.csv; do
    need "/home/vio/jtzero_500mm_${ds}${suffix}"
  done
done

g++ -std=c++17 -O2   tools/replay_mono_imu_tbs_sweep_v12.cpp   -I. -Itools -I"$MAVLINK"   -I/home/vio/Kimera-VIO/include   -I/home/vio/Kimera-VIO/build   -I/usr/local/include   -I/usr/include/eigen3   $(pkg-config --cflags opencv4)   -L/home/vio/Kimera-VIO/build   -L/usr/local/lib   -Wl,-rpath,/home/vio/Kimera-VIO/build   -Wl,-rpath,/usr/local/lib   -lkimera_vio -lgtsam -lgflags -lglog -lpthread   $(pkg-config --libs opencv4)   -o "$BIN"

echo 'dataset,candidate,tag,roll_deg,pitch_deg,backend,legs' > "$MANIFEST"
: > "$CONSOLE"

run_one() {
  local ds="$1" cand="$2" roll="$3" pitch="$4"
  local low
  low=$(echo "$ds" | tr '[:upper:]' '[:lower:]')
  local base="/home/vio/jtzero_500mm_${low}"
  local tag="CV13_${ds}_${cand}"
  echo "$ds,$cand,$tag,$roll,$pitch,${base}_backend.csv,${base}_legs.csv" >> "$MANIFEST"
  echo "================ $tag roll=$roll pitch=$pitch ================" | tee -a "$CONSOLE"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}     "$BIN" "$PARAMS" "$tag" "$roll" "$pitch"       "${base}.csv" "${base}_camera.csv" "${base}.mjpg"       "${base}_backend.csv" "${base}_legs.csv"       2>&1 | tee -a "$CONSOLE" |       grep -E '(^\[TBS\]|^LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT)'
}

CANDIDATES=(
  "BASE 0 0"
  "Rm1_Pm5 -1 -5"
  "Rm2_Pm4 -2 -4"
  "Rm2_Pm5 -2 -5"
)

for ds in V11 V12 V13; do
  for row in "${CANDIDATES[@]}"; do
    read -r cand roll pitch <<< "$row"
    run_one "$ds" "$cand" "$roll" "$pitch"
  done
done

python3 tools/analyze_tbs_cross_validation_v13.py | tee /home/vio/jtzero_tbs_cross_validation_v13_top.txt

echo
echo "Saved:"
echo "  $MANIFEST"
echo "  $CONSOLE"
echo "  /home/vio/jtzero_tbs_cross_validation_v13_detail.csv"
echo "  /home/vio/jtzero_tbs_cross_validation_v13_summary.csv"
echo "  /home/vio/jtzero_tbs_cross_validation_v13_top.txt"
