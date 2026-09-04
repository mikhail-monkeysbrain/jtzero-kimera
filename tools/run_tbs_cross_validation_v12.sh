#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_tbs_cross_validation_manifest.csv
CONSOLE=/home/vio/jtzero_tbs_cross_validation_console.txt

cd "$ROOT"

need_file() {
  if [[ ! -s "$1" ]]; then
    echo "MISSING_OR_EMPTY: $1" >&2
    exit 2
  fi
}

# Independent datasets. V11 is intentionally used as cross-validation because
# it was recorded in a different physical run / initial attitude.
for f in \
  /home/vio/jtzero_500mm_v11.csv \
  /home/vio/jtzero_500mm_v11_camera.csv \
  /home/vio/jtzero_500mm_v11.mjpg \
  /home/vio/jtzero_500mm_v11_backend.csv \
  /home/vio/jtzero_500mm_v11_legs.csv \
  /home/vio/jtzero_500mm_v12.csv \
  /home/vio/jtzero_500mm_v12_camera.csv \
  /home/vio/jtzero_500mm_v12.mjpg \
  /home/vio/jtzero_500mm_v12_backend.csv \
  /home/vio/jtzero_500mm_v12_legs.csv; do
  need_file "$f"
done

echo "[BUILD] replay_mono_imu_tbs_sweep_v12.cpp"
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

cat > "$MANIFEST" <<'EOF'
dataset,candidate,tag,roll_deg,pitch_deg,backend,legs
EOF
: > "$CONSOLE"

run_one() {
  local ds="$1" cand="$2" roll="$3" pitch="$4"
  local imu cam mjpg backend legs tag
  if [[ "$ds" == "V11" ]]; then
    imu=/home/vio/jtzero_500mm_v11.csv
    cam=/home/vio/jtzero_500mm_v11_camera.csv
    mjpg=/home/vio/jtzero_500mm_v11.mjpg
    backend=/home/vio/jtzero_500mm_v11_backend.csv
    legs=/home/vio/jtzero_500mm_v11_legs.csv
  else
    imu=/home/vio/jtzero_500mm_v12.csv
    cam=/home/vio/jtzero_500mm_v12_camera.csv
    mjpg=/home/vio/jtzero_500mm_v12.mjpg
    backend=/home/vio/jtzero_500mm_v12_backend.csv
    legs=/home/vio/jtzero_500mm_v12_legs.csv
  fi
  tag="CV_${ds}_${cand}"
  echo "$ds,$cand,$tag,$roll,$pitch,$backend,$legs" >> "$MANIFEST"
  echo "================ $tag roll=$roll pitch=$pitch ================" | tee -a "$CONSOLE"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" "$tag" "$roll" "$pitch" \
      "$imu" "$cam" "$mjpg" "$backend" "$legs" \
      2>&1 | tee -a "$CONSOLE" | \
      grep -E '(^\[TBS\]|^LEG [1-6] |LEG_Z_RMS_MM|final \|dP\||REPLAY RESULT)'
}

# Baseline plus the most informative candidates from the V12 fine sweep.
# Keep this list short: the goal is robustness across an independent run,
# not another single-dataset optimization.
CANDIDATES=(
  "BASE 0 0"
  "BEST_Rp2_Pm9 2 -9"
  "NEIGH_Rp2_Pm8 2 -8"
  "LOWZ_Rm3_Pm8 -3 -8"
  "BAL_Rm2_Pm5 -2 -5"
  "LOWZ_Rm3_Pm10 -3 -10"
)

for ds in V11 V12; do
  for row in "${CANDIDATES[@]}"; do
    read -r cand roll pitch <<< "$row"
    run_one "$ds" "$cand" "$roll" "$pitch"
  done
done

python3 tools/analyze_tbs_cross_validation_v12.py

echo
echo "Saved:"
echo "  $MANIFEST"
echo "  $CONSOLE"
echo "  /home/vio/jtzero_tbs_cross_validation_detail.csv"
echo "  /home/vio/jtzero_tbs_cross_validation_summary.csv"
echo "  /home/vio/jtzero_tbs_cross_validation_top.txt"
