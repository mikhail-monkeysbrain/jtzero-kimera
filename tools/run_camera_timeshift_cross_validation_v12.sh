#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_camera_timeshift_cv_manifest.csv
CONSOLE=/home/vio/jtzero_camera_timeshift_cv_console.txt

cd "$ROOT"

need_file() {
  if [[ ! -s "$1" ]]; then
    echo "MISSING_OR_EMPTY: $1" >&2
    exit 2
  fi
}

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
dataset,shift_ms,tag,backend,legs
EOF
: > "$CONSOLE"

make_shifted_camera_csv() {
  local src="$1" dst="$2" shift_ms="$3"
  python3 - "$src" "$dst" "$shift_ms" <<'PY'
import csv, sys
src, dst, shift_s = sys.argv[1], sys.argv[2], sys.argv[3]
shift_ns = int(round(float(shift_s) * 1_000_000.0))
with open(src, newline="") as fi, open(dst, "w", newline="") as fo:
    r = csv.DictReader(fi)
    w = csv.DictWriter(fo, fieldnames=r.fieldnames)
    w.writeheader()
    for row in r:
        row["corrected_timestamp_ns"] = str(int(row["corrected_timestamp_ns"]) + shift_ns)
        w.writerow(row)
PY
}

run_one() {
  local ds="$1" shift="$2"
  local imu cam mjpg backend legs shifted tag slug
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

  slug=$(python3 - "$shift" <<'PY'
import sys
x=float(sys.argv[1])
if abs(x) < 1e-9:
    print("p0")
elif x > 0:
    print("p" + str(x).replace(".","d"))
else:
    print("m" + str(abs(x)).replace(".","d"))
PY
)
  shifted="/tmp/jtzero_${ds}_camera_shift_${slug}.csv"
  tag="TS_${ds}_${slug}"
  make_shifted_camera_csv "$cam" "$shifted" "$shift"
  echo "$ds,$shift,$tag,$backend,$legs" >> "$MANIFEST"

  echo "================ $tag additional_shift=${shift}ms ================" | tee -a "$CONSOLE"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" "$tag" 0 0 \
      "$imu" "$shifted" "$mjpg" "$backend" "$legs" \
      2>&1 | tee -a "$CONSOLE" | \
      grep -E '(^\[TBS\]|^LEG [1-6] |LEG_Z_RMS_MM|final \|dP\||REPLAY RESULT)'
}

# The recorded camera CSV already contains a fixed +10.5 ms correction.
# Values below are ADDITIONAL shifts relative to that existing correction.
SHIFTS=(-60 -45 -30 -20 -10 0 10 20 30 45 60)

for ds in V11 V12; do
  for shift in "${SHIFTS[@]}"; do
    run_one "$ds" "$shift"
  done
done

python3 tools/analyze_camera_timeshift_cross_validation_v12.py

echo
echo "Saved:"
echo "  $MANIFEST"
echo "  $CONSOLE"
echo "  /home/vio/jtzero_camera_timeshift_cv_detail.csv"
echo "  /home/vio/jtzero_camera_timeshift_cv_summary.csv"
echo "  /home/vio/jtzero_camera_timeshift_cv_top.txt"
