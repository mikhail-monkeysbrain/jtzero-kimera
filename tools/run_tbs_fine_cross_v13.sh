#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
MANIFEST=/home/vio/jtzero_tbs_fine_cross_v13_manifest.csv
CONSOLE=/home/vio/jtzero_tbs_fine_cross_v13_console.txt
cd "$ROOT"

need(){ [[ -s "$1" ]] || { echo "MISSING_OR_EMPTY: $1" >&2; exit 2; }; }
for ds in v11 v12 v13; do
  for suffix in .csv _camera.csv .mjpg _backend.csv _legs.csv; do
    need "/home/vio/jtzero_500mm_${ds}${suffix}"
  done
done

g++ -std=c++17 -O2 tools/replay_mono_imu_tbs_sweep_v12.cpp  -I. -Itools -I"$MAVLINK" -I/home/vio/Kimera-VIO/include  -I/home/vio/Kimera-VIO/build -I/usr/local/include -I/usr/include/eigen3  $(pkg-config --cflags opencv4) -L/home/vio/Kimera-VIO/build -L/usr/local/lib  -Wl,-rpath,/home/vio/Kimera-VIO/build -Wl,-rpath,/usr/local/lib  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4) -o "$BIN"

echo 'dataset,candidate,tag,roll_deg,pitch_deg,backend,legs' > "$MANIFEST"
: > "$CONSOLE"

tagnum(){ local x="$1"; x="${x/-/m}"; x="${x/./d}"; printf '%s' "$x"; }
run_one(){
 local ds="$1" r="$2" p="$3" low base cand tag
 low=$(echo "$ds"|tr '[:upper:]' '[:lower:]')
 base="/home/vio/jtzero_500mm_${low}"
 cand="R$(tagnum "$r")_P$(tagnum "$p")"
 tag="FCV13_${ds}_${cand}"
 echo "$ds,$cand,$tag,$r,$p,${base}_backend.csv,${base}_legs.csv" >> "$MANIFEST"
 echo "================ $tag roll=$r pitch=$p ================" | tee -a "$CONSOLE"
 LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}  "$BIN" "$PARAMS" "$tag" "$r" "$p" "${base}.csv" "${base}_camera.csv" "${base}.mjpg"  "${base}_backend.csv" "${base}_legs.csv" 2>&1 | tee -a "$CONSOLE" |  grep -E '(^\[TBS\]|^LEG [1-6] |LEG_Z_RMS_MM|LEG_XY_ERR_RMS_MM|LEG_XY_MEAN_MM|final \|dP\||REPLAY RESULT)'
}

for ds in V11 V12 V13; do
 for r in -0.5 -1.0 -1.5; do
  for p in -4.5 -5.0 -5.5; do run_one "$ds" "$r" "$p"; done
 done
done

python3 tools/analyze_tbs_fine_cross_v13.py | tee /home/vio/jtzero_tbs_fine_cross_v13_top.txt
echo "Saved: $MANIFEST"
echo "       /home/vio/jtzero_tbs_fine_cross_v13_summary.csv"
echo "       /home/vio/jtzero_tbs_fine_cross_v13_detail.csv"
echo "       /home/vio/jtzero_tbs_fine_cross_v13_top.txt"
