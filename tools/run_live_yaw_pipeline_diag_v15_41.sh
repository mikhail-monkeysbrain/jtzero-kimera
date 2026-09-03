#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$ROOT/tools/live_mono_imu_yaw_pipeline_diag_v15_41.cpp"
BIN=/tmp/live_mono_imu_yaw_pipeline_diag_v15_41
P=/tmp/JTZeroMonoFLU_v15_41_H30
RAW=/home/vio/jtzero_live_yaw_only_hud_v15_3.csv
OUT=/home/vio/jtzero_live_yaw_pipeline_diag_v15_41.csv
rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 30/' "$P/BackendParams.yaml"

echo '============================================================'
echo 'JT-ZERO v15.41 LIVE PIPELINE DIAGNOSTIC — H30'
echo '10 s ПОКОЙ -> 15 s YAW ~90° -> 10 s ПОКОЙ'
echo 'Смотрите отдельно FC R/P и VIO R/P.'
echo '============================================================'

echo '[1/3] Build'
cmake --build "$K/build" -j2 --target kimera_vio
g++ -std=c++17 -O2 "$SRC" -o "$BIN" \
 -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
 $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib \
 -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4) -lopencv_freetype

echo '[2/3] Physical GUI run'
rm -f "$RAW" "$OUT"
set +e
LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P"
rc=$?
set -e
[[ $rc -eq 0 ]] || { echo "RESULT: LIVE_RUN_FAILED rc=$rc"; exit "$rc"; }
[[ -s "$RAW" ]] || { echo 'RESULT: INVALID_MISSING_CSV'; exit 4; }
cp "$RAW" "$OUT"

echo '[3/3] Strict backend phase-coverage gate'
awk -F, '
NR==1 {next}
{
 n++;
 phase[$1]++;
 k=$2+0;
 if(n==1){first=k} last=k;
}
END {
 printf("states=%d keyframes=%d..%d\n",n,first,last);
 printf("INIT=%d YAW=%d SETTLE=%d\n",phase["INIT"]+0,phase["YAW"]+0,phase["SETTLE"]+0);
 if(n<3 || phase["INIT"]<1 || phase["YAW"]<1 || phase["SETTLE"]<1){
   print "BACKEND_PHASE_COVERAGE=FAIL";
   print "RESULT: INVALID_BACKEND_DID_NOT_COVER_FULL_TEST";
   exit 20;
 }
 print "BACKEND_PHASE_COVERAGE=PASS";
 print "RESULT: COMPLETE";
}' "$OUT"

echo "Saved CSV: $OUT"
