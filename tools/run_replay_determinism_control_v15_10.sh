#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"
BIN=/tmp/replay_determinism_v15_10

COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
PARAMS="${4:-$ROOT/params/JTZeroMonoFLU}"

A_LOG=/home/vio/jtzero_determinism_v15_10_A.txt
B_LOG=/home/vio/jtzero_determinism_v15_10_B.txt
A_CSV=/home/vio/jtzero_determinism_v15_10_A.csv
B_CSV=/home/vio/jtzero_determinism_v15_10_B.csv

cat <<EOF
============================================================
JT-ZERO REPLAY DETERMINISM CONTROL v15.10
============================================================
A and B are two independent executions of the SAME binary.
Source:   $SRC
Params:   $PARAMS
Combined: $COMBINED
Camera:   $CAMINDEX
MJPEG:    $MJPEG
Mode:     CURRENT
EOF

COMMON=(
  -std=c++17 -O2
  -I"$ROOT/tools"
  -I/home/vio/Kimera-VIO/include
  -I/home/vio/Kimera-VIO/build
  -I/home/vio/Kimera-VIO/third_party/mavlink
  -I/usr/include/eigen3
)
LIBS=(
  -L/home/vio/Kimera-VIO/build
  -L/usr/local/lib
  -lkimera_vio -lgtsam -lglog -lgflags -lpthread
)

# shellcheck disable=SC2046
g++ "${COMMON[@]}" "$SRC" -o "$BIN" $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"

run_once() {
  local tag="$1"
  local log="$2"
  local outcsv="$3"
  echo
  echo "================ RUN $tag: CURRENT =============================="
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$log"
  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$outcsv"
}

run_once A "$A_LOG" "$A_CSV"
run_once B "$B_LOG" "$B_CSV"

echo
echo "================ EXACT FILE CHECK ================================"nA_SHA="$(sha256sum "$A_CSV" | awk '{print $1}')"
B_SHA="$(sha256sum "$B_CSV" | awk '{print $1}')"
echo "A sha256: $A_SHA"
echo "B sha256: $B_SHA"
if cmp -s "$A_CSV" "$B_CSV"; then
  echo "exact CSV identity: YES"
else
  echo "exact CSV identity: NO"
fi

echo
echo "================ PAIRED STATE COMPARISON ========================="
awk -F, '
function abs(x){return x<0?-x:x}
function wrap(x){while(x>180)x-=360; while(x<-180)x+=360; return x}
NR==FNR {
  if (FNR==1) next
  k=$1
  ta[k]=$2+0
  ax[k]=$3+0; ay[k]=$4+0; az[k]=$5+0
  ar[k]=$9+0; ap[k]=$10+0; ayaw[k]=$11+0
  countA++
  next
}
FNR==1 { next }
{
  k=$1
  countB++
  if (!(k in ax)) { missingA++; next }
  if (($2+0) != ta[k]) timestampMismatch++
  dx=($3+0)-ax[k]; dy=($4+0)-ay[k]; dz=($5+0)-az[k]
  dp=sqrt(dx*dx+dy*dy+dz*dz)*1000.0
  dr=wrap(($9+0)-ar[k]); dpit=wrap(($10+0)-ap[k]); dyaw=wrap(($11+0)-ayaw[k])
  drot=sqrt(dr*dr+dpit*dpit+dyaw*dyaw)
  if (dp>maxDP) {maxDP=dp; maxDPkf=k; maxDPt=($2+0)}
  if (drot>maxDR) {maxDR=drot; maxDRkf=k; maxDRt=($2+0)}
  if (!seen01 && dp>=0.1) {seen01=1; k01=k; d01=dp}
  if (!seen1 && dp>=1.0) {seen1=1; k1=k; d1=dp}
  if (!seen10 && dp>=10.0) {seen10=1; k10=k; d10=dp}
  paired++
}
END {
  printf("states A: %d\n", countA)
  printf("states B: %d\n", countB)
  printf("paired: %d\n", paired)
  printf("timestamp mismatches: %d\n", timestampMismatch+0)
  printf("missing A states for B rows: %d\n", missingA+0)
  printf("max position separation: %.9f mm at kf=%d timestamp_ns=%.0f\n", maxDP+0, maxDPkf+0, maxDPt+0)
  printf("max Euler-vector separation: %.9f deg at kf=%d timestamp_ns=%.0f\n", maxDR+0, maxDRkf+0, maxDRt+0)
  if (seen01) printf("first >=0.1 mm: kf=%d dP=%.9f mm\n", k01, d01); else print "first >=0.1 mm: NONE"
  if (seen1) printf("first >=1 mm: kf=%d dP=%.9f mm\n", k1, d1); else print "first >=1 mm: NONE"
  if (seen10) printf("first >=10 mm: kf=%d dP=%.9f mm\n", k10, d10); else print "first >=10 mm: NONE"
}
' "$A_CSV" "$B_CSV"

echo
echo "============================================================"
echo "DETERMINISM CONTROL v15.10 COMPLETE"
echo "============================================================"
echo "A log: $A_LOG"
echo "B log: $B_LOG"
echo "A CSV: $A_CSV"
echo "B CSV: $B_CSV"
echo
echo "Interpretation:"
echo "  If A/B are identical or differ only at numerical-noise scale, replay is deterministic."
echo "  If A/B diverge by mm/cm during yaw, previous gravity A/B results are confounded by replay/Kimera nondeterminism."
