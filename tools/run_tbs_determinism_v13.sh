#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
BIN=/tmp/replay_mono_imu_tbs_sweep_v12
PARAMS="${ROOT}/params/JTZeroMonoFLU"
MAVLINK=/home/vio/Kimera-VIO/third_party/mavlink
OUT=/home/vio/jtzero_tbs_determinism_v13.txt

cd "$ROOT"

g++ -std=c++17 -O2   tools/replay_mono_imu_tbs_sweep_v12.cpp   -I. -Itools -I"$MAVLINK"   -I/home/vio/Kimera-VIO/include   -I/home/vio/Kimera-VIO/build   -I/usr/local/include   -I/usr/include/eigen3   $(pkg-config --cflags opencv4)   -L/home/vio/Kimera-VIO/build   -L/usr/local/lib   -Wl,-rpath,/home/vio/Kimera-VIO/build   -Wl,-rpath,/usr/local/lib   -lkimera_vio -lgtsam -lgflags -lglog -lpthread   $(pkg-config --libs opencv4)   -o "$BIN"

need() { [[ -s "$1" ]] || { echo "MISSING_OR_EMPTY: $1" >&2; exit 2; }; }
for f in   /home/vio/jtzero_500mm_v13.csv   /home/vio/jtzero_500mm_v13_camera.csv   /home/vio/jtzero_500mm_v13.mjpg   /home/vio/jtzero_500mm_v13_backend.csv   /home/vio/jtzero_500mm_v13_legs.csv; do need "$f"; done

: > "$OUT"

run_case() {
  local cand="$1" roll="$2" pitch="$3"
  echo "================ $cand roll=$roll pitch=$pitch ================" | tee -a "$OUT"
  local prev=""
  for n in 1 2 3; do
    local tag="DET_${cand}_R${n}"
    LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}       "$BIN" "$PARAMS" "$tag" "$roll" "$pitch"         /home/vio/jtzero_500mm_v13.csv         /home/vio/jtzero_500mm_v13_camera.csv         /home/vio/jtzero_500mm_v13.mjpg         /home/vio/jtzero_500mm_v13_backend.csv         /home/vio/jtzero_500mm_v13_legs.csv         >/home/vio/jtzero_tbs_det_${cand}_${n}.log 2>&1

    local csv="/home/vio/jtzero_extrinsics_replay_v10_${tag}.csv"
    need "$csv"
    local sha
    sha=$(sha256sum "$csv" | awk '{print $1}')
    echo "run=$n sha256=$sha" | tee -a "$OUT"

    if [[ -n "$prev" ]]; then
      if cmp -s "$prev" "$csv"; then
        echo "compare_prev=IDENTICAL" | tee -a "$OUT"
      else
        echo "compare_prev=DIFFERENT" | tee -a "$OUT"
        python3 - "$prev" "$csv" <<'PY' | tee -a "$OUT"
import csv, math, sys
a,b=sys.argv[1:3]
def load(p):
    d={}
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            d[int(r["keyframe"])] = r
    return d
A,B=load(a),load(b)
ks=sorted(set(A)&set(B))
max_dp=(0.0,None); max_rot=(0.0,None)
def wrap(x):
    while x>180: x-=360
    while x<-180: x+=360
    return x
for k in ks:
    x=A[k]; y=B[k]
    dp=1000*math.sqrt(sum((float(y[q])-float(x[q]))**2 for q in ("px_m","py_m","pz_m")))
    dr=math.sqrt(sum(wrap(float(y[q])-float(x[q]))**2 for q in ("roll_deg","pitch_deg","yaw_deg")))
    if dp>max_dp[0]: max_dp=(dp,k)
    if dr>max_rot[0]: max_rot=(dr,k)
print(f"paired={len(ks)} max_dP_mm={max_dp[0]:.9f} at_kf={max_dp[1]}")
print(f"max_dRPY_deg={max_rot[0]:.9f} at_kf={max_rot[1]}")
PY
      fi
    fi
    prev="$csv"
  done
}

run_case Rm1_Pm5 -1 -5
run_case Rm2_Pm4 -2 -4
run_case Rm2_Pm5 -2 -5

echo
echo "Saved: $OUT"
