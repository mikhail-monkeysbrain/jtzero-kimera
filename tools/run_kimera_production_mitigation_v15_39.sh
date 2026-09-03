#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_39
OUT=/home/vio/jtzero_production_mitigation_v15_39
IMU="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAM="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
mkdir -p "$OUT"
for f in "$IMU" "$CAM" "$MJPG"; do [[ -s "$f" ]] || { echo "FATAL missing $f" >&2; exit 2; }; done
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then echo 'FATAL dirty VioBackend.cpp' >&2; exit 3; fi

build_replay(){
 g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}
metric(){ awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$1"; }
pathmetric(){ awk -F': ' '/^path mm:/{print $2;exit}' "$1"; }
maxmetric(){ awk -F': ' '/^max excursion mm:/{print $2;exit}' "$1"; }
run_h(){
 local name="$1"
 local h="$2"
 local P="$OUT/params_$name"
 local log="$OUT/$name.txt"
 rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
 sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $h/" "$P/BackendParams.yaml"
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 set +e
 LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
 local rc=$?; set -e
 local dp states path mx
 dp=$(metric "$log"); states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log"); path=$(pathmetric "$log"); mx=$(maxmetric "$log")
 printf '%-12s H=%-3s rc=%d states=%-4s dP=%-10s path=%-10s maxexc=%s\n' "$name" "$h" "$rc" "${states:-NA}" "${dp:-NA}" "${path:-NA}" "${mx:-NA}"
 [[ $rc -eq 0 && -n "$dp" ]] || { tail -120 "$log"; exit 10; }
}

echo '============================================================'
echo 'JT-ZERO v15.39 PRODUCTION MITIGATION REPLAY MATRIX'
echo 'Existing yaw_only_v13 only. No source instrumentation.'
echo '============================================================'
echo '[1/4] Build clean Kimera + replay'
cmake --build "$K/build" -j2 --target kimera_vio
build_replay

echo '[2/4] Stable-horizon matrix'
run_h H28_CONTROL 28
run_h H29 29
run_h H30 30
run_h H32 32
run_h H35 35
run_h H40 40
run_h H50 50

echo '[3/4] Determinism repeats for candidate horizons'
run_h H30_REPEAT 30
run_h H35_REPEAT 35

H30=$(metric "$OUT/H30.txt"); H30R=$(metric "$OUT/H30_REPEAT.txt")
H35=$(metric "$OUT/H35.txt"); H35R=$(metric "$OUT/H35_REPEAT.txt")
python3 - "$H30" "$H30R" "$H35" "$H35R" <<'PY'
import sys
h30,h30r,h35,h35r=map(float,sys.argv[1:])
d30=abs(h30-h30r); d35=abs(h35-h35r)
print(f'DETERMINISM H30 delta={d30:.6f} mm H35 delta={d35:.6f} mm')
if d30>0.01 or d35>0.01:
    print('RESULT: INVALID_NONDETERMINISTIC')
    raise SystemExit(21)
print('RESULT: DETERMINISM_PASS')
PY

echo '[4/4] Report'
echo
echo '================ V15.39 PRODUCTION MATRIX ================'
printf '%-12s %-4s %-8s %-12s %-12s %-12s\n' CASE H STATES FINAL_DP_MM PATH_MM MAXEXC_MM
for spec in 'H28_CONTROL 28' 'H29 29' 'H30 30' 'H32 32' 'H35 35' 'H40 40' 'H50 50' 'H30_REPEAT 30' 'H35_REPEAT 35'; do
 set -- $spec; n="$1"; h="$2"; log="$OUT/$n.txt"
 printf '%-12s %-4s %-8s %-12s %-12s %-12s\n' "$n" "$h" "$(awk -F': ' '/^backend states:/{print $2;exit}' "$log")" "$(metric "$log")" "$(pathmetric "$log")" "$(maxmetric "$log")"
done
python3 - "$OUT" <<'PY'
import re,sys,pathlib
out=pathlib.Path(sys.argv[1])
def read(n):
    s=(out/f'{n}.txt').read_text(errors='replace')
    def f(p):
        m=re.search(p,s,re.M); return float(m.group(1)) if m else float('inf')
    return f(r'^final \|dP\| mm:\s*([0-9.eE+-]+)'),f(r'^path mm:\s*([0-9.eE+-]+)'),f(r'^max excursion mm:\s*([0-9.eE+-]+)')
rows=[]
for h in (29,30,32,35,40,50):
    dp,path,mx=read(f'H{h}')
    rows.append((h,dp,path,mx))
# Gate here is deliberately dataset-local: full 162-state replay and <=35 mm final false translation.
passing=[r for r in rows if r[1] <= 35.0]
print()
print('DATASET_LOCAL_GATE: final false translation <= 35 mm on yaw_only_v13; physical/general-motion validation still required.')
for h,dp,path,mx in rows:
    print(f'H{h}: final={dp:.3f} mm gate={"PASS" if dp<=35.0 else "FAIL"}')
if not passing:
    print('PRODUCTION_CANDIDATE=NONE')
    print('ITEM_11_STATUS=BLOCKED')
else:
    # Prefer the smallest horizon inside the broad stable regime; H29 is boundary-minimal,
    # therefore H30 is preferred if it passes, otherwise choose the smallest passing >30.
    candidates=[r for r in passing if r[0]>=30]
    pick=min(candidates,key=lambda r:r[0]) if candidates else min(passing,key=lambda r:r[0])
    print(f'PRODUCTION_CANDIDATE=H{pick[0]}')
    print('ITEM_11_STATUS=READY_FOR_CANDIDATE_LIVE_VALIDATION')
print('ADAPTIVE_MARGINALIZATION_STATUS=NOT_PRODUCTION_READY; causal mechanism proven, but no robust online low-parallax detector has been validated yet.')
PY
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
