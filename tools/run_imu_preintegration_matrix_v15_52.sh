#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/jtzero-kimera-sync"
KIMERA=/home/vio/Kimera-VIO
SRC="$KIMERA/src/backend/RegularVioBackend.cpp"
BAK=/tmp/RegularVioBackend.cpp.v15_52.bak
PARAMS="$ROOT/params/JTZeroMonoFLU"
BASE=/home/vio/jtzero_yaw_only_v15_42.csv
CAM=/home/vio/jtzero_yaw_only_v15_42_camera.csv
MJPG=/home/vio/jtzero_yaw_only_v15_42.mjpg
TMP=/tmp/jtzero_v15_52
OUT=/home/vio/jtzero_imu_preintegration_v15_52
BIN="$TMP/replay"
mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/*.csv "$OUT/report.tsv" "$OUT/summary.txt"
cp "$SRC" "$BAK"

restore(){
  cp "$BAK" "$SRC"
  echo '[RESTORE] Restored clean RegularVioBackend.cpp'
  cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_52_restore.log 2>&1 || true
}
trap restore EXIT

# Hard-disable visual measurements in the actual active regular backend path.
python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); s=p.read_text()
needle='''  const StereoMeasurements& smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n'''
repl='''  StereoMeasurements smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n  static size_t jt52_in = 0, jt52_out = 0, jt52_lines = 0;\n  jt52_in += smart_stereo_measurements_kf.size();\n  smart_stereo_measurements_kf.clear();\n  jt52_out += smart_stereo_measurements_kf.size();\n  ++jt52_lines;\n  std::cerr << "JTZERO_V15_52_VIS_PROOF kf=" << curr_kf_id_\n            << " in_total=" << jt52_in << " out_total=" << jt52_out\n            << " lines=" << jt52_lines << "\\n";\n'''
if needle not in s:
    raise SystemExit('v15.52 measurement anchor missing')
s=s.replace(needle,repl,1)
p.write_text(s)
PY

echo '============================================================'
echo 'JT-ZERO v15.52 IMU / PREINTEGRATION CAUSAL MATRIX'
echo 'Visual measurements are hard-disabled in active RegularVioBackend.'
echo 'One Kimera build, many deterministic IMU variants.'
echo '============================================================'

echo '[BUILD] diagnostic Kimera'
cmake --build "$KIMERA/build" -j"$(nproc)"

echo '[BUILD] replay'
g++ -std=c++17 -O2 \
  -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" \
  -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)

# Generate deterministic IMU variants while preserving all timestamps and camera data.
python3 - "$BASE" "$OUT" <<'PY'
import csv, math, os, statistics, sys
base,out=sys.argv[1:]
rows=[]
with open(base,newline='') as f:
    r=csv.reader(f); hdr=next(r); rows=list(r)
imu_idx=[i for i,x in enumerate(rows) if len(x)>=14 and x[0]=='IMU']
if not imu_idx: raise SystemExit('No IMU rows')
# first 8 s of the initial still period
first_ns=int(rows[imu_idx[0]][3])
static=[i for i in imu_idx if int(rows[i][3])-first_ns <= 8_000_000_000]
if len(static)<100: raise SystemExit('Too few initial-static samples')
def mean_col(c): return statistics.fmean(float(rows[i][c]) for i in static)
g0=[mean_col(8),mean_col(9),mean_col(10)]
w0=[mean_col(11),mean_col(12),mean_col(13)]
gnorm=math.sqrt(sum(x*x for x in g0))
unit=[x/gnorm for x in g0]
ideal=[9.81*x for x in unit]
res=[g0[k]-ideal[k] for k in range(3)]

print('V15_52_STATIC_SAMPLES',len(static))
print('V15_52_INITIAL_ACCEL_MEAN',*['%.9f'%x for x in g0])
print('V15_52_INITIAL_ACCEL_NORM %.9f'%gnorm)
print('V15_52_INITIAL_GYRO_MEAN',*['%.9f'%x for x in w0])
print('V15_52_ACCEL_NORM_RESIDUAL',*['%.9f'%x for x in res])

variants={
 'CONTROL':('dyn',1.0),
 'ACC_DYN_0':('dyn',0.0),
 'ACC_DYN_025':('dyn',0.25),
 'ACC_DYN_050':('dyn',0.50),
 'ACC_DYN_075':('dyn',0.75),
 'ACC_DYN_125':('dyn',1.25),
 'FREEZE_AX':('freeze_axis',0),
 'FREEZE_AY':('freeze_axis',1),
 'FREEZE_AZ':('freeze_axis',2),
 'REMOVE_STATIC_NORM_RESIDUAL':('remove_res',0),
 'GYRO_BIAS_REMOVE':('gyro_bias',0),
 'GYRO_SCALE_098':('gyro_scale',0.98),
 'GYRO_SCALE_102':('gyro_scale',1.02),
}
for name,(kind,arg) in variants.items():
    rr=[x[:] for x in rows]
    for i in imu_idx:
        a=[float(rr[i][8]),float(rr[i][9]),float(rr[i][10])]
        w=[float(rr[i][11]),float(rr[i][12]),float(rr[i][13])]
        if kind=='dyn':
            a=[g0[k]+arg*(a[k]-g0[k]) for k in range(3)]
        elif kind=='freeze_axis':
            a[arg]=g0[arg]
        elif kind=='remove_res':
            a=[a[k]-res[k] for k in range(3)]
        elif kind=='gyro_bias':
            w=[w[k]-w0[k] for k in range(3)]
        elif kind=='gyro_scale':
            w=[arg*x for x in w]
        for k in range(3): rr[i][8+k]='%.12f'%a[k]
        for k in range(3): rr[i][11+k]='%.12f'%w[k]
    p=os.path.join(out,name+'.csv')
    with open(p,'w',newline='') as f:
        q=csv.writer(f); q.writerow(hdr); q.writerows(rr)

# Pure data diagnostic: fit lateral accel residual to rotational terms alpha x r + omega x (omega x r).
# Uses CURRENT raw FRD values only as a comparative signature; no claim of exact lever arm/frame.
t=[]; A=[]; acc=[]
prev=None
for i in imu_idx:
    ts=int(rows[i][3])*1e-9
    a=[float(rows[i][8]),float(rows[i][9]),float(rows[i][10])]
    w=[float(rows[i][11]),float(rows[i][12]),float(rows[i][13])]
    if prev is not None:
        dt=ts-prev[0]
        if 0<dt<0.03:
            al=[(w[k]-prev[1][k])/dt for k in range(3)]
            # matrix M such that M*r = alpha x r + omega x (omega x r)
            wx,wy,wz=w; ax,ay,az=al
            W2=[[wx*wx-(wx*wx+wy*wy+wz*wz), wx*wy, wx*wz],
                [wy*wx, wy*wy-(wx*wx+wy*wy+wz*wz), wy*wz],
                [wz*wx, wz*wy, wz*wz-(wx*wx+wy*wy+wz*wz)]]
            X=[[0,-az,ay],[az,0,-ax],[-ay,ax,0]]
            M=[[X[r][c]+W2[r][c] for c in range(3)] for r in range(3)]
            ar=[a[k]-g0[k] for k in range(3)]
            A.extend(M); acc.extend(ar)
    prev=(ts,w)
try:
    import numpy as np
    AA=np.asarray(A,float); bb=np.asarray(acc,float)
    rhat,*_=np.linalg.lstsq(AA,bb,rcond=None)
    pred=AA@rhat
    ss=np.sum((bb-bb.mean())**2)
    r2=1-np.sum((bb-pred)**2)/ss if ss>0 else float('nan')
    print('V15_52_ROT_ACCEL_EFFECTIVE_R_M',' '.join('%.6f'%x for x in rhat))
    print('V15_52_ROT_ACCEL_FIT_R2 %.6f'%r2)
except Exception as e:
    print('V15_52_ROT_ACCEL_FIT_SKIPPED',repr(e))
PY

printf 'MODE\tREPLAY_MODE\tRC\tSTATES\tDP_MM\tMAXEXC_MM\tDR\tDPITCH\tDYAW\tVIS_LINES\tVIS_OUT_FINAL\n' > "$OUT/report.tsv"
run(){
  local tag="$1"; local replay_mode="$2"; local csv="$3"; local log="$OUT/$tag.log"
  local rc st dp mx dr dpt dy vl vo
  echo "[RUN] $tag replay=$replay_mode"
  set +e
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$PARAMS" "$replay_mode" "$csv" "$CAM" "$MJPG" >"$log" 2>&1
  rc=$?
  set -e
  st=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  read -r dr dpt dy < <(awk '/final dRPY deg:/ {gsub(/[\[\]]/,"",$4); gsub(/[\[\]]/,"",$5); gsub(/[\[\]]/,"",$6); a=$4;b=$5;c=$6} END{print a+0,b+0,c+0}' "$log")
  vl=$(grep -c JTZERO_V15_52_VIS_PROOF "$log" || true)
  vo=$(grep JTZERO_V15_52_VIS_PROOF "$log" | tail -1 | sed -n 's/.*out_total=\([0-9]*\).*/\1/p'); vo=${vo:-NA}
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$tag" "$replay_mode" "$rc" "$st" "${dp:-NA}" "${mx:-NA}" "${dr:-NA}" "${dpt:-NA}" "${dy:-NA}" "$vl" "$vo" >> "$OUT/report.tsv"
  echo " rc=$rc states=$st dP=${dp:-NA} vis_lines=$vl vis_out=$vo"
}

for m in CONTROL ACC_DYN_0 ACC_DYN_025 ACC_DYN_050 ACC_DYN_075 ACC_DYN_125 FREEZE_AX FREEZE_AY FREEZE_AZ REMOVE_STATIC_NORM_RESIDUAL GYRO_BIAS_REMOVE GYRO_SCALE_098 GYRO_SCALE_102; do
  run "$m" CURRENT "$OUT/$m.csv"
done
run NO_ZXY NO_ZXY "$OUT/CONTROL.csv"

echo
echo '================ v15.52 MATRIX ================'
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

python3 - "$OUT/report.tsv" <<'PY' | tee "$OUT/summary.txt"
import csv,sys,math
r=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))
valid=[]
for x in r:
    try:
        if int(x['RC'])==0 and int(x['STATES'])>=140 and x['DP_MM']!='NA': valid.append(x)
    except: pass
c=next((x for x in valid if x['MODE']=='CONTROL'),None)
if not c:
    print('V15_52_VERDICT=INVALID_CONTROL'); print('RESULT: COMPLETE'); raise SystemExit
cd=float(c['DP_MM'])
print(f'CONTROL_DP_MM={cd:.3f} CONTROL_STATES={c["STATES"]}')
proof=all(int(x['VIS_LINES'])>=140 and x['VIS_OUT_FINAL']=='0' for x in valid)
print('HARD_NO_VIS_PROOF=' + ('PASS' if proof else 'FAIL'))
for x in sorted((x for x in valid if x['MODE']!='CONTROL'), key=lambda z: float(z['DP_MM'])):
    d=float(x['DP_MM'])
    print(f'{x["MODE"]}_DP_MM={d:.3f} RATIO={d/cd:.4f} DRPY=[{x["DR"]},{x["DPITCH"]},{x["DYAW"]}]')

def d(name):
    x=next((q for q in valid if q['MODE']==name),None); return float(x['DP_MM']) if x else math.nan
ad0=d('ACC_DYN_0'); ax=d('FREEZE_AX'); ay=d('FREEZE_AY'); az=d('FREEZE_AZ'); gb=d('GYRO_BIAS_REMOVE')
if not proof:
    verdict='INVALID_VISUAL_NOT_HARD_DISABLED'
elif math.isfinite(ad0) and ad0 < 0.5*cd:
    verdict='DYNAMIC_ACCEL_IS_DOMINANT_TRANSLATION_DRIVER'
elif min(ax,ay,az) < 0.7*cd:
    verdict='ACCEL_AXIS_LOCALIZED_TRANSLATION_DRIVER'
elif math.isfinite(gb) and gb < 0.7*cd:
    verdict='GYRO_BIAS_ORIENTATION_PROPAGATION_DOMINANT'
else:
    verdict='ERROR_PERSISTS_AFTER_INPUT_ABLATIONS_BACKEND_PREINTEGRATION_NEXT'
print('V15_52_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo "Logs: $OUT"
echo 'Source will now be restored and Kimera rebuilt clean by trap.'
