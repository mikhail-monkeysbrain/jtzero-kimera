#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
KIMERA="/home/vio/Kimera-VIO"
PARAMS="${ROOT}/params/JTZeroMonoFLU"
IMU="/home/vio/jtzero_yaw_only_v15_42.csv"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
TMP="/tmp/jtzero_v15_44"
SRC="${TMP}/replay_mono_imu_zxy_scale_v15_44_generated.cpp"
BIN="${TMP}/replay_v15_44"
OUT="/home/vio/jtzero_zxy_scale_v15_44"
SCALES=(0.00 0.25 0.50 0.75 1.00 1.25)

cd "$ROOT"
for f in "$IMU" "$CAM" "$MJPG" "$ATT" tools/replay_mono_imu_zxy_ab_v11.cpp; do
  [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }
done
mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/*.csv "$OUT"/report.tsv

echo "============================================================"
echo "JT-ZERO v15.44 ZXY SCALE + FC REFERENCE"
echo "Dataset: v15.42"
echo "Production sources/params are NOT modified"
echo "Only ZXY scale changes: 0, .25, .50, .75, 1, 1.25"
echo "============================================================"

# Generate a diagnostic-only copy of the proven v11 replay. CURRENT still selects
# the ZXY path; JTZERO_ZXY_SCALE changes only its magnitude. Gravity feedback,
# camera bytes/timestamps, extrinsics and Kimera params remain identical.
cp tools/replay_mono_imu_zxy_ab_v11.cpp "$SRC"
perl -0777 -i -pe 's/explicit ReplayImuCorrection11\(bool use_zxy\):use_zxy_\(use_zxy\)\{\}/explicit ReplayImuCorrection11(bool use_zxy){ const char* e=std::getenv("JTZERO_ZXY_SCALE"); zxy_scale_=e?std::stod(e):(use_zxy?1.0:0.0); }/g' "$SRC"
perl -0777 -i -pe 's/const Eigen::Vector3d gyro_in=use_zxy_\?jtzero::ImuCorrection::applyZxy\(gyro_flu\):gyro_flu;/Eigen::Vector3d gyro_in=gyro_flu; gyro_in.x() += zxy_scale_*jtzero::ImuCorrection::kGyroCx*gyro_flu.z(); gyro_in.y() += zxy_scale_*jtzero::ImuCorrection::kGyroCy*gyro_flu.z();/g' "$SRC"
perl -0777 -i -pe 's/bool use_zxy_=true;/double zxy_scale_=1.0;/g' "$SRC"

grep -q 'zxy_scale_\*jtzero::ImuCorrection::kGyroCy' "$SRC" || { echo "[FATAL] diagnostic transform failed"; exit 1; }

# Clean Kimera library only; no source instrumentation.
cmake --build "$KIMERA/build" -j"$(nproc)"

g++ -std=c++17 -O2 \
  -I"$ROOT/tools" \
  -I"$KIMERA/include" -I"$KIMERA/build" \
  -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  "$SRC" -o "$BIN" \
  -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread \
  $(pkg-config --libs opencv4)

printf 'SCALE\tRC\tSTATES\tFINAL_DP_MM\tMAXEXC_MM\tVIO_DROLL\tVIO_DPITCH\tVIO_DYAW\tEXTRA_X\tEXTRA_Y\tEXTRA_Z\n' > "$OUT/report.tsv"

for scale in "${SCALES[@]}"; do
  tag=${scale/./p}
  log="$OUT/scale_${tag}.log"
  echo "[RUN] ZXY scale=$scale"
  set +e
  JTZERO_ZXY_SCALE="$scale" \
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  rc=$?
  set -e

  states=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  rpy=$(sed -n 's/.*final dRPY deg: \[\([^]]*\)\].*/\1/p' "$log" | tail -1)
  extra=$(sed -n 's/.*integral FED-RAW deg XYZ: \[\([^]]*\)\].*/\1/p' "$log" | tail -1)
  read -r vr vp vy <<<"${rpy:-NA NA NA}"
  read -r ex ey ez <<<"${extra:-NA NA NA}"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$scale" "$rc" "$states" "${dp:-NA}" "${mx:-NA}" "$vr" "$vp" "$vy" "$ex" "$ey" "$ez" >> "$OUT/report.tsv"
  echo "      rc=$rc states=$states dP=${dp:-NA} mm maxexc=${mx:-NA} mm dRPY=[$vr $vp $vy] extra=[$ex $ey $ez]"
done

echo
echo "================ RAW SWEEP ================"
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

echo
echo "================ FC REFERENCE ================"n
python3 - "$ATT" "$OUT/report.tsv" <<'PY'
import csv, math, statistics, sys
att, report = sys.argv[1], sys.argv[2]
with open(att,newline='') as f: a=list(csv.DictReader(f))
T=[int(r['mapped_rpi_ns']) for r in a]; t0,t1=T[0],T[-1]
def medcol(name,lo,hi):
    q=[float(r[name]) for r in a if lo<=int(r['mapped_rpi_ns'])<=hi]
    return statistics.median(q)
def Rxyz(r,p,y):
    r,p,y=map(math.radians,(r,p,y)); cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def tr(A): return sum(A[i][i] for i in range(3))
def Tm(A): return [list(x) for x in zip(*A)]
def angle(A): return math.degrees(math.acos(max(-1,min(1,(tr(A)-1)/2))))
# 2-second medians, matching prior v15.43c reference.
s0=[medcol(c,t0,t0+2_000_000_000) for c in ('roll_deg','pitch_deg','yaw_deg')]
s1=[medcol(c,t1-2_000_000_000,t1) for c in ('roll_deg','pitch_deg','yaw_deg')]
R0,R1=Rxyz(*s0),Rxyz(*s1); Rfc=mm(Tm(R0),R1); fcang=angle(Rfc)
print('FC START absolute RPY deg: %.4f %.4f %.4f'%tuple(s0))
print('FC END   absolute RPY deg: %.4f %.4f %.4f'%tuple(s1))
print('FC relative rotation geodesic: %.4f deg'%fcang)
print()
print('NOTE: VIO dRPY below is diagnostic output; do not subtract Euler components from FC.')
print('The causal decision in v15.44 is based first on translation monotonicity/branching and FED-RAW injection.')
rows=list(csv.DictReader(open(report),delimiter='\t'))
valid=[r for r in rows if r['RC']=='0' and r['FINAL_DP_MM'] not in ('','NA')]
for r in valid:
    print('scale=%s  dP=%s mm  maxexc=%s mm  VIO dRPY=[%s %s %s]  extraY=%s deg'%(r['SCALE'],r['FINAL_DP_MM'],r['MAXEXC_MM'],r['VIO_DROLL'],r['VIO_DPITCH'],r['VIO_DYAW'],r['EXTRA_Y']))
vals=[(float(r['SCALE']),float(r['FINAL_DP_MM'])) for r in valid]
if len(vals)<4:
    verdict='INSUFFICIENT_VALID_REPLAYS'
else:
    ds=[v for _,v in vals]
    monotonic=all(ds[i]<=ds[i+1] for i in range(len(ds)-1)) or all(ds[i]>=ds[i+1] for i in range(len(ds)-1))
    best=min(vals,key=lambda x:x[1]); current=min(vals,key=lambda x:abs(x[0]-1.0))
    if monotonic and best[0] <= 0.25 and best[1] < 0.5*current[1]: verdict='ZXY_SCALE_IS_STRONG_CAUSAL_CANDIDATE'
    elif not monotonic: verdict='ZXY_ACTS_AS_BRANCH_TRIGGER_IN_ILL_CONDITIONED_BACKEND'
    else: verdict='ZXY_EFFECT_SIGNIFICANT_BUT_NOT_YET_CAUSAL'
    print('BEST_SCALE=%.2f BEST_DP_MM=%.3f CURRENT_DP_MM=%.3f'%(best[0],best[1],current[1]))
print('V15_44_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo
echo "Logs: $OUT"
echo "Production files were not modified."
