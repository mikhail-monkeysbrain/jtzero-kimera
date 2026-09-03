#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/jtzero-kimera-sync"
KIMERA=/home/vio/Kimera-VIO
REG="$KIMERA/src/backend/RegularVioBackend.cpp"
VIO="$KIMERA/src/backend/VioBackend.cpp"
REG_BAK=/tmp/RegularVioBackend.cpp.v15_53.bak
VIO_BAK=/tmp/VioBackend.cpp.v15_53.bak
PARAMS="$ROOT/params/JTZeroMonoFLU"
BASE=/home/vio/jtzero_yaw_only_v15_42.csv
CAM=/home/vio/jtzero_yaw_only_v15_42_camera.csv
MJPG=/home/vio/jtzero_yaw_only_v15_42.mjpg
TMP=/tmp/jtzero_v15_53
OUT=/home/vio/jtzero_backend_factor_v15_53
BIN="$TMP/replay"
mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/*.tsv "$OUT"/summary.txt
cp "$REG" "$REG_BAK"
cp "$VIO" "$VIO_BAK"

restore(){
  cp "$REG_BAK" "$REG"
  cp "$VIO_BAK" "$VIO"
  echo '[RESTORE] Restored clean Kimera backend sources'
  cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_53_restore.log 2>&1 || true
}
trap restore EXIT

python3 - "$REG" "$VIO" <<'PY'
from pathlib import Path
import sys
reg=Path(sys.argv[1]); vio=Path(sys.argv[2])
s=reg.read_text()
# Runtime switches are read every KF so one diagnostic build serves all modes.
anchor='''  // Add initial guess.\n  addStateValues(curr_kf_id_, status_smart_stereo_measurements_kf.first, pim);\n'''
repl='''  // JT-ZERO v15.53 diagnostic runtime switches.\n  const bool jt53_no_between = std::getenv("JTZERO_V15_53_NO_BETWEEN") != nullptr;\n  const bool jt53_no_lowdisp = std::getenv("JTZERO_V15_53_NO_LOWDISP") != nullptr;\n  const bool jt53_keep_visual = std::getenv("JTZERO_V15_53_KEEP_VISUAL") != nullptr;\n  // Add initial guess.\n  addStateValues(curr_kf_id_, status_smart_stereo_measurements_kf.first, pim);\n'''
if anchor not in s: raise SystemExit('v15.53 addStateValues anchor missing')
s=s.replace(anchor,repl,1)
anchor='''  if (backend_params_.addBetweenStereoFactors_ &&\n      status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_ ==\n          TrackingStatus::VALID) {\n'''
repl='''  if (!jt53_no_between && backend_params_.addBetweenStereoFactors_ &&\n      status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_ ==\n          TrackingStatus::VALID) {\n'''
if anchor not in s: raise SystemExit('v15.53 between anchor missing')
s=s.replace(anchor,repl,1)
anchor='''  const StereoMeasurements& smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n'''
repl='''  StereoMeasurements smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n  static size_t jt53_vis_in = 0, jt53_vis_out = 0, jt53_kfs = 0;\n  jt53_vis_in += smart_stereo_measurements_kf.size();\n  if (!jt53_keep_visual) smart_stereo_measurements_kf.clear();\n  jt53_vis_out += smart_stereo_measurements_kf.size();\n  ++jt53_kfs;\n  std::cerr << "JTZERO_V15_53_PATH kf=" << curr_kf_id_\n            << " vis_in=" << jt53_vis_in << " vis_out=" << jt53_vis_out\n            << " no_between=" << jt53_no_between\n            << " no_lowdisp=" << jt53_no_lowdisp\n            << " keep_visual=" << jt53_keep_visual << "\\n";\n'''
if anchor not in s: raise SystemExit('v15.53 visual anchor missing')
s=s.replace(anchor,repl,1)
# Suppress LOW_DISPARITY priors without changing tracking status or visual data.
anchor='''    case TrackingStatus::LOW_DISPARITY: {\n      // Vehicle is not moving.\n      VLOG(0) << "Tracker has a LOW_DISPARITY status.";\n      VLOG(10) << "Add zero velocity and no motion factors.";\n      addZeroVelocityPrior(curr_kf_id_);\n      addNoMotionFactor(last_kf_id_, curr_kf_id_);\n'''
repl='''    case TrackingStatus::LOW_DISPARITY: {\n      // Vehicle is not moving.\n      VLOG(0) << "Tracker has a LOW_DISPARITY status.";\n      VLOG(10) << "Add zero velocity and no motion factors.";\n      if (!jt53_no_lowdisp) {\n        addZeroVelocityPrior(curr_kf_id_);\n        addNoMotionFactor(last_kf_id_, curr_kf_id_);\n      }\n      std::cerr << "JTZERO_V15_53_LOWDISP kf=" << curr_kf_id_\n                << " suppressed=" << jt53_no_lowdisp << "\\n";\n'''
if anchor not in s: raise SystemExit('v15.53 low disparity anchor missing')
s=s.replace(anchor,repl,1)
reg.write_text(s)

# Instrument the actual preintegration prediction and optimized state.
s=vio.read_text()
anchor='''  const gtsam::NavState& navstate_k = pim.predict(navstate_lkf, imu_bias_lkf_);\n  debug_info_.navstate_k_ = navstate_k;\n'''
repl='''  const gtsam::NavState& navstate_k = pim.predict(navstate_lkf, imu_bias_lkf_);\n  debug_info_.navstate_k_ = navstate_k;\n  {\n    const auto jt53_dp = pim.deltaPij();\n    const auto jt53_dv = pim.deltaVij();\n    const auto jt53_dr = gtsam::Rot3::Logmap(pim.deltaRij());\n    const auto jt53_pp = navstate_k.pose().translation();\n    const auto jt53_vp = navstate_k.velocity();\n    const auto jt53_ba = imu_bias_lkf_.accelerometer();\n    const auto jt53_bg = imu_bias_lkf_.gyroscope();\n    std::cerr << "JTZERO_V15_53_PRE kf=" << frame_id\n              << " dt=" << pim.deltaTij()\n              << " dp=" << jt53_dp.x() << "," << jt53_dp.y() << "," << jt53_dp.z()\n              << " dv=" << jt53_dv.x() << "," << jt53_dv.y() << "," << jt53_dv.z()\n              << " dr=" << jt53_dr.x() << "," << jt53_dr.y() << "," << jt53_dr.z()\n              << " predp=" << jt53_pp.x() << "," << jt53_pp.y() << "," << jt53_pp.z()\n              << " predv=" << jt53_vp.x() << "," << jt53_vp.y() << "," << jt53_vp.z()\n              << " ba=" << jt53_ba.x() << "," << jt53_ba.y() << "," << jt53_ba.z()\n              << " bg=" << jt53_bg.x() << "," << jt53_bg.y() << "," << jt53_bg.z() << "\\n";\n  }\n'''
if anchor not in s: raise SystemExit('v15.53 predict anchor missing')
s=s.replace(anchor,repl,1)
anchor='''  imu_bias_lkf_ = state_.at<gtsam::imuBias::ConstantBias>(\n      gtsam::Symbol(kImuBiasSymbolChar, cur_id));\n'''
repl='''  imu_bias_lkf_ = state_.at<gtsam::imuBias::ConstantBias>(\n      gtsam::Symbol(kImuBiasSymbolChar, cur_id));\n  {\n    const auto jt53_po = W_Pose_B_lkf_from_state_.translation();\n    const auto jt53_vo = W_Vel_B_lkf_;\n    const auto jt53_ba = imu_bias_lkf_.accelerometer();\n    const auto jt53_bg = imu_bias_lkf_.gyroscope();\n    std::cerr << "JTZERO_V15_53_POST kf=" << cur_id\n              << " optp=" << jt53_po.x() << "," << jt53_po.y() << "," << jt53_po.z()\n              << " optv=" << jt53_vo.x() << "," << jt53_vo.y() << "," << jt53_vo.z()\n              << " ba=" << jt53_ba.x() << "," << jt53_ba.y() << "," << jt53_ba.z()\n              << " bg=" << jt53_bg.x() << "," << jt53_bg.y() << "," << jt53_bg.z() << "\\n";\n  }\n'''
if anchor not in s: raise SystemExit('v15.53 updateStates anchor missing')
s=s.replace(anchor,repl,1)
vio.write_text(s)
PY

echo '============================================================'
echo 'JT-ZERO v15.53 FINAL STAGE-11 BACKEND FACTOR ISOLATION'
echo 'Separates visual SmartFactors, stereo between-factor, LOW_DISPARITY priors, and pure IMU.'
echo 'Also logs PIM deltaP/deltaV/deltaR, prediction, optimized state and bias.'
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

printf 'MODE\tRC\tSTATES\tDP_MM\tMAXEXC_MM\tVIS_OUT\tPATH_LINES\tLOWDISP_LINES\tPRE_LINES\tPOST_LINES\n' > "$OUT/report.tsv"
run(){
  local tag="$1" nb="$2" nl="$3" kv="$4"
  local log="$OUT/$tag.log" rc st dp mx vo pl ll pre post
  echo "[RUN] $tag no_between=$nb no_lowdisp=$nl keep_visual=$kv"
  set +e
  env \
    $( [[ "$nb" == 1 ]] && echo JTZERO_V15_53_NO_BETWEEN=1 ) \
    $( [[ "$nl" == 1 ]] && echo JTZERO_V15_53_NO_LOWDISP=1 ) \
    $( [[ "$kv" == 1 ]] && echo JTZERO_V15_53_KEEP_VISUAL=1 ) \
    LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$PARAMS" CURRENT "$BASE" "$CAM" "$MJPG" >"$log" 2>&1
  rc=$?
  set -e
  st=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  vo=$(grep JTZERO_V15_53_PATH "$log" | tail -1 | sed -n 's/.*vis_out=\([0-9]*\).*/\1/p'); vo=${vo:-NA}
  pl=$(grep -c JTZERO_V15_53_PATH "$log" || true)
  ll=$(grep -c JTZERO_V15_53_LOWDISP "$log" || true)
  pre=$(grep -c JTZERO_V15_53_PRE "$log" || true)
  post=$(grep -c JTZERO_V15_53_POST "$log" || true)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$tag" "$rc" "$st" "${dp:-NA}" "${mx:-NA}" "$vo" "$pl" "$ll" "$pre" "$post" >> "$OUT/report.tsv"
  echo " rc=$rc states=$st dP=${dp:-NA} vis_out=$vo lowdisp=$ll pre=$pre post=$post"
}

# First four are the crucial causal decomposition. VIS_ON modes tell whether
# the same conclusion survives when normal SmartFactors are restored.
run VIS_OFF_BETWEEN_ON_LOWDISP_ON 0 0 0
run VIS_OFF_BETWEEN_OFF_LOWDISP_ON 1 0 0
run VIS_OFF_BETWEEN_ON_LOWDISP_OFF 0 1 0
run PURE_IMU 1 1 0
run VIS_ON_BETWEEN_ON_LOWDISP_ON 0 0 1
run VIS_ON_BETWEEN_OFF_LOWDISP_ON 1 0 1
run VIS_ON_BETWEEN_ON_LOWDISP_OFF 0 1 1
run VIS_ON_BETWEEN_OFF_LOWDISP_OFF 1 1 1

echo
echo '================ v15.53 MATRIX ================'
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

python3 - "$OUT/report.tsv" "$OUT" <<'PY' | tee "$OUT/summary.txt"
import csv,sys,math,re
report,out=sys.argv[1:]
r=list(csv.DictReader(open(report),delimiter='\t'))
D={x['MODE']:x for x in r}
def val(m,k='DP_MM'):
    try:return float(D[m][k])
    except:return math.nan
base=val('VIS_OFF_BETWEEN_ON_LOWDISP_ON')
nb=val('VIS_OFF_BETWEEN_OFF_LOWDISP_ON')
nl=val('VIS_OFF_BETWEEN_ON_LOWDISP_OFF')
pure=val('PURE_IMU')
print(f'BASE_VIS_OFF_DP_MM={base:.3f}')
print(f'NO_BETWEEN_DP_MM={nb:.3f}')
print(f'NO_LOWDISP_DP_MM={nl:.3f}')
print(f'PURE_IMU_DP_MM={pure:.3f}')
# Hard path proof for visual-off modes.
proof=True
for m in ['VIS_OFF_BETWEEN_ON_LOWDISP_ON','VIS_OFF_BETWEEN_OFF_LOWDISP_ON','VIS_OFF_BETWEEN_ON_LOWDISP_OFF','PURE_IMU']:
    x=D[m]
    proof &= int(x['RC'])==0 and int(x['STATES'])>=140 and int(x['PATH_LINES'])>=140 and x['VIS_OUT']=='0' and int(x['PRE_LINES'])>=140 and int(x['POST_LINES'])>=140
print('HARD_PATH_PROOF='+('PASS' if proof else 'FAIL'))
print('LOWDISP_EVENT_COUNT='+D['PURE_IMU']['LOWDISP_LINES'])
if math.isfinite(base) and math.isfinite(nb): print(f'BETWEEN_EFFECT_MM={base-nb:.3f}')
if math.isfinite(base) and math.isfinite(nl): print(f'LOWDISP_EFFECT_MM={base-nl:.3f}')
if math.isfinite(base) and math.isfinite(pure): print(f'NON_IMU_CONSTRAINT_EFFECT_MM={base-pure:.3f}')

# Parse PURE_IMU PIM/prediction/optimized logs to identify where motion appears.
log=open(out+'/PURE_IMU.log',errors='replace').read().splitlines()
def vec(s,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',s)
    return tuple(map(float,m.groups())) if m else None
def norm(v):return math.sqrt(sum(x*x for x in v)) if v else math.nan
pre=[]; post=[]
for line in log:
    if 'JTZERO_V15_53_PRE ' in line:
        pre.append((vec(line,'dp'),vec(line,'dv'),vec(line,'dr'),vec(line,'predp'),vec(line,'predv'),vec(line,'ba'),vec(line,'bg')))
    elif 'JTZERO_V15_53_POST ' in line:
        post.append((vec(line,'optp'),vec(line,'optv'),vec(line,'ba'),vec(line,'bg')))
if pre:
    print('PURE_IMU_PIM_MAX_DP_MM=%.3f'%(1000*max(norm(x[0]) for x in pre)))
    print('PURE_IMU_PIM_MAX_DV_MPS=%.6f'%max(norm(x[1]) for x in pre))
    print('PURE_IMU_PIM_MAX_DR_DEG=%.3f'%(180/math.pi*max(norm(x[2]) for x in pre)))
    p0=pre[0][3]; pn=pre[-1][3]
    print('PURE_IMU_PRED_FINAL_DELTA_MM=%.3f'%(1000*norm(tuple(pn[i]-p0[i] for i in range(3)))))
    print('PURE_IMU_PRED_MAX_VEL_MPS=%.6f'%max(norm(x[4]) for x in pre))
if post:
    p0=post[0][0]; pn=post[-1][0]
    print('PURE_IMU_OPT_FINAL_DELTA_MM=%.3f'%(1000*norm(tuple(pn[i]-p0[i] for i in range(3)))))
    print('PURE_IMU_OPT_MAX_VEL_MPS=%.6f'%max(norm(x[1]) for x in post))
    ba0,baf=post[0][2],post[-1][2]; bg0,bgf=post[0][3],post[-1][3]
    print('PURE_IMU_BA_CHANGE=%.9f'%norm(tuple(baf[i]-ba0[i] for i in range(3))))
    print('PURE_IMU_BG_CHANGE=%.9f'%norm(tuple(bgf[i]-bg0[i] for i in range(3))))

if not proof:
    verdict='INVALID_EXECUTION_PROOF'
elif math.isfinite(nb) and nb < 0.5*base:
    verdict='STEREO_BETWEEN_FACTOR_DOMINANT'
elif math.isfinite(nl) and nl < 0.5*base:
    verdict='LOW_DISPARITY_PRIORS_DOMINANT'
elif math.isfinite(pure) and pure < 0.5*base:
    verdict='NON_IMU_CONSTRAINT_COUPLING_DOMINANT'
else:
    verdict='PURE_IMU_PREINTEGRATION_STATE_DRIFT_CONFIRMED'
print('V15_53_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo "Logs: $OUT"
echo 'This is diagnostic iteration 3/3 for Stage 11.'
echo 'Sources will now be restored and Kimera rebuilt clean by trap.'
