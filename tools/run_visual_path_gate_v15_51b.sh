#!/usr/bin/env bash
set -euo pipefail
ROOT="$HOME/jtzero-kimera-sync"; KIMERA=/home/vio/Kimera-VIO
SRC="$KIMERA/src/backend/RegularVioBackend.cpp"; BAK=/tmp/RegularVioBackend.cpp.v15_51b.bak
PARAMS="$ROOT/params/JTZeroMonoFLU"; IMU=/home/vio/jtzero_yaw_only_v15_42.csv; CAM=/home/vio/jtzero_yaw_only_v15_42_camera.csv; MJPG=/home/vio/jtzero_yaw_only_v15_42.mjpg
TMP=/tmp/jtzero_v15_51b; BIN="$TMP/replay"; OUT=/home/vio/jtzero_visual_path_v15_51b
mkdir -p "$TMP" "$OUT"; rm -f "$OUT"/*.log "$OUT/report.tsv"; cp "$SRC" "$BAK"
restore(){ cp "$BAK" "$SRC"; echo '[RESTORE] Restored clean RegularVioBackend.cpp'; cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_51b_restore.log 2>&1 || true; }
trap restore EXIT
python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]);s=p.read_text()
if '#include <cstdlib>' not in s: s=s.replace('#include <', '#include <cstdlib>\n#include <',1)
anchor='namespace VIO {\n'
g='''namespace VIO {\nstatic bool jt51b_rot=false, jt51b_seen=false;\nstatic std::string jt51b_mode(){const char* p=std::getenv("JTZERO_V1551B_MODE");return p?p:"CONTROL";}\nstatic double jt51b_thr(){const char* p=std::getenv("JTZERO_V1551B_THR");return p?std::atof(p):0.25;}\n'''
if anchor not in s: raise SystemExit('namespace anchor missing')
s=s.replace(anchor,g,1)
needle='''  last_kf_id_ = curr_kf_id_;\n  ++curr_kf_id_;\n'''
ins=needle+'''  const double jt51b_deg=gtsam::Rot3::Logmap(pim.deltaRij()).norm()*180.0/M_PI;\n  jt51b_rot=jt51b_deg>=jt51b_thr(); if(jt51b_rot) jt51b_seen=true;\n'''
if needle not in s: raise SystemExit('state anchor missing')
s=s.replace(needle,ins,1)
needle='''  const StereoMeasurements& smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n'''
ins='''  StereoMeasurements smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n  const std::string jt51b=jt51b_mode();\n  if(jt51b=="NO_VIS_ALL") smart_stereo_measurements_kf.clear();\n  else if(jt51b=="NO_VIS_DURING_ROT" && jt51b_rot) smart_stereo_measurements_kf.clear();\n  else if(jt51b=="NO_VIS_AFTER_TRIGGER" && jt51b_seen) smart_stereo_measurements_kf.clear();\n  else if(jt51b=="NO_VIS_BEFORE_TRIGGER" && !jt51b_seen) smart_stereo_measurements_kf.clear();\n  else if((jt51b=="ROT_NEW_ONLY" || jt51b=="ROT_EXISTING_ONLY") && jt51b_rot){\n    StereoMeasurements f; f.reserve(smart_stereo_measurements_kf.size());\n    for(const auto& m:smart_stereo_measurements_kf){const bool ex=feature_tracks_.find(m.first)!=feature_tracks_.end();\n      if((jt51b=="ROT_NEW_ONLY"&&!ex)||(jt51b=="ROT_EXISTING_ONLY"&&ex)) f.push_back(m);}\n    smart_stereo_measurements_kf.swap(f);\n  }\n  if(jt51b!="CONTROL") LOG(INFO)<<"JTZERO_V15_51B mode="<<jt51b<<" kf="<<curr_kf_id_<<" rot="<<jt51b_rot<<" seen="<<jt51b_seen<<" meas="<<smart_stereo_measurements_kf.size();\n'''
if needle not in s: raise SystemExit('measurement anchor missing')
s=s.replace(needle,ins,1);p.write_text(s)
PY
echo '============================================================'; echo 'JT-ZERO v15.51b ACTIVE VISUAL PATH MATRIX'; echo 'Patches RegularVioBackend.cpp (actual active override).'; echo '============================================================'
echo '[BUILD] diagnostic Kimera'; cmake --build "$KIMERA/build" -j"$(nproc)"
echo '[BUILD] replay'; g++ -std=c++17 -O2 -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 $(pkg-config --cflags opencv4) "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" -L"$KIMERA/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
printf 'MODE\tTHR\tRC\tSTATES\tDP_MM\tMAXEXC_MM\tGUARD_LINES\n' > "$OUT/report.tsv"
run(){
  local mode="$1"
  local thr="$2"
  local tag="$3"
  local log="$OUT/$tag.log"
  local rc st dp mx gl
  echo "[RUN] $mode thr=$thr"
  set +e
  JTZERO_V1551B_MODE="$mode" JTZERO_V1551B_THR="$thr" LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  rc=$?
  set -e
  st=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  gl=$(grep -c JTZERO_V15_51B "$log" || true)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$mode" "$thr" "$rc" "$st" "${dp:-NA}" "${mx:-NA}" "$gl" >> "$OUT/report.tsv"
  echo " rc=$rc states=$st dP=${dp:-NA} guard_lines=$gl"
}
run CONTROL 0.25 control
for t in 0.10 0.25 0.50 1.00 2.00; do run NO_VIS_DURING_ROT "$t" "during_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run NO_VIS_AFTER_TRIGGER "$t" "after_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run NO_VIS_BEFORE_TRIGGER "$t" "before_${t/./p}"; done
for m in ROT_NEW_ONLY ROT_EXISTING_ONLY; do for t in 0.10 0.25 0.50 1.00; do run "$m" "$t" "${m,,}_${t/./p}"; done; done
run NO_VIS_ALL 0.25 no_vis_all
echo; echo '================ v15.51b MATRIX ================'; column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"
python3 - "$OUT/report.tsv" <<'PY'
import csv,sys
r=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))
valid=[]
for x in r:
 try:
  if int(x['RC'])==0 and int(x['STATES'])>=140: valid.append(x)
 except: pass
c=next((x for x in valid if x['MODE']=='CONTROL'),None)
if not c: print('V15_51B_VERDICT=INVALID_CONTROL');print('RESULT: COMPLETE');raise SystemExit
cd=float(c['DP_MM']); print(f'CONTROL_DP_MM={cd:.3f} CONTROL_STATES={c["STATES"]}')
ranked=sorted([(float(x['DP_MM']),i,x) for i,x in enumerate(valid) if x['MODE']!='CONTROL'],key=lambda z:(z[0],z[1]))
for i,(d,_,x) in enumerate(ranked[:12],1): print(f'RANK{i}={x["MODE"]}:THR={x["THR"]}:DP_MM={d:.3f}:RATIO={d/cd:.4f}:STATES={x["STATES"]}:GUARD_LINES={x["GUARD_LINES"]}')
nv=next((x for x in valid if x['MODE']=='NO_VIS_ALL'),None)
if nv: print(f'NO_VIS_ALL_DP_MM={float(nv["DP_MM"]):.3f}')
if any(int(x['GUARD_LINES'])>0 for x in valid if x['MODE']!='CONTROL'):
 verdict='ACTIVE_PATH_CONFIRMED_MATRIX_VALID'
else: verdict='PATCH_NOT_ACTIVE_INVALID'
print('V15_51B_VERDICT='+verdict); print('RESULT: COMPLETE')
PY
echo "Logs: $OUT"
