#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/jtzero-kimera-sync"
KIMERA="/home/vio/Kimera-VIO"
SRC="$KIMERA/src/backend/RegularVioBackend.cpp"
BAK=/tmp/RegularVioBackend.cpp.v15_51c.bak
PARAMS="$ROOT/params/JTZeroMonoFLU"
IMU=/home/vio/jtzero_yaw_only_v15_42.csv
CAM=/home/vio/jtzero_yaw_only_v15_42_camera.csv
MJPG=/home/vio/jtzero_yaw_only_v15_42.mjpg
TMP=/tmp/jtzero_v15_51c
BIN="$TMP/replay"
OUT=/home/vio/jtzero_visual_execution_v15_51c

mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/*.proof.csv "$OUT/report.tsv"
cp "$SRC" "$BAK"

restore(){
  cp "$BAK" "$SRC"
  echo '[RESTORE] Restored clean RegularVioBackend.cpp'
  cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_51c_restore.log 2>&1 || true
}
trap restore EXIT

python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); s=p.read_text()
if '#include <cstdlib>' not in s:
    s=s.replace('#include <', '#include <cstdlib>\n#include <fstream>\n#include <', 1)
elif '#include <fstream>' not in s:
    s=s.replace('#include <cstdlib>\n', '#include <cstdlib>\n#include <fstream>\n', 1)

anchor='namespace VIO {\n'
g=r'''namespace VIO {
static bool jt51c_rot=false, jt51c_seen=false;
static std::string jt51c_mode(){const char* p=std::getenv("JTZERO_V1551C_MODE");return p?p:"CONTROL";}
static double jt51c_thr(){const char* p=std::getenv("JTZERO_V1551C_THR");return p?std::atof(p):0.25;}
static int jt51c_n(){const char* p=std::getenv("JTZERO_V1551C_N");return p?std::atoi(p):0;}
static const char* jt51c_proof(){const char* p=std::getenv("JTZERO_V1551C_PROOF");return p?p:"";}
'''
if anchor not in s: raise SystemExit('namespace anchor missing')
s=s.replace(anchor,g,1)

needle='''  last_kf_id_ = curr_kf_id_;\n  ++curr_kf_id_;\n'''
ins=needle+'''  const double jt51c_deg = gtsam::Rot3::Logmap(pim.deltaRij()).norm()*180.0/M_PI;\n  jt51c_rot = jt51c_deg >= jt51c_thr();\n  if (jt51c_rot) jt51c_seen = true;\n'''
if needle not in s: raise SystemExit('state anchor missing')
s=s.replace(needle,ins,1)

needle='''  const StereoMeasurements& smart_stereo_measurements_kf =\n      status_smart_stereo_measurements_kf.second;\n'''
ins=r'''  StereoMeasurements smart_stereo_measurements_kf =
      status_smart_stereo_measurements_kf.second;

  const std::string jt51c = jt51c_mode();
  const size_t jt51c_in = smart_stereo_measurements_kf.size();
  size_t jt51c_new = 0, jt51c_existing = 0;
  for (const auto& m : smart_stereo_measurements_kf) {
    if (feature_tracks_.find(m.first) == feature_tracks_.end()) ++jt51c_new;
    else ++jt51c_existing;
  }

  if (jt51c == "NO_VIS_ALL") {
    smart_stereo_measurements_kf.clear();
  } else if (jt51c == "NO_VIS_DURING_ROT" && jt51c_rot) {
    smart_stereo_measurements_kf.clear();
  } else if (jt51c == "NO_VIS_AFTER_TRIGGER" && jt51c_seen) {
    smart_stereo_measurements_kf.clear();
  } else if (jt51c == "NO_VIS_BEFORE_TRIGGER" && !jt51c_seen) {
    smart_stereo_measurements_kf.clear();
  } else if (jt51c == "KEEP_NEW_ONLY" && jt51c_rot) {
    StereoMeasurements f; f.reserve(smart_stereo_measurements_kf.size());
    for (const auto& m : smart_stereo_measurements_kf)
      if (feature_tracks_.find(m.first) == feature_tracks_.end()) f.push_back(m);
    smart_stereo_measurements_kf.swap(f);
  } else if (jt51c == "KEEP_EXISTING_ONLY" && jt51c_rot) {
    StereoMeasurements f; f.reserve(smart_stereo_measurements_kf.size());
    for (const auto& m : smart_stereo_measurements_kf)
      if (feature_tracks_.find(m.first) != feature_tracks_.end()) f.push_back(m);
    smart_stereo_measurements_kf.swap(f);
  } else if (jt51c == "DROP_FIRST_N" && curr_kf_id_ <= jt51c_n()) {
    smart_stereo_measurements_kf.clear();
  } else if (jt51c == "DROP_AFTER_N" && curr_kf_id_ > jt51c_n()) {
    smart_stereo_measurements_kf.clear();
  }

  if (const char* pp = jt51c_proof(); pp && *pp) {
    std::ofstream pf(pp, std::ios::app);
    pf << curr_kf_id_ << ',' << jt51c << ',' << jt51c_thr() << ',' << jt51c_n()
       << ',' << jt51c_deg << ',' << (jt51c_rot?1:0) << ',' << (jt51c_seen?1:0)
       << ',' << jt51c_in << ',' << smart_stereo_measurements_kf.size()
       << ',' << jt51c_new << ',' << jt51c_existing << '\n';
  }
'''
if needle not in s: raise SystemExit('measurement anchor missing')
s=s.replace(needle,ins,1)
p.write_text(s)
PY

echo '============================================================'
echo 'JT-ZERO v15.51c HARD EXECUTION + TIME PARTITION MATRIX'
echo 'Hard proof is written directly by active RegularVioBackend.cpp.'
echo 'One diagnostic Kimera build; all hypotheses are runtime modes.'
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
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread \
  $(pkg-config --libs opencv4)

printf 'MODE\tTHR\tN\tRC\tSTATES\tDP_MM\tMAXEXC_MM\tPROOF_LINES\tSUM_IN\tSUM_OUT\tZERO_OUT_LINES\tROT_LINES\n' > "$OUT/report.tsv"

run_one(){
  local mode="$1" thr="$2" n="$3" tag="$4"
  local log="$OUT/$tag.log" proof="$OUT/$tag.proof.csv"
  local rc st dp mx proof_lines sums
  rm -f "$proof"
  echo "[RUN] mode=$mode thr=$thr n=$n"
  set +e
  JTZERO_V1551C_MODE="$mode" \
  JTZERO_V1551C_THR="$thr" \
  JTZERO_V1551C_N="$n" \
  JTZERO_V1551C_PROOF="$proof" \
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  rc=$?
  set -e
  st=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  proof_lines=$(wc -l < "$proof" 2>/dev/null || echo 0)
  sums=$(awk -F, '{si+=$8;so+=$9;if($9==0)z++;if($6==1)r++} END{printf "%d %d %d %d",si,so,z,r}' "$proof" 2>/dev/null || echo '0 0 0 0')
  read -r sum_in sum_out zero_out rot_lines <<<"$sums"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$mode" "$thr" "$n" "$rc" "$st" "${dp:-NA}" "${mx:-NA}" "$proof_lines" "$sum_in" "$sum_out" "$zero_out" "$rot_lines" >> "$OUT/report.tsv"
  echo " rc=$rc states=$st dP=${dp:-NA} proof=$proof_lines in=$sum_in out=$sum_out zero_out=$zero_out rot_lines=$rot_lines"
}

# Execution/data-flow controls.
run_one CONTROL 0.25 0 control
run_one NO_VIS_ALL 0.25 0 no_vis_all

# Rotation-conditioned partitions.
for t in 0.10 0.25 0.50 1.00 2.00; do run_one NO_VIS_DURING_ROT "$t" 0 "during_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run_one NO_VIS_AFTER_TRIGGER "$t" 0 "after_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run_one NO_VIS_BEFORE_TRIGGER "$t" 0 "before_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run_one KEEP_NEW_ONLY "$t" 0 "new_${t/./p}"; done
for t in 0.10 0.25 0.50 1.00; do run_one KEEP_EXISTING_ONLY "$t" 0 "existing_${t/./p}"; done

# Direct time-localization of visual influence. This is independent of rotation threshold.
for n in 1 2 4 8 12 16 24 32 48 64; do run_one DROP_FIRST_N 0.25 "$n" "drop_first_${n}"; done
for n in 1 2 4 8 12 16 24 32 48 64; do run_one DROP_AFTER_N 0.25 "$n" "drop_after_${n}"; done

echo
echo '================ v15.51c MATRIX ================'
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

echo
python3 - "$OUT/report.tsv" <<'PY'
import csv,sys
rows=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))

def i(x,k):
    try:return int(x[k])
    except:return -1
def f(x,k):
    try:return float(x[k])
    except:return float('nan')
valid=[x for x in rows if i(x,'RC')==0 and i(x,'STATES')>=140 and i(x,'PROOF_LINES')>0]
ctrl=next((x for x in valid if x['MODE']=='CONTROL'),None)
nv=next((x for x in valid if x['MODE']=='NO_VIS_ALL'),None)
if not ctrl:
    print('V15_51C_VERDICT=EXECUTION_PROOF_FAILED_NO_CONTROL')
    print('RESULT: COMPLETE'); raise SystemExit
cd=f(ctrl,'DP_MM')
print(f'CONTROL_DP_MM={cd:.3f} CONTROL_STATES={ctrl["STATES"]} CONTROL_PROOF_LINES={ctrl["PROOF_LINES"]}')
if nv:
    print(f'NO_VIS_ALL_DP_MM={f(nv,"DP_MM"):.3f} NO_VIS_ALL_SUM_IN={nv["SUM_IN"]} NO_VIS_ALL_SUM_OUT={nv["SUM_OUT"]} NO_VIS_ALL_ZERO_OUT_LINES={nv["ZERO_OUT_LINES"]}/{nv["PROOF_LINES"]}')

# Hard proof: NO_VIS_ALL must actually have zero visual measurements out on every backend call.
proof_ok = nv is not None and i(nv,'SUM_IN')>0 and i(nv,'SUM_OUT')==0 and i(nv,'ZERO_OUT_LINES')==i(nv,'PROOF_LINES')
print('HARD_NO_VIS_PROOF=' + ('PASS' if proof_ok else 'FAIL'))

ranked=sorted([(f(x,'DP_MM'),idx,x) for idx,x in enumerate(valid) if x['MODE'] not in ('CONTROL',)], key=lambda z:(z[0],z[1]))
for k,(d,_,x) in enumerate(ranked[:15],1):
    print(f'RANK{k}={x["MODE"]}:THR={x["THR"]}:N={x["N"]}:DP_MM={d:.3f}:RATIO={d/cd:.4f}:OUT={x["SUM_OUT"]}:PROOF={x["PROOF_LINES"]}')

first=[(f(x,'DP_MM'),i(x,'N')) for x in valid if x['MODE']=='DROP_FIRST_N']
after=[(f(x,'DP_MM'),i(x,'N')) for x in valid if x['MODE']=='DROP_AFTER_N']
if first:
    d,n=min(first); print(f'BEST_DROP_FIRST_N={n} BEST_DROP_FIRST_DP_MM={d:.3f} RATIO={d/cd:.4f}')
if after:
    d,n=min(after); print(f'BEST_DROP_AFTER_N={n} BEST_DROP_AFTER_DP_MM={d:.3f} RATIO={d/cd:.4f}')

if not proof_ok:
    verdict='INVALID_HARD_EXECUTION_PROOF'
elif nv and abs(f(nv,'DP_MM')-cd) <= 0.10*max(cd,1.0):
    # Visual path is definitely removed, yet final translation barely moves.
    # Any isolated branch improvement then reflects branch selection, not dominant visual forcing.
    verdict='VISUAL_FACTORS_NOT_DOMINANT_FINAL_TRANSLATION_DRIVER'
elif nv and f(nv,'DP_MM') <= 0.5*cd:
    verdict='VISUAL_FACTORS_DOMINANT_FINAL_TRANSLATION_DRIVER'
else:
    verdict='VISUAL_FACTORS_MODERATE_OR_BRANCH_DEPENDENT_EFFECT'
print('V15_51C_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo "Logs/proofs: $OUT"
echo 'Source will now be restored and Kimera rebuilt clean by trap.'
