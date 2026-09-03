#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
OUT=/home/vio/jtzero_final_suite_v15_29_results
RAW=/home/vio/jtzero_final_suite_v15_29.csv
CAM=/home/vio/jtzero_final_suite_v15_29_camera.csv
MJPG=/home/vio/jtzero_final_suite_v15_29.mjpg
ATT=/home/vio/jtzero_final_suite_v15_29_attitude.csv
PHASES=/home/vio/jtzero_final_suite_v15_29_phases.csv
REC=/tmp/record_mono_imu_final_suite_gui_v15_29
REPLAY=/tmp/replay_mono_imu_zxy_ab_v11_v15_29
AN=/tmp/analyze_final_suite_segments_v15_29
SRC="$KIMERA/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_29.bak
mkdir -p "$OUT"

if ! git -C "$KIMERA" diff --quiet -- src/backend/VioBackend.cpp; then echo 'ERROR: dirty VioBackend.cpp' >&2; exit 1; fi

build_tools(){
  echo '[BUILD] recorder'
  g++ -std=c++17 -O2 "$ROOT/tools/record_mono_imu_final_suite_gui_v15_29.cpp" -o "$REC" \
    -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
    $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
    -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
  echo '[BUILD] replay'
  g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$REPLAY" \
    -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
    $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
    -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
  g++ -std=c++17 -O2 "$ROOT/tools/analyze_final_suite_segments_v15_29.cpp" -o "$AN"
}

run_case(){
  local name="$1" H="$2" relin="$3" mode="${4:-OFF}" target="${5:--1}"
  local P="$OUT/params_$name" LOG="$OUT/$name.txt"
  rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
  sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $H/" "$P/BackendParams.yaml"
  sed -i -E "s/^[[:space:]]*relinearizeThreshold:[[:space:]]*.*/relinearizeThreshold: $relin/" "$P/BackendParams.yaml"
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  JT1529_MODE="$mode" JT1529_TARGET="$target" LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$REPLAY" "$P" CURRENT "$RAW" "$CAM" "$MJPG" >"$LOG" 2>&1
  local rc=$?
  set -e
  if [[ $rc -ne 0 ]]; then echo "ERROR $name rc=$rc"; tail -100 "$LOG"; return $rc; fi
  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$OUT/$name.csv"
  "$AN" "$OUT/$name.csv" "$PHASES" "$name" > "$OUT/$name.segments.tsv"
  local dp states
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG")
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
  echo "$name H=$H relin=$relin mode=$mode target=$target states=${states:-NA} final_dP=${dp:-NA} trace=$(grep -c '\[JT15.29' "$LOG" || true)"
}

cleanup(){
  if [[ -f "$BAK" ]]; then cp "$BAK" "$SRC" 2>/dev/null || true; cmake --build "$KIMERA/build" -j2 --target kimera_vio >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT

printf '\n============================================================\n'
printf 'JT-ZERO FINAL DIAGNOSTIC SUITE v15.29\n'
printf 'Один физический прогон + автоматическая causal/production matrix\n'
printf '============================================================\n\n'

echo '[1/8] Build clean Kimera and tools'
cmake --build "$KIMERA/build" -j2 --target kimera_vio
build_tools

echo '[2/8] Physical recording — follow Russian fullscreen GUI'
rm -f "$RAW" "$CAM" "$MJPG" "$ATT" "$PHASES"
LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$REC"
for f in "$RAW" "$CAM" "$MJPG" "$ATT" "$PHASES"; do [[ -s "$f" ]] || { echo "ERROR: missing $f" >&2; exit 2; }; done

echo '[3/8] Clean production/control matrix'
run_case H25_CURRENT 25 0.01
run_case H28_CONTROL 28 0.01
run_case H29 29 0.01
run_case H35 35 0.01
run_case RELIN_TIGHT 25 0.001

echo '[4/8] Inject generic fixed-lag boundary causal instrumentation'
cp "$SRC" "$BAK"
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo 'ERROR: smoother update anchor not unique' >&2; exit 3; }
TMP=/tmp/VioBackend.cpp.v15_29.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          std::map<Key,double> jt1529_ts = timestamps;";
 print "          gtsam::FactorIndices jt1529_delete = delete_slots;";
 print "          const char* jt1529_mode_c = std::getenv(\"JT1529_MODE\");";
 print "          const char* jt1529_target_c = std::getenv(\"JT1529_TARGET\");";
 print "          const std::string jt1529_mode = jt1529_mode_c ? jt1529_mode_c : \"OFF\";";
 print "          const long jt1529_target = jt1529_target_c ? std::strtol(jt1529_target_c,nullptr,10) : -1;";
 print "          if (static_cast<size_t>(backend_params_.nr_states_) == 28 && curr_kf_id_ > 29) {";
 print "            const size_t jt1529_idx = static_cast<size_t>(curr_kf_id_) - 29;";
 print "            const gtsam::Key jt1529_x = gtsam::Symbol(\"x\"[0],jt1529_idx);";
 print "            const auto& jt1529_graph = smoother_->getFactors();";
 print "            size_t jt1529_smart = 0; std::vector<size_t> jt1529_slots;";
 print "            for(size_t jt1529_slot=0;jt1529_slot<jt1529_graph.size();++jt1529_slot){";
 print "              const auto& jt1529_f=jt1529_graph[jt1529_slot]; if(!jt1529_f) continue;";
 print "              bool jt1529_touch=false; for(const auto jt1529_k:jt1529_f->keys()) if(jt1529_k==jt1529_x){jt1529_touch=true;break;}";
 print "              if(!jt1529_touch) continue; const std::string jt1529_type=typeid(*jt1529_f).name();";
 print "              if(jt1529_type.find(\"SmartStereoProjectionPoseFactor\")!=std::string::npos){++jt1529_smart;jt1529_slots.push_back(jt1529_slot);}";
 print "            }";
 print "            if(jt1529_mode==\"SCAN\") std::cerr<<\"[JT15.29 SCAN] kf=\"<<curr_kf_id_<<\" idx=\"<<jt1529_idx<<\" smart=\"<<jt1529_smart<<std::endl;";
 print "            if(jt1529_target>=0 && static_cast<long>(jt1529_idx)==jt1529_target && jt1529_mode==\"DELAY\") {";
 print "              const double nt=static_cast<double>(jt1529_idx+1); jt1529_ts[gtsam::Symbol(\"x\"[0],jt1529_idx)]=nt; jt1529_ts[gtsam::Symbol(\"v\"[0],jt1529_idx)]=nt; jt1529_ts[gtsam::Symbol(\"b\"[0],jt1529_idx)]=nt;";
 print "              std::cerr<<\"[JT15.29 DELAY] kf=\"<<curr_kf_id_<<\" idx=\"<<jt1529_idx<<\" smart=\"<<jt1529_smart<<std::endl;";
 print "            }";
 print "            if(jt1529_target>=0 && static_cast<long>(jt1529_idx)==jt1529_target && jt1529_mode==\"ABLATE\") {";
 print "              for(const auto slot:jt1529_slots) if(std::find(jt1529_delete.begin(),jt1529_delete.end(),slot)==jt1529_delete.end()) jt1529_delete.push_back(slot);";
 print "              std::cerr<<\"[JT15.29 ABLATE] kf=\"<<curr_kf_id_<<\" idx=\"<<jt1529_idx<<\" smart=\"<<jt1529_smart<<\" slots=\"<<jt1529_slots.size()<<std::endl;";
 print "            }";
 print "          }";
 print "          return smoother_->update(new_factors,new_values,jt1529_ts,jt1529_delete);";
 print "        })();"; done=1; next
}
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"
mv "$TMP" "$SRC"
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo '[5/8] Scan H28 marginalization boundary for densest SmartStereo state'
run_case H28_SCAN 28 0.01 SCAN -1
TARGET=$(grep '\[JT15.29 SCAN\]' "$OUT/H28_SCAN.txt" | sed -n 's/.*idx=\([0-9][0-9]*\) smart=\([0-9][0-9]*\).*/\2 \1/p' | sort -nr -k1,1 -k2,2 | head -1 | awk '{print $2}')
MAXSMART=$(grep '\[JT15.29 SCAN\]' "$OUT/H28_SCAN.txt" | sed -n 's/.*idx=\([0-9][0-9]*\) smart=\([0-9][0-9]*\).*/\2/p' | sort -nr | head -1)
if [[ -z "${TARGET:-}" ]]; then echo 'ERROR: no scan target' >&2; exit 4; fi
echo "[CAUSAL] densest boundary pose: x$TARGET, SmartStereo factors=${MAXSMART:-0}"

echo '[6/8] Causal target runs: one-update delay and SmartFactor ablation'
run_case H28_DELAY_AUTO 28 0.01 DELAY "$TARGET"
run_case H28_ABLATE_AUTO 28 0.01 ABLATE "$TARGET"

echo '[7/8] Restore clean Kimera'
cp "$BAK" "$SRC"; cmake --build "$KIMERA/build" -j2 --target kimera_vio; rm -f "$BAK"; trap - EXIT

echo '[8/8] Build combined report'
{
  head -1 "$OUT/H25_CURRENT.segments.tsv"
  for n in H25_CURRENT H28_CONTROL H29 H35 RELIN_TIGHT H28_SCAN H28_DELAY_AUTO H28_ABLATE_AUTO; do tail -n +2 "$OUT/$n.segments.tsv"; done
} > "$OUT/segments_all.tsv"

metric(){ awk -F'\t' -v L="$1" -v P="$2" '$1==L&&$2==P{print $4;exit}' "$OUT/segments_all.tsv"; }
max3(){ awk -v a="$1" -v b="$2" -v c="$3" 'BEGIN{x=a+0;if(b+0>x)x=b+0;if(c+0>x)x=c+0;print x}'; }
absdiff(){ awk -v a="$1" -v b="$2" 'BEGIN{x=a-b;if(x<0)x=-x;print x}'; }
BASE_YAW=$(max3 "$(metric H25_CURRENT YAW_A)" "$(metric H25_CURRENT YAW_RETURN_A)" "$(metric H25_CURRENT YAW_RETURN_500)")

REPORT="$OUT/summary.txt"
{
 echo '============================================================'
 echo 'JT-ZERO FINAL DIAGNOSTIC SUITE v15.29'
 echo '============================================================'
 echo "critical_boundary_pose=x$TARGET"
 echo "critical_smart_factors=${MAXSMART:-0}"
 echo "base_H25_max_pure_yaw_dxy_mm=$BASE_YAW"
 echo
 printf '%-18s %12s %12s %12s %12s %12s\n' VARIANT YAW_A YAW_RET_A YAW_RET_500 MOVE_YAW MOVE_BACK
 for n in H25_CURRENT H28_CONTROL H29 H35 RELIN_TIGHT H28_DELAY_AUTO H28_ABLATE_AUTO; do
   printf '%-18s %12s %12s %12s %12s %12s\n' "$n" "$(metric "$n" YAW_A)" "$(metric "$n" YAW_RETURN_A)" "$(metric "$n" YAW_RETURN_500)" "$(metric "$n" MOVE_YAW_500)" "$(metric "$n" MOVE_BACK_500)"
 done
 echo
 echo 'PRODUCTION GATE: pure-yaw false XY <= max(30 mm, 30% of H25), both 500-mm motions within +/-75 mm.'
 best='NONE'; bestyaw=999999
 for n in H29 H35 RELIN_TIGHT; do
   y=$(max3 "$(metric "$n" YAW_A)" "$(metric "$n" YAW_RETURN_A)" "$(metric "$n" YAW_RETURN_500)")
   e1=$(absdiff "$(metric "$n" MOVE_YAW_500)" 500); e2=$(absdiff "$(metric "$n" MOVE_BACK_500)" 500)
   gate=$(awk -v y="$y" -v base="$BASE_YAW" -v e1="$e1" -v e2="$e2" 'BEGIN{lim=0.3*base;if(lim<30)lim=30;print (y<=lim&&e1<=75&&e2<=75)?1:0}')
   echo "$n max_yaw_false_xy_mm=$y move_yaw_error_mm=$e1 move_back_error_mm=$e2 gate=$([[ $gate == 1 ]]&&echo PASS||echo FAIL)"
   if [[ $gate == 1 ]]; then less=$(awk -v a="$y" -v b="$bestyaw" 'BEGIN{print a<b?1:0}'); if [[ $less == 1 ]]; then best="$n"; bestyaw="$y"; fi; fi
 done
 echo
 echo "PRODUCTION_CANDIDATE=$best"
 if [[ "$best" != NONE ]]; then echo 'ITEM_11_STATUS=READY_FOR_FINAL_LIVE_VALIDATION'; else echo 'ITEM_11_STATUS=BLOCKED_BY_FINAL_SUITE'; fi
 echo 'CAUSAL_NOTE=DELAY and ABLATE are diagnostics only; never production candidates.'
} | tee "$REPORT"

echo
echo "Raw dataset: $RAW"
echo "Segment metrics: $OUT/segments_all.tsv"
echo "Final report: $REPORT"
echo 'RESULT: COMPLETE'
