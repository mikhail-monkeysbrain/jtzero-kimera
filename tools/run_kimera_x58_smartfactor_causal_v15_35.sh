#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$K/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_35.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_35
OUT=/home/vio/jtzero_x58_causal_v15_35
IMU="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAM="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
mkdir -p "$OUT"

for f in "$IMU" "$CAM" "$MJPG"; do [[ -s "$f" ]] || { echo "[FATAL] missing $f" >&2; exit 2; }; done
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then echo '[FATAL] dirty Kimera VioBackend.cpp' >&2; exit 3; fi
cp "$SRC" "$BAK"
CLEAN=0
cleanup(){ cp "$BAK" "$SRC" 2>/dev/null || true; if [[ "$CLEAN" -eq 0 ]]; then cmake --build "$K/build" -j2 --target kimera_vio || true; fi; }
trap cleanup EXIT

echo '============================================================'
echo 'JT-ZERO x58 SMARTSTEREO CAUSAL SWEEP v15.35'
echo 'Dataset: existing yaw_only_v13; H28; no physical rerun.'
echo '============================================================'

echo '[1/6] Instrument VioBackend.cpp'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo '[FATAL] smoother update anchor not unique' >&2; exit 4; }
TMP=/tmp/VioBackend.cpp.v15_35.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          gtsam::FactorIndices jt1535_delete = delete_slots;";
 print "          const char* jt1535_mode_c = std::getenv(\"JT1535_MODE\");";
 print "          const char* jt1535_idx_c = std::getenv(\"JT1535_INDEX\");";
 print "          const std::string jt1535_mode = jt1535_mode_c ? jt1535_mode_c : \"CONTROL\";";
 print "          const long jt1535_index = jt1535_idx_c ? std::strtol(jt1535_idx_c,nullptr,10) : -1;";
 print "          if (static_cast<size_t>(backend_params_.nr_states_) == 28 && curr_kf_id_ == 87) {";
 print "            const gtsam::Key jt1535_x = gtsam::Symbol(\"x\"[0],58);";
 print "            const auto& jt1535_graph = smoother_->getFactors();";
 print "            struct JT1535F { std::string sig; size_t slot; }; std::vector<JT1535F> jt1535_fs;";
 print "            for(size_t s=0;s<jt1535_graph.size();++s){ const auto& f=jt1535_graph[s]; if(!f)continue;";
 print "              bool touch=false; for(const auto k:f->keys())if(k==jt1535_x){touch=true;break;} if(!touch)continue;";
 print "              const std::string tn=typeid(*f).name(); if(tn.find(\"SmartStereoProjectionPoseFactor\")==std::string::npos)continue;";
 print "              std::ostringstream os; bool first=true; for(const auto k:f->keys()){gtsam::Symbol sy(k); if(sy.chr()!=\"x\"[0])continue; if(!first)os<<\",\"; first=false; os<<\"x\"<<sy.index();}";
 print "              jt1535_fs.push_back({os.str(),s}); }";
 print "            std::sort(jt1535_fs.begin(),jt1535_fs.end(),[](const JT1535F&a,const JT1535F&b){if(a.sig!=b.sig)return a.sig<b.sig;return a.slot<b.slot;});";
 print "            std::cerr<<\"[JT15.35 SET] kf=87 count=\"<<jt1535_fs.size()<<std::endl;";
 print "            for(size_t i=0;i<jt1535_fs.size();++i)std::cerr<<\"[JT15.35 F] F\"<<i<<\" slot=\"<<jt1535_fs[i].slot<<\" keys={\"<<jt1535_fs[i].sig<<\"}\"<<std::endl;";
 print "            for(size_t i=0;i<jt1535_fs.size();++i){ bool drop=false;";
 print "              if(jt1535_mode==\"DROP_ALL\")drop=true;";
 print "              else if(jt1535_mode==\"DROP_ONE\" && static_cast<long>(i)==jt1535_index)drop=true;";
 print "              else if(jt1535_mode==\"KEEP_ONE\" && static_cast<long>(i)!=jt1535_index)drop=true;";
 print "              if(drop){const size_t s=jt1535_fs[i].slot;if(std::find(jt1535_delete.begin(),jt1535_delete.end(),s)==jt1535_delete.end())jt1535_delete.push_back(s);}";
 print "            }";
 print "            std::cerr<<\"[JT15.35 MODE] \"<<jt1535_mode<<\" index=\"<<jt1535_index<<\" delete_total=\"<<jt1535_delete.size()<<std::endl;";
 print "          }";
 print "          return smoother_->update(new_factors,new_values,timestamps,jt1535_delete);";
 print "        })();"; done=1; next
}
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"
mv "$TMP" "$SRC"
# Added code needs sstream/iostream; inject only if absent.
grep -q '^#include <sstream>' "$SRC" || sed -i '/#include <string>/i #include <sstream>\n#include <iostream>' "$SRC"

echo '[2/6] Build instrumented Kimera'
cmake --build "$K/build" -j2 --target kimera_vio

echo '[3/6] Build deterministic replay'
g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
 -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
 $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)

P="$OUT/params_H28"; rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
SUMMARY="$OUT/summary.tsv"
echo -e 'variant\trc\tstates\tfinal_dp_mm\tpath_mm\tmax_exc_mm\ttrace_count' > "$SUMMARY"
run(){
 local name="$1" mode="$2" idx="$3" log="$OUT/$name.txt"
 echo -n "[RUN] $name ... "
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 set +e
 JT1535_MODE="$mode" JT1535_INDEX="$idx" LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
   "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
 local rc=$?
 set -e
 local st dp path exc tr
 st=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log")
 dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$log")
 path=$(awk -F': ' '/^path mm:/{print $2;exit}' "$log")
 exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$log")
 tr=$(grep -c '\[JT15.35' "$log" || true)
 echo -e "$name\t$rc\t${st:-NA}\t${dp:-NA}\t${path:-NA}\t${exc:-NA}\t$tr" >> "$SUMMARY"
 echo "rc=$rc states=${st:-NA} dP=${dp:-NA} mm"
 [[ $rc -eq 0 ]] || { tail -60 "$log"; return "$rc"; }
}

echo '[4/6] Run 14-way causal matrix'
run CONTROL CONTROL -1
for i in 0 1 2 3 4 5; do run "DROP_F$i" DROP_ONE "$i"; done
run DROP_ALL DROP_ALL -1
for i in 0 1 2 3 4 5; do run "KEEP_ONLY_F$i" KEEP_ONE "$i"; done

echo '[5/6] Restore clean Kimera'
cp "$BAK" "$SRC"; cmake --build "$K/build" -j2 --target kimera_vio; CLEAN=1; trap - EXIT

echo '[6/6] Report'
echo
echo '================ FACTOR IDENTITIES ================'
grep '\[JT15.35 F\]' "$OUT/CONTROL.txt" || true
echo
echo '================ V15.35 CAUSAL MATRIX ================'
column -t -s $'\t' "$SUMMARY" 2>/dev/null || cat "$SUMMARY"
base=$(awk -F'\t' '$1=="CONTROL"{print $4}' "$SUMMARY")
echo
echo '================ EFFECT VS CONTROL ================'
awk -F'\t' -v b="$base" 'NR>1 && $4!="NA"{d=$4-b; printf "%-16s final_dP=%9.3f mm  delta=%+9.3f mm  ratio=%7.3f\n",$1,$4,d,(b!=0?$4/b:0)}' "$SUMMARY"
echo
echo "Full logs: $OUT"
echo 'Interpretation: a DROP_ONE that switches H28 from ~228 mm toward ~29 mm identifies a causal track candidate; DROP_ALL-only improvement indicates a group/conditioning effect; no meaningful ablation effect points back to marginal-prior timing rather than these six tracks themselves.'
