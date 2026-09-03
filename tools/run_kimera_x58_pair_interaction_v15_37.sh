#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$K/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_37.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_37
OUT=/home/vio/jtzero_x58_pair_v15_37
IMU="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAM="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
mkdir -p "$OUT"
for f in "$IMU" "$CAM" "$MJPG"; do [[ -s "$f" ]] || { echo "FATAL missing $f" >&2; exit 2; }; done
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then echo 'FATAL dirty VioBackend.cpp' >&2; exit 3; fi
cp "$SRC" "$BAK"
CLEAN=0
cleanup(){ cp "$BAK" "$SRC" 2>/dev/null || true; if [[ "$CLEAN" -eq 0 ]]; then cmake --build "$K/build" -j2 --target kimera_vio || true; fi; }
trap cleanup EXIT
build_replay(){
 g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}
P="$OUT/params_H28"; rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"; sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
metric(){ awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$1"; }
run_case(){
 local name="$1"; local mode="$2"; local log="$OUT/$name.txt"
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 set +e
 JT1537_MODE="$mode" LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
 local rc=$?
 set -e
 local dp states exc
 dp=$(metric "$log"); states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log"); exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$log")
 printf '%-22s rc=%d states=%s final_dP=%s maxexc=%s\n' "$name" "$rc" "${states:-NA}" "${dp:-NA}" "${exc:-NA}"
 # A shortened trajectory is evidence for DROP853, not a runner failure. Only require process success and a metric.
 [[ $rc -eq 0 && -n "$dp" ]] || { tail -100 "$log"; exit 10; }
}

echo '============================================================'
echo 'JT-ZERO v15.37 x58 SMARTSTEREO PAIR x MARGINALIZATION'
echo 'H28, existing yaw_only_v13, zero-perturbation control gate.'
echo '============================================================'
echo '[1/7] Build clean Kimera + replay'; cmake --build "$K/build" -j2 --target kimera_vio; build_replay
echo '[2/7] CLEAN H28'; run_case CLEAN_H28 OFF; CLEAN_DP=$(metric "$OUT/CLEAN_H28.txt")

echo '[3/7] Inject minimal fixed-slot + x58-delay branch'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo 'FATAL update anchor not unique' >&2; exit 4; }
TMP=/tmp/VioBackend.cpp.v15_37.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          gtsam::FactorIndices jt1537_delete = delete_slots;";
 print "          std::map<Key,double> jt1537_ts = timestamps;";
 print "          const char* e = std::getenv(\"JT1537_MODE\"); const std::string m = e ? e : \"CONTROL\";";
 print "          if (curr_kf_id_ == 87) {";
 print "            const bool d853 = (m==\"DROP853\" || m==\"DROP_BOTH\" || m==\"DELAY_DROP853\");";
 print "            const bool d854 = (m==\"DROP854\" || m==\"DROP_BOTH\" || m==\"DELAY_DROP854\");";
 print "            const bool delay = (m==\"DELAY\" || m==\"DELAY_DROP853\" || m==\"DELAY_DROP854\");";
 print "            if (d853 && std::find(jt1537_delete.begin(),jt1537_delete.end(),static_cast<size_t>(853))==jt1537_delete.end()) jt1537_delete.push_back(853);";
 print "            if (d854 && std::find(jt1537_delete.begin(),jt1537_delete.end(),static_cast<size_t>(854))==jt1537_delete.end()) jt1537_delete.push_back(854);";
 print "            if (delay) { const size_t i=58; const double t=59.0; jt1537_ts[gtsam::Symbol(\"x\"[0],i)]=t; jt1537_ts[gtsam::Symbol(\"v\"[0],i)]=t; jt1537_ts[gtsam::Symbol(\"b\"[0],i)]=t; }";
 print "          }";
 print "          return smoother_->update(new_factors,new_values,jt1537_ts,jt1537_delete);";
 print "        })();"; done=1; next }
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"; mv "$TMP" "$SRC"
grep -q '^#include <cstdlib>' "$SRC" || sed -i '/#include <map>/a #include <cstdlib>' "$SRC"
cmake --build "$K/build" -j2 --target kimera_vio

echo '[4/7] Instrumented CONTROL gate'; run_case INSTR_CONTROL CONTROL; INSTR_DP=$(metric "$OUT/INSTR_CONTROL.txt")
python3 - "$CLEAN_DP" "$INSTR_DP" <<'PY'
import sys
c=float(sys.argv[1]); i=float(sys.argv[2]); d=abs(c-i)
print(f'GATE clean={c:.6f} instrumented={i:.6f} abs_delta={d:.6f} mm')
if d>0.01:
 print('RESULT: INVALID_INSTRUMENTATION'); raise SystemExit(21)
print('RESULT: CONTROL_GATE_PASS')
PY

echo '[5/7] Pair / timing interaction matrix'
run_case DELAY_X58 DELAY
run_case DROP853 DROP853
run_case DROP854 DROP854
run_case DROP853_854 DROP_BOTH
run_case DELAY_X58_DROP853 DELAY_DROP853
run_case DELAY_X58_DROP854 DELAY_DROP854

echo '[6/7] Restore clean Kimera'; cp "$BAK" "$SRC"; cmake --build "$K/build" -j2 --target kimera_vio; CLEAN=1; trap - EXIT

echo '[7/7] Report'; echo
echo '================ V15.37 RESULTS ================'
for n in CLEAN_H28 INSTR_CONTROL DELAY_X58 DROP853 DROP854 DROP853_854 DELAY_X58_DROP853 DELAY_X58_DROP854; do
 f="$OUT/$n.txt"; printf '%-22s states=%-5s dP=%-10s maxexc=%-10s\n' "$n" "$(awk -F': ' '/^backend states:/{print $2;exit}' "$f")" "$(metric "$f")" "$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$f")"
done
echo
echo '================ INTERACTION DELTAS ================'
B=$(metric "$OUT/INSTR_CONTROL.txt"); D=$(metric "$OUT/DELAY_X58.txt")
for n in DROP853 DROP854 DROP853_854; do X=$(metric "$OUT/$n.txt"); awk -v n="$n" -v b="$B" -v x="$X" 'BEGIN{printf "%-22s vs CONTROL  delta=%+10.3f mm ratio=%8.3f\n",n,x-b,(b?x/b:0)}'; done
for n in DELAY_X58_DROP853 DELAY_X58_DROP854; do X=$(metric "$OUT/$n.txt"); awk -v n="$n" -v d="$D" -v x="$X" 'BEGIN{printf "%-22s vs DELAY_X58 delta=%+10.3f mm ratio=%8.3f\n",n,x-d,(d?x/d:0)}'; done
echo
echo '================ STOP DIAGNOSTIC FOR DROP853 ================'
grep -E 'ERROR|FATAL|exception|Exception|Cheirality|Indetermin|singular|Singular|failed|Failed|failure|Failure|backend states:|final \|dP\|' "$OUT/DROP853.txt" | tail -80 || true
echo
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
