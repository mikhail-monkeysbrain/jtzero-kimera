#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$K/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_36.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_36
OUT=/home/vio/jtzero_x58_fixedslot_v15_36
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
  $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}
P="$OUT/params_H28"; rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
metric(){ awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$1"; }
run_case(){
 local name="$1"
 local mode="$2"
 local slot="$3"
 local log="$OUT/$name.txt"
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 set +e
 JT1536_MODE="$mode" JT1536_SLOT="$slot" LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
  "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
 local rc=$?
 set -e
 local dp states exc
 dp=$(metric "$log"); states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log"); exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$log")
 printf '%-18s rc=%d states=%s final_dP=%s maxexc=%s\n' "$name" "$rc" "${states:-NA}" "${dp:-NA}" "${exc:-NA}"
 [[ $rc -eq 0 && -n "$dp" ]] || { tail -80 "$log"; exit 10; }
}

echo '============================================================'
echo 'JT-ZERO v15.36 ZERO-PERTURBATION GATE + FIXED-SLOT ABLATION'
echo 'No getFactors/typeid/sort/ostringstream. H28 yaw_only_v13.'
echo '============================================================'
echo '[1/7] Build CLEAN Kimera + replay'
cmake --build "$K/build" -j2 --target kimera_vio
build_replay

echo '[2/7] CLEAN H28 reference'
run_case CLEAN_H28 OFF -1
CLEAN_DP=$(metric "$OUT/CLEAN_H28.txt")

echo '[3/7] Inject minimal fixed-slot branch'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo 'FATAL update anchor not unique' >&2; exit 4; }
TMP=/tmp/VioBackend.cpp.v15_36.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          gtsam::FactorIndices jt1536_delete = delete_slots;";
 print "          const char* jt1536_mode = std::getenv(\"JT1536_MODE\");";
 print "          const char* jt1536_slot_s = std::getenv(\"JT1536_SLOT\");";
 print "          if (jt1536_mode && std::string(jt1536_mode) == \"DROP\" && curr_kf_id_ == 87) {";
 print "            const long s = jt1536_slot_s ? std::strtol(jt1536_slot_s,nullptr,10) : -1;";
 print "            if (s >= 0 && std::find(jt1536_delete.begin(),jt1536_delete.end(),static_cast<size_t>(s))==jt1536_delete.end()) jt1536_delete.push_back(static_cast<size_t>(s));";
 print "          }";
 print "          return smoother_->update(new_factors,new_values,timestamps,jt1536_delete);";
 print "        })();"; done=1; next
}
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"; mv "$TMP" "$SRC"
# stdlib is normally already transitively available; add cstdlib only if absent.
grep -q '^#include <cstdlib>' "$SRC" || sed -i '/#include <map>/a #include <cstdlib>' "$SRC"
cmake --build "$K/build" -j2 --target kimera_vio

echo '[4/7] Instrumented CONTROL gate'
run_case INSTR_CONTROL CONTROL -1
INSTR_DP=$(metric "$OUT/INSTR_CONTROL.txt")
python3 - "$CLEAN_DP" "$INSTR_DP" <<'PY'
import sys
c=float(sys.argv[1]); i=float(sys.argv[2]); d=abs(c-i)
print(f'GATE clean={c:.6f} instrumented={i:.6f} abs_delta={d:.6f} mm')
# deterministic replay should be essentially identical; allow only 0.01 mm formatting/numeric slack.
if d > 0.01:
    print('RESULT: INVALID_INSTRUMENTATION')
    raise SystemExit(21)
print('RESULT: CONTROL_GATE_PASS')
PY

echo '[5/7] Fixed-slot causal ablations'
# Slots are the four x58 SmartStereo slots observed at kf87 in v15.35.
# We intentionally do not inspect the graph in this build: inspection itself perturbed CONTROL.
run_case DROP_SLOT729 DROP 729
run_case DROP_SLOT735 DROP 735
run_case DROP_SLOT853 DROP 853
run_case DROP_SLOT854 DROP 854

echo '[6/7] Restore clean Kimera'
cp "$BAK" "$SRC"; cmake --build "$K/build" -j2 --target kimera_vio; CLEAN=1; trap - EXIT

echo '[7/7] Report'
echo
echo '================ V15.36 RESULTS ================'
for n in CLEAN_H28 INSTR_CONTROL DROP_SLOT729 DROP_SLOT735 DROP_SLOT853 DROP_SLOT854; do
 f="$OUT/$n.txt"; printf '%-18s dP=%-10s maxexc=%-10s\n' "$n" "$(metric "$f")" "$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$f")"
done
echo
echo '================ EFFECT VS INSTR_CONTROL ================'
B=$(metric "$OUT/INSTR_CONTROL.txt")
for n in DROP_SLOT729 DROP_SLOT735 DROP_SLOT853 DROP_SLOT854; do D=$(metric "$OUT/$n.txt"); awk -v n="$n" -v b="$B" -v d="$D" 'BEGIN{printf "%-18s dP=%9.3f mm delta=%+9.3f mm ratio=%7.3f\n",n,d,d-b,(b?d/b:0)}'; done
echo
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
