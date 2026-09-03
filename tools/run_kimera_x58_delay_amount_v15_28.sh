#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
OUT=/home/vio/jtzero_x58_delay_amount_v15_28
IMU="${1:?imu csv}"; CAM="${2:?camera csv}"; MJPG="${3:?mjpg}"
SRC="$KIMERA/src/backend/VioBackend.cpp"; BAK=/tmp/VioBackend.cpp.v15_28.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_28
mkdir -p "$OUT"

if ! git -C "$KIMERA" diff --quiet -- src/backend/VioBackend.cpp; then echo 'ERROR: dirty VioBackend.cpp' >&2; exit 1; fi
cp "$SRC" "$BAK"
CLEAN_REBUILT=0
cleanup(){ cp "$BAK" "$SRC" 2>/dev/null || true; if [[ "$CLEAN_REBUILT" -eq 0 ]]; then cmake --build "$KIMERA/build" -j2 --target kimera_vio || true; fi; }
trap cleanup EXIT

build_replay(){
 g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}
run_case(){
 local name="$1" mode="$2" value="$3" P="$OUT/params_$1" LOG="$OUT/$1.txt"
 rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"; sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 JT1528_MODE="$mode" JT1528_VALUE="$value" LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$LOG" 2>&1
 local dp states path exc yaw
 dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG"); states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
 path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$LOG"); exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$LOG")
 yaw=$(awk -F': ' '/^orientation span deg/{gsub(/[\[\]]/,"",$2);split($2,a," ");print a[3];exit}' "$LOG")
 echo "$name mode=$mode value=$value states=$states final_dP=$dp path=$path maxexc=$exc yawspan=$yaw trace=$(grep -c '\[JT15.28\]' "$LOG" || true)"
}

echo '[1/5] Build clean Kimera + replay'; cmake --build "$KIMERA/build" -j2 --target kimera_vio; build_replay
echo '[2/5] H28 control'; run_case H28_CONTROL OFF 0

echo '[3/5] Inject x58 timestamp sweep instrumentation'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo 'ERROR: update anchor not unique' >&2; exit 2; }
TMP=/tmp/VioBackend.cpp.v15_28.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          std::map<Key, double> jt1528_timestamps = timestamps;";
 print "          const char* jt1528_mode = std::getenv(\"JT1528_MODE\");";
 print "          const char* jt1528_val_s = std::getenv(\"JT1528_VALUE\");";
 print "          if (jt1528_mode && std::string(jt1528_mode) != \"OFF\" && curr_kf_id_ == 87) {";
 print "            const double v = jt1528_val_s ? std::atof(jt1528_val_s) : 59.0;";
 print "            const size_t idx = 58;";
 print "            jt1528_timestamps[gtsam::Symbol(\"x\"[0],idx)] = v;";
 print "            jt1528_timestamps[gtsam::Symbol(\"v\"[0],idx)] = v;";
 print "            jt1528_timestamps[gtsam::Symbol(\"b\"[0],idx)] = v;";
 print "            std::cerr << \"[JT15.28] kf=\" << curr_kf_id_ << \" x58_ts=\" << v << std::endl;";
 print "          }";
 print "          return smoother_->update(new_factors,new_values,jt1528_timestamps,delete_slots);";
 print "        })();"; done=1; next }
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"; mv "$TMP" "$SRC"
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo '[4/5] x58 timestamp amount sweep'
# Original x58 timestamp is 58. Values below/above 59 probe whether the branch switch is a threshold or merely instrumentation perturbation.
for spec in 'TS58_25:58.25' 'TS58_50:58.50' 'TS58_75:58.75' 'TS58_90:58.90' 'TS58_99:58.99' 'TS59_00:59.00' 'TS59_01:59.01' 'TS59_10:59.10' 'TS59_50:59.50' 'TS60_00:60.00'; do
 name=${spec%%:*}; val=${spec#*:}; run_case "$name" SET "$val"
done

echo '[5/5] Restore clean Kimera'; cp "$BAK" "$SRC"; cmake --build "$KIMERA/build" -j2 --target kimera_vio; CLEAN_REBUILT=1; trap - EXIT

echo; echo '================ RESULTS ================='
for f in "$OUT"/*.txt; do n=$(basename "$f" .txt); dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$f"); path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$f"); exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$f"); tr=$(grep '\[JT15.28\]' "$f" | head -1 || true); printf '%-14s dP=%-10s path=%-10s maxexc=%-10s %s\n' "$n" "${dp:-NA}" "${path:-NA}" "${exc:-NA}" "$tr"; done | sort

echo; echo "Full logs: $OUT"; echo 'RESULT: COMPLETE'
