#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
OUT=/home/vio/jtzero_single_boundary_delay_v15_27
IMU="${1:?imu csv}"; CAM="${2:?camera csv}"; MJPG="${3:?mjpg}"
SRC="$KIMERA/src/backend/VioBackend.cpp"; BAK=/tmp/VioBackend.cpp.v15_27.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_27
mkdir -p "$OUT"

if ! git -C "$KIMERA" diff --quiet -- src/backend/VioBackend.cpp; then
  echo 'ERROR: dirty VioBackend.cpp' >&2
  exit 1
fi
cp "$SRC" "$BAK"
CLEAN_REBUILT=0
cleanup(){
  cp "$BAK" "$SRC" 2>/dev/null || true
  if [[ "$CLEAN_REBUILT" -eq 0 ]]; then
    cmake --build "$KIMERA/build" -j2 --target kimera_vio || true
  fi
}
trap cleanup EXIT

build_replay(){
  g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
    -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" \
    -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
    $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
    -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}

run_case(){
  local name="$1" target="$2"
  local P="$OUT/params_$name" LOG="$OUT/$name.txt"
  rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
  sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  JT15_27_TARGET="$target" LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$LOG" 2>&1
  rc=$?
  set -e
  [[ $rc -eq 0 ]] || { echo "ERROR $name rc=$rc"; tail -100 "$LOG"; exit "$rc"; }
  local dp states path exc yaw trace
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG")
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
  path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$LOG")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$LOG")
  yaw=$(awk -F': ' '/^orientation span deg/{gsub(/[\[\]]/,"",$2); split($2,a," "); print a[3]; exit}' "$LOG")
  trace=$(grep -c '\[JT15.27\]' "$LOG" || true)
  echo "$name target=$target states=${states:-NA} final_dP=${dp:-NA} path=${path:-NA} maxexc=${exc:-NA} yawspan=${yaw:-NA} trace=$trace"
}

echo '[1/6] Build clean Kimera + replay'
cmake --build "$KIMERA/build" -j2 --target kimera_vio
build_replay

echo '[2/6] Clean H28 control'
run_case H28_CONTROL -1

echo '[3/6] Inject single-boundary diagnostic'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
count=$(grep -Foc "$anchor" "$SRC" || true)
if [[ "$count" != 1 ]]; then
  echo "ERROR: expected exactly one update anchor, found $count" >&2
  grep -nF "$anchor" "$SRC" || true
  exit 2
fi

TMP=/tmp/VioBackend.cpp.v15_27.instrumented
awk -v anchor="$anchor" '
BEGIN { replaced=0 }
$0 == anchor && replaced == 0 {
  print "        ([&]() {"
  print "          std::map<Key, double> jt1527_timestamps = timestamps;"
  print "          int jt1527_target = -1;"
  print "          if (const char* e = std::getenv(\"JT15_27_TARGET\")) jt1527_target = std::atoi(e);"
  print "          if (static_cast<size_t>(backend_params_.nr_states_) == 28 && curr_kf_id_ >= 87) {"
  print "            const size_t boundary_idx = static_cast<size_t>(curr_kf_id_) - 29;"
  print "            if (jt1527_target >= 0 && boundary_idx == static_cast<size_t>(jt1527_target)) {"
  print "              const double refreshed_ts = static_cast<double>(boundary_idx + 1);"
  print "              jt1527_timestamps[gtsam::Symbol(\"x\"[0], boundary_idx)] = refreshed_ts;"
  print "              jt1527_timestamps[gtsam::Symbol(\"v\"[0], boundary_idx)] = refreshed_ts;"
  print "              jt1527_timestamps[gtsam::Symbol(\"b\"[0], boundary_idx)] = refreshed_ts;"
  print "              std::cerr << \"[JT15.27] target=x\" << boundary_idx"
  print "                        << \" kf=\" << curr_kf_id_"
  print "                        << \" refresh_ts=\" << refreshed_ts << std::endl;"
  print "            }"
  print "          }"
  print "          return smoother_->update(new_factors, new_values, jt1527_timestamps, delete_slots);"
  print "        })();"
  replaced=1
  next
}
{ print }
END { if (replaced != 1) exit 7 }
' "$SRC" > "$TMP"
mv "$TMP" "$SRC"

# add only the standard headers needed by the diagnostic, once
sed -i '/#include <map>/a #include <cstdlib>\n#include <iostream>' "$SRC"

if ! grep -q '\[JT15.27\]' "$SRC"; then
  echo 'ERROR: instrumentation insertion failed' >&2
  exit 3
fi
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo '[4/6] Single-boundary causal sweep'
run_case DELAY_X58_ONLY 58
run_case DELAY_X59_ONLY 59
run_case DELAY_X60_ONLY 60

echo '[5/6] Restore clean Kimera'
cp "$BAK" "$SRC"
cmake --build "$KIMERA/build" -j2 --target kimera_vio
CLEAN_REBUILT=1
trap - EXIT

echo '[6/6] Results'
echo
echo '================ RESULTS ================='
for name in H28_CONTROL DELAY_X58_ONLY DELAY_X59_ONLY DELAY_X60_ONLY; do
  echo "--- $name ---"
  grep -E '^backend states:|^final \|dP\| mm:|^path length mm:|^max excursion mm:|^orientation span deg|^final dRPY deg:' "$OUT/$name.txt" || true
  grep '\[JT15.27\]' "$OUT/$name.txt" || true
done

echo
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
