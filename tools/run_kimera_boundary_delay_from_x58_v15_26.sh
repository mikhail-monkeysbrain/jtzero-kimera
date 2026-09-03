#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
OUT=/home/vio/jtzero_boundary_delay_from_x58_v15_26
IMU="${1:?imu csv}"; CAM="${2:?camera csv}"; MJPG="${3:?mjpg}"
SRC="$KIMERA/src/backend/VioBackend.cpp"; BAK=/tmp/VioBackend.cpp.v15_26.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_26
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
  local name="$1" H="$2"
  local P="$OUT/params_$name" LOG="$OUT/$name.txt"
  rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
  sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $H/" "$P/BackendParams.yaml"
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$LOG" 2>&1
  rc=$?
  set -e
  [[ $rc -eq 0 ]] || { echo "ERROR $name rc=$rc"; tail -100 "$LOG"; exit "$rc"; }
  local dp states path exc yaw
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG")
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
  path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$LOG")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$LOG")
  yaw=$(awk -F': ' '/^orientation span deg/{gsub(/[\[\]]/,"",$2); split($2,a," "); print a[3]; exit}' "$LOG")
  echo "$name H=$H states=${states:-NA} final_dP=${dp:-NA} path=${path:-NA} maxexc=${exc:-NA} yawspan=${yaw:-NA} trace=$(grep -c '\[JT15.26\]' "$LOG" || true)"
}

echo '[1/6] Build clean Kimera + replay'
cmake --build "$KIMERA/build" -j2 --target kimera_vio
build_replay

echo '[2/6] Clean H28 control'
run_case H28_CONTROL 28

echo '[3/6] Inject context-independent one-state boundary delay'
python_anchor='smoother_->update(new_factors, new_values, timestamps, delete_slots);'
count=$(grep -Foc "$python_anchor" "$SRC" || true)
if [[ "$count" != 1 ]]; then
  echo "ERROR: expected exactly one update anchor, found $count" >&2
  grep -nF "$python_anchor" "$SRC" || true
  exit 2
fi
perl -0pi -e 's@\Q        smoother_->update(new_factors, new_values, timestamps, delete_slots);\E@        ([&]() {\n          std::map<Key, double> jt1526_timestamps = timestamps;\n          if (static_cast<size_t>(backend_params_.nr_states_) == 28 && curr_kf_id_ >= 87) {\n            const size_t boundary_idx = static_cast<size_t>(curr_kf_id_) - 29;\n            if (boundary_idx >= 58) {\n              const double refreshed_ts = static_cast<double>(boundary_idx + 1);\n              jt1526_timestamps[gtsam::Symbol('\''x'\'', boundary_idx)] = refreshed_ts;\n              jt1526_timestamps[gtsam::Symbol('\''v'\'', boundary_idx)] = refreshed_ts;\n              jt1526_timestamps[gtsam::Symbol('\''b'\'', boundary_idx)] = refreshed_ts;\n              LOG(WARNING) << "[JT15.26] kf=" << curr_kf_id_\n                           << " refresh={b" << boundary_idx\n                           << ",v" << boundary_idx\n                           << ",x" << boundary_idx\n                           << "}@" << refreshed_ts;\n            }\n          }\n          return smoother_->update(new_factors, new_values, jt1526_timestamps, delete_slots);\n        })();@' "$SRC"

if ! grep -q '\[JT15.26\]' "$SRC"; then
  echo 'ERROR: instrumentation insertion failed' >&2
  exit 3
fi
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo '[4/6] Delayed H28 causal run'
run_case H28_DELAY_FROM_X58 28

echo '[5/6] Restore clean Kimera'
cp "$BAK" "$SRC"
cmake --build "$KIMERA/build" -j2 --target kimera_vio
CLEAN_REBUILT=1
trap - EXIT

echo '[6/6] Clean H29 control'
run_case H29_CONTROL 29

echo
echo '================ RESULTS ================='
for name in H28_CONTROL H28_DELAY_FROM_X58 H29_CONTROL; do
  echo "--- $name ---"
  grep -E '^backend states:|^final \|dP\| mm:|^path length mm:|^max excursion mm:|^orientation span deg|^final dRPY deg:' "$OUT/$name.txt" || true
done

echo
echo '================ DELAY TRACE (first/last) ================='
grep '\[JT15.26\]' "$OUT/H28_DELAY_FROM_X58.txt" | head -10 || true
echo '...'
grep '\[JT15.26\]' "$OUT/H28_DELAY_FROM_X58.txt" | tail -10 || true

echo
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
