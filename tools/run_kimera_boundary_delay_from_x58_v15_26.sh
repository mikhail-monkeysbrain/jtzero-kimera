#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
PATCH="$ROOT/tools/kimera_v15_26_boundary_delay_from_x58.patch"
OUT=/home/vio/jtzero_boundary_delay_from_x58_v15_26
IMU="${1:?imu csv}"; CAM="${2:?camera csv}"; MJPG="${3:?mjpg}"
SRC="$KIMERA/src/backend/VioBackend.cpp"; BAK=/tmp/VioBackend.cpp.v15_26.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_26
mkdir -p "$OUT"
if ! git -C "$KIMERA" diff --quiet -- src/backend/VioBackend.cpp; then echo 'ERROR: dirty VioBackend.cpp' >&2; exit 1; fi
cp "$SRC" "$BAK"
CLEAN_REBUILT=0
cleanup(){ cp "$BAK" "$SRC" 2>/dev/null || true; if [[ "$CLEAN_REBUILT" -eq 0 ]]; then cmake --build "$KIMERA/build" -j2 --target kimera_vio || true; fi; }
trap cleanup EXIT

echo '[1/5] Apply v15.26 patch'; git -C "$KIMERA" apply --recount "$PATCH"
echo '[2/5] Build instrumented Kimera'; cmake --build "$KIMERA/build" -j2 --target kimera_vio
echo '[3/5] Build replay'; g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)

run_case(){
  local name="$1" H="$2" delay="$3"
  local P="$OUT/params_$name" LOG="$OUT/$name.txt"
  rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
  sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $H/" "$P/BackendParams.yaml"
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  JT15_26_DELAY_FROM_X58="$delay" LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$LOG" 2>&1
  rc=$?
  set -e
  [[ $rc -eq 0 ]] || { echo "ERROR $name rc=$rc"; tail -100 "$LOG"; exit "$rc"; }
  local dp states path exc yaw
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG")
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
  path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$LOG")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$LOG")
  yaw=$(awk -F': ' '/^orientation span deg/{gsub(/[\[\]]/,"",$2); split($2,a," "); print a[3]; exit}' "$LOG")
  echo "$name H=$H delay=$delay states=${states:-NA} final_dP=${dp:-NA} path=${path:-NA} maxexc=${exc:-NA} yawspan=${yaw:-NA} trace=$(grep -c '\[JT15.26\]' "$LOG" || true)"
}

echo '[4/5] Causal A/B'
run_case H28_CONTROL 28 0
run_case H28_DELAY_FROM_X58 28 1
run_case H29_CONTROL 29 0

echo '[5/5] Restore and rebuild clean Kimera'; cp "$BAK" "$SRC"; cmake --build "$KIMERA/build" -j2 --target kimera_vio; CLEAN_REBUILT=1; trap - EXIT

echo; echo '================ RESULTS ================='
for name in H28_CONTROL H28_DELAY_FROM_X58 H29_CONTROL; do
  echo "--- $name ---"
  grep -E '^backend states:|^final \|dP\| mm:|^path length mm:|^max excursion mm:|^orientation span deg|^final dRPY deg:' "$OUT/$name.txt" || true
done

echo; echo '================ DELAY TRACE (first/last) ================='
grep '\[JT15.26\]' "$OUT/H28_DELAY_FROM_X58.txt" | head -10 || true
echo '...'
grep '\[JT15.26\]' "$OUT/H28_DELAY_FROM_X58.txt" | tail -10 || true

echo; echo "Full logs: $OUT"; echo 'RESULT: COMPLETE'
