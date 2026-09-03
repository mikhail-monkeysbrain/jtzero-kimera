#!/usr/bin/env bash
set -euo pipefail

JTROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KROOT=/home/vio/Kimera-VIO
PATCH="$JTROOT/tools/kimera_v15_22b_call_timestamp_trace.patch"
BASE="$JTROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUT=/home/vio/jtzero_call_timestamp_trace_v15_22b
BIN=/tmp/replay_call_timestamp_trace_v15_22b
SRC="$KROOT/src/backend/VioBackend.cpp"
BACKUP=/tmp/VioBackend.cpp.v15_22b_backup
mkdir -p "$OUT"

restore_source() {
  if [[ -f "$BACKUP" ]]; then cp "$BACKUP" "$SRC"; fi
}
trap restore_source EXIT

echo '============================================================'
echo 'JT-ZERO MINIMAL SMOOTHER CALL/TIMESTAMP TRACE v15.22b'
echo '============================================================'

cd "$KROOT"
if ! git diff --quiet -- src/backend/VioBackend.cpp; then
  echo '[FATAL] src/backend/VioBackend.cpp already has local modifications.' >&2
  exit 2
fi
cp "$SRC" "$BACKUP"

echo '[1/5] Apply minimal trace patch'
git apply --recount --check "$PATCH"
git apply --recount "$PATCH"

echo '[2/5] Rebuild instrumented Kimera-VIO'
cmake --build build -j2 --target kimera_vio

echo '[3/5] Build replay'
cd "$JTROOT"
g++ -std=c++17 -O2 \
  -I"$JTROOT/tools" \
  -I/home/vio/Kimera-VIO/include \
  -I/home/vio/Kimera-VIO/build \
  -I/home/vio/Kimera-VIO/third_party/mavlink \
  -I/usr/include/eigen3 \
  "$JTROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4) \
  -L/home/vio/Kimera-VIO/build -L/usr/local/lib \
  -lkimera_vio -lgtsam -lglog -lgflags -lpthread

replace_nr_states() {
  local file="$1" value="$2" count
  count=$(grep -Ec '^[[:space:]]*nr_states:' "$file" || true)
  [[ "$count" == 1 ]] || { echo "[FATAL] nr_states count=$count in $file" >&2; exit 3; }
  sed -E -i "s|^([[:space:]]*nr_states:[[:space:]]*).*|\\1${value}|" "$file"
}

echo '[4/5] Replay H29 only'
P=/tmp/jtzero_v15_22b_H29
rm -rf "$P"
cp -a "$BASE" "$P"
replace_nr_states "$P/BackendParams.yaml" 29
LOG="$OUT/H29.txt"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$BIN" "$P" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" > "$LOG" 2>&1
grep '\[JT15\.22B\]' "$LOG" > "$OUT/H29_trace.txt" || true
TRACE_LINES=$(wc -l < "$OUT/H29_trace.txt")
echo "trace_lines=$TRACE_LINES"
awk -F': ' '/^final \|dP\| mm:/{dp=$2} /^backend states:/{st=$2} END{printf "states=%s final_dP=%s mm\n",st,dp}' "$LOG"

echo '[5/5] Restore source and rebuild clean Kimera-VIO'
restore_source
rm -f "$BACKUP"
trap - EXIT
cd "$KROOT"
cmake --build build -j2 --target kimera_vio

echo
echo '================ FIRST 20 CALLS ================================'
head -20 "$OUT/H29_trace.txt" || true
echo
echo '================ CALLS 65..105 ================================'
sed -n '65,105p' "$OUT/H29_trace.txt" || true
echo
echo '================ LAST 20 CALLS ================================'
tail -20 "$OUT/H29_trace.txt" || true
echo
echo "Full log: $LOG"
echo "Trace: $OUT/H29_trace.txt"
echo 'Kimera source restored and clean library rebuilt.'
if [[ "$TRACE_LINES" -gt 0 ]]; then
  echo 'RESULT: COMPLETE'
else
  echo 'RESULT: NO_TRACE'
  exit 4
fi
