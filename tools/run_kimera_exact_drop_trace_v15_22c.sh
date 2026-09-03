#!/usr/bin/env bash
set -euo pipefail

JTROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KROOT=/home/vio/Kimera-VIO
PATCH="$JTROOT/tools/kimera_v15_22c_exact_drop_trace.patch"
BASE="$JTROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUT=/home/vio/jtzero_exact_drop_trace_v15_22c
BIN=/tmp/replay_exact_drop_trace_v15_22c
SRC="$KROOT/src/backend/VioBackend.cpp"
BACKUP=/tmp/VioBackend.cpp.v15_22c_backup
mkdir -p "$OUT"

restore_source() {
  if [[ -f "$BACKUP" ]]; then cp "$BACKUP" "$SRC"; fi
}
trap restore_source EXIT

echo '============================================================'
echo 'JT-ZERO EXACT FIXED-LAG DROP TRACE v15.22c'
echo '============================================================'

cd "$KROOT"
if ! git diff --quiet -- src/backend/VioBackend.cpp; then
  echo '[FATAL] src/backend/VioBackend.cpp already has local modifications.' >&2
  exit 2
fi
cp "$SRC" "$BACKUP"

echo '[1/5] Apply exact drop trace patch'
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

echo '[4/5] Replay H28/H29/H30'
for n in 28 29 30; do
  P=/tmp/jtzero_v15_22c_H${n}
  rm -rf "$P"
  cp -a "$BASE" "$P"
  replace_nr_states "$P/BackendParams.yaml" "$n"
  LOG="$OUT/H${n}.txt"
  echo "[RUN] H$n"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$P" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" > "$LOG" 2>&1
  grep '\[JT15\.22C ' "$LOG" > "$OUT/H${n}_trace.txt" || true
  awk -F': ' '/^final \|dP\| mm:/{dp=$2} /^backend states:/{st=$2} END{printf "  states=%s final_dP=%s mm trace_lines=",st,dp}' "$LOG"
  wc -l < "$OUT/H${n}_trace.txt"
done

echo '[5/5] Restore source and rebuild clean Kimera-VIO'
restore_source
rm -f "$BACKUP"
trap - EXIT
cd "$KROOT"
cmake --build build -j2 --target kimera_vio

echo
echo '================ EXACT DROPS kf=86..90 ========================'
for n in 28 29 30; do
  echo "--- H$n ---"
  grep '\[JT15\.22C DROP\]' "$OUT/H${n}_trace.txt" || true
done

echo
echo '================ FULL TRACE kf=86..90 ========================='
for n in 28 29 30; do
  echo "--- H$n ---"
  cat "$OUT/H${n}_trace.txt"
done

echo
echo "Full logs: $OUT"
echo 'Kimera source restored and clean library rebuilt.'
echo 'RESULT: COMPLETE'
