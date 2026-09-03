#!/usr/bin/env bash
set -euo pipefail
JTROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KROOT=/home/vio/Kimera-VIO
PATCH="$JTROOT/tools/kimera_v15_23_boundary_factor_trace.patch"
BASE="$JTROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUT=/home/vio/jtzero_boundary_factor_trace_v15_23
BIN=/tmp/replay_boundary_factor_trace_v15_23
SRC="$KROOT/src/backend/VioBackend.cpp"
BACKUP=/tmp/VioBackend.cpp.v15_23_backup
mkdir -p "$OUT"
restore_source(){ [[ -f "$BACKUP" ]] && cp "$BACKUP" "$SRC" || true; }
trap restore_source EXIT

cd "$KROOT"
if ! git diff --quiet -- src/backend/VioBackend.cpp; then
  echo '[FATAL] VioBackend.cpp already modified.' >&2; exit 2
fi
cp "$SRC" "$BACKUP"
echo '[1/5] Apply v15.23 patch'
git apply --recount --check "$PATCH"
git apply --recount "$PATCH"
echo '[2/5] Build instrumented Kimera'
cmake --build build -j2 --target kimera_vio

echo '[3/5] Build replay'
cd "$JTROOT"
g++ -std=c++17 -O2 -I"$JTROOT/tools" -I/home/vio/Kimera-VIO/include -I/home/vio/Kimera-VIO/build -I/home/vio/Kimera-VIO/third_party/mavlink -I/usr/include/eigen3 \
 "$JTROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4) \
 -L/home/vio/Kimera-VIO/build -L/usr/local/lib -lkimera_vio -lgtsam -lglog -lgflags -lpthread

replace_nr(){ sed -E -i "s|^([[:space:]]*nr_states:[[:space:]]*).*|\\1$2|" "$1/BackendParams.yaml"; }
echo '[4/5] Replay H28/H29/H30'
for H in 28 29 30; do
  P="/tmp/jtzero_v15_23_H$H"; rm -rf "$P"; cp -a "$BASE" "$P"; replace_nr "$P" "$H"
  LOG="$OUT/H${H}.txt"
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN" "$P" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" >"$LOG" 2>&1
  grep '\[JT15\.23' "$LOG" > "$OUT/H${H}_trace.txt" || true
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2}' "$LOG" | tail -1)
  echo "H$H final_dP=${dp:-?} mm trace_lines=$(wc -l < "$OUT/H${H}_trace.txt")"
done

echo '[5/5] Restore and rebuild clean Kimera'
restore_source; rm -f "$BACKUP"; trap - EXIT
cd "$KROOT"; cmake --build build -j2 --target kimera_vio

echo
echo '================ SUMMARIES ================='
for H in 28 29 30; do echo "--- H$H ---"; grep '\[JT15\.23 SUMMARY\]' "$OUT/H${H}_trace.txt" || true; done
echo
echo '================ FACTORS AT kf=88 ================='
for H in 28 29 30; do echo "--- H$H ---"; grep '\[JT15\.23 FACTOR\] kf=88 ' "$OUT/H${H}_trace.txt" || true; done
echo
echo "Full traces: $OUT"
echo 'RESULT: COMPLETE'
