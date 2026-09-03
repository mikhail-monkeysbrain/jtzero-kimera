#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA=/home/vio/Kimera-VIO
PATCH="$ROOT/tools/kimera_v15_24_boundary_smartfactor_stats.patch"
OUT=/home/vio/jtzero_boundary_smartfactor_stats_v15_24
IMU="${1:?imu csv}"
CAM="${2:?camera csv}"
MJPG="${3:?mjpg}"
SRC="$KIMERA/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_24.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_24

mkdir -p "$OUT"
if ! git -C "$KIMERA" diff --quiet -- src/backend/VioBackend.cpp; then
  echo "ERROR: VioBackend.cpp has local changes; refusing to overwrite." >&2
  exit 1
fi
cp "$SRC" "$BAK"
restore() {
  cp "$BAK" "$SRC" 2>/dev/null || true
}
trap restore EXIT

echo "[1/5] Apply v15.24 patch"
git -C "$KIMERA" apply --recount "$PATCH"
echo "[2/5] Build instrumented Kimera"
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo "[3/5] Build replay"
g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)

echo "[4/5] Replay H28/H29/H30"
for H in 28 29 30; do
  P="$OUT/params_H$H"
  rm -rf "$P"
  cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
  sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $H/" "$P/BackendParams.yaml"
  LOG="$OUT/H$H.txt"
  CSV="$OUT/H$H.csv"
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$IMU" "$CAM" "$MJPG" "$P" "$CSV" >"$LOG" 2>&1
  dp=$(grep -E 'Final dP|\|dP\|' "$LOG" | tail -1 || true)
  echo "H$H $dp trace_lines=$(grep -c '\[JT15.24' "$LOG" || true)"
done

echo "[5/5] Restore and rebuild clean Kimera"
restore
trap - EXIT
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo
echo "================ SMART SUMMARY kf=86..90 ================="
for H in 28 29 30; do
  echo "--- H$H ---"
  grep '\[JT15.24 SUMMARY\]' "$OUT/H$H.txt" || true
done

echo
echo "================ SMART FACTORS AT kf=88 =================="
for H in 28 29 30; do
  echo "--- H$H ---"
  grep '\[JT15.24 SMART\] kf=88 ' "$OUT/H$H.txt" || true
done

echo
echo "Full traces: $OUT"
echo "RESULT: COMPLETE"
