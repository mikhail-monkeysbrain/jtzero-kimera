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
CLEAN_REBUILT=0
restore_and_rebuild() {
  cp "$BAK" "$SRC" 2>/dev/null || true
  if [[ "$CLEAN_REBUILT" -eq 0 ]]; then
    echo "[cleanup] Rebuild clean Kimera"
    cmake --build "$KIMERA/build" -j2 --target kimera_vio || true
  fi
}
trap restore_and_rebuild EXIT

echo "[1/5] Apply v15.24 patch"
git -C "$KIMERA" apply --recount "$PATCH"
echo "[2/5] Build instrumented Kimera"
cmake --build "$KIMERA/build" -j2 --target kimera_vio

echo "[3/5] Build replay"
g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
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
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$LOG" 2>&1
  rc=$?
  set -e
  if [[ $rc -ne 0 ]]; then
    echo "ERROR: H$H replay failed rc=$rc"
    tail -80 "$LOG" || true
    exit "$rc"
  fi
  if [[ -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then
    cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$CSV"
  fi
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$LOG")
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$LOG")
  echo "H$H states=${states:-NA} final_dP=${dp:-NA} mm trace_lines=$(grep -c '\[JT15.24' "$LOG" || true)"
done

echo "[5/5] Restore and rebuild clean Kimera"
cp "$BAK" "$SRC"
cmake --build "$KIMERA/build" -j2 --target kimera_vio
CLEAN_REBUILT=1
trap - EXIT

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
