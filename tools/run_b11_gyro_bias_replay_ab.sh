#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
SRC="${ROOT}/tools/replay_mono_imu_extrinsics_ab_v10.cpp"
PARAMS="${ROOT}/params/JTZeroMonoFLU"

BASE="/home/vio/jtzero_yaw_only_v15_42"
SUMMARY="/home/vio/p11_b11_gyro_bias_replay_ab.txt"

MAVLINK="/home/vio/Kimera-VIO/third_party/mavlink"

export LD_LIBRARY_PATH="/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

require_file() {
  local p="$1"
  [[ -f "$p" ]] || { echo "[FATAL] missing file: $p" >&2; exit 1; }
}

require_file "$SRC"
require_file "${BASE}.csv"
require_file "${BASE}_camera.csv"
require_file "${BASE}.mjpg"
require_file "${MAVLINK}/common/mavlink.h"

make_variant() {
  local tag="$1"
  local mode="$2"
  local tmp="/tmp/replay_b11_${tag}.cpp"

  cp "$SRC" "$tmp"

  python3 - "$tmp" "$mode" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
mode = sys.argv[2]
s = path.read_text()

old = """const Eigen::Vector3d w=corr.correctGyro((uint64_t)(s.source_ns/1000LL),acc,g,true);"""
if old not in s:
    raise SystemExit("FATAL: correctGyro injection point not found")

if mode == "CURRENT":
    new = old
elif mode == "BG_AVG_MINUS":
    new = """Eigen::Vector3d w=corr.correctGyro((uint64_t)(s.source_ns/1000LL),acc,g,true);
      w -= Eigen::Vector3d(+0.001031,-0.000052,+0.000165);"""
elif mode == "BG_AVG_PLUS":
    new = """Eigen::Vector3d w=corr.correctGyro((uint64_t)(s.source_ns/1000LL),acc,g,true);
      w += Eigen::Vector3d(+0.001031,-0.000052,+0.000165);"""
elif mode == "BG_X1_MINUS":
    new = """Eigen::Vector3d w=corr.correctGyro((uint64_t)(s.source_ns/1000LL),acc,g,true);
      w.x() -= 0.001000;"""
elif mode == "RAW_FLU":
    new = """const Eigen::Vector3d w=g;"""
else:
    raise SystemExit("FATAL: unknown mode " + mode)

path.write_text(s.replace(old, new, 1))
PY
}

build_and_run() {
  local tag="$1"
  local mode="$2"
  local src_tmp="/tmp/replay_b11_${tag}.cpp"
  local bin="/tmp/replay_b11_${tag}"
  local log="/home/vio/p11_b11_${tag}.txt"

  echo
  echo "================================================================"
  echo "$tag"
  echo "================================================================"

  make_variant "$tag" "$mode"

  echo "[BUILD]"
  g++ -std=c++17 -O2 \
    -I"$ROOT" \
    -I"$ROOT/tools" \
    -I"$MAVLINK" \
    -I/home/vio/Kimera-VIO/include \
    -I/home/vio/Kimera-VIO/build \
    -I/usr/local/include \
    -I/usr/include/eigen3 \
    $(pkg-config --cflags opencv4) \
    "$src_tmp" \
    -L/home/vio/Kimera-VIO/build \
    -L/usr/local/lib \
    -Wl,-rpath,/home/vio/Kimera-VIO/build \
    -Wl,-rpath,/usr/local/lib \
    -lkimera_vio \
    -lgtsam \
    -lgflags \
    -lglog \
    -lpthread \
    $(pkg-config --libs opencv4) \
    -o "$bin"

  echo "[RUN]"
  "$bin" \
    "$PARAMS" \
    "B11_${tag}" \
    "${BASE}.csv" \
    "${BASE}_camera.csv" \
    "${BASE}.mjpg" \
    > "$log" 2>&1

  grep -E \
    'final dP mm|final \|dP\||path length|max excursion|max speed|orientation span|final dRPY|REPLAY RESULT' \
    "$log"
}

: > "$SUMMARY"

for spec in \
  "CURRENT CURRENT" \
  "BG_AVG_MINUS BG_AVG_MINUS" \
  "BG_AVG_PLUS BG_AVG_PLUS" \
  "BG_X1_MINUS BG_X1_MINUS" \
  "RAW_FLU RAW_FLU"
do
  read -r tag mode <<< "$spec"
  build_and_run "$tag" "$mode" | tee -a "$SUMMARY"
done

echo
echo "================================================================"
echo "FINAL SUMMARY"
echo "================================================================"
cat "$SUMMARY"

echo
echo "Saved:"
echo "  $SUMMARY"
echo "  /home/vio/jtzero_extrinsics_replay_v10_B11_*.csv"
