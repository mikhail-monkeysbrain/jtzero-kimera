#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"

COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
PARAMS="${4:-$ROOT/params/JTZeroMonoFLU}"

CUR_SRC=/tmp/replay_gravity_current_v15_6.cpp
OFF_SRC=/tmp/replay_gravity_off_v15_6.cpp
CUR_BIN=/tmp/replay_gravity_current_v15_6
OFF_BIN=/tmp/replay_gravity_off_v15_6
CUR_LOG=/home/vio/jtzero_gravity_v15_6_CURRENT.txt
OFF_LOG=/home/vio/jtzero_gravity_v15_6_NO_GRAVITY.txt
CUR_CSV=/home/vio/jtzero_gravity_v15_6_CURRENT.csv
OFF_CSV=/home/vio/jtzero_gravity_v15_6_NO_GRAVITY.csv

cp "$SRC" "$CUR_SRC"
cp "$SRC" "$OFF_SRC"

# In v11 the replay correction method has a default argument
#   bool allow_gravity_feedback=true
# and the replay call relies on that default.  For B we change ONLY that
# default to false. ZXY remains enabled because both runs execute mode CURRENT.
MATCHES="$(grep -c 'bool allow_gravity_feedback=true' "$OFF_SRC" || true)"
if [[ "$MATCHES" != "1" ]]; then
  echo "[FATAL] expected exactly one gravity-feedback default in v11, found $MATCHES" >&2
  exit 2
fi
sed -i 's/bool allow_gravity_feedback=true/bool allow_gravity_feedback=false/' "$OFF_SRC"

# Sanity check: source copies must differ only in that one token.
DIFF_LINES="$(diff -u "$CUR_SRC" "$OFF_SRC" || true)"
echo "============================================================"
echo "JT-ZERO GRAVITY FEEDBACK A/B YAW REPLAY v15.6"
echo "============================================================"
echo "Dataset:"
echo "  combined: $COMBINED"
echo "  camera:   $CAMINDEX"
echo "  mjpeg:    $MJPEG"
echo "  params:   $PARAMS"
echo
echo "Source A/B diff:"
echo "$DIFF_LINES"
if ! echo "$DIFF_LINES" | grep -q 'allow_gravity_feedback=false'; then
  echo "[FATAL] gravity OFF patch not present" >&2
  exit 2
fi

echo "[CHECK] PASS: B differs only by gravity feedback default true -> false"

COMMON=(
  -std=c++17 -O2
  -I"$ROOT/tools"
  -I/home/vio/Kimera-VIO/include
  -I/home/vio/Kimera-VIO/build
  -I/home/vio/Kimera-VIO/third_party/mavlink
  -I/usr/include/eigen3
)
LIBS=(
  -L/home/vio/Kimera-VIO/build
  -L/usr/local/lib
  -lkimera_vio -lgtsam -lglog -lgflags -lpthread
)

# shellcheck disable=SC2046
g++ "${COMMON[@]}" "$CUR_SRC" -o "$CUR_BIN" $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"
# shellcheck disable=SC2046
g++ "${COMMON[@]}" "$OFF_SRC" -o "$OFF_BIN" $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"

echo
echo "================ RUN A: CURRENT GRAVITY FEEDBACK ================"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$CUR_BIN" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$CUR_LOG"
cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$CUR_CSV"

echo
echo "================ RUN B: NO GRAVITY FEEDBACK ====================="
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$OFF_BIN" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$OFF_LOG"
cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$OFF_CSV"

echo
echo "============================================================"
echo "GRAVITY FEEDBACK A/B v15.6 COMPLETE"
echo "============================================================"
echo "A log: $CUR_LOG"
echo "B log: $OFF_LOG"
echo "A CSV: $CUR_CSV"
echo "B CSV: $OFF_CSV"
echo
echo "Compare the two ZXY A/B REPLAY V11 summary blocks above."
