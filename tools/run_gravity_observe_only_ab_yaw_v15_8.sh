#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"

COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
PARAMS="${4:-$ROOT/params/JTZeroMonoFLU}"

CUR_SRC=/tmp/replay_gravity_current_v15_8.cpp
OBS_SRC=/tmp/replay_gravity_observe_only_v15_8.cpp
CUR_BIN=/tmp/replay_gravity_current_v15_8
OBS_BIN=/tmp/replay_gravity_observe_only_v15_8
CUR_LOG=/home/vio/jtzero_gravity_v15_8_CURRENT.txt
OBS_LOG=/home/vio/jtzero_gravity_v15_8_OBSERVE_ONLY.txt
CUR_CSV=/home/vio/jtzero_gravity_v15_8_CURRENT.csv
OBS_CSV=/home/vio/jtzero_gravity_v15_8_OBSERVE_ONLY.csv

cp "$SRC" "$CUR_SRC"
cp "$SRC" "$OBS_SRC"

# OBSERVE_ONLY keeps the gravity subsystem fully active and preserves its
# internal state evolution. The only difference is what is returned to Kimera:
# CURRENT returns `corrected`, while OBSERVE_ONLY returns `gyro_in` after the
# exact same detector, initialization, correction calculation and gravity_body_
# propagation have already executed.
MATCHES="$(grep -c '^[[:space:]]*return corrected;' "$OBS_SRC" || true)"
if [[ "$MATCHES" != "1" ]]; then
  echo "[FATAL] expected exactly one 'return corrected;' in v11, found $MATCHES" >&2
  exit 2
fi
sed -i 's/^[[:space:]]*return corrected;/    return gyro_in;/' "$OBS_SRC"

DIFF_LINES="$(diff -u "$CUR_SRC" "$OBS_SRC" || true)"
echo "============================================================"
echo "JT-ZERO GRAVITY OBSERVE-ONLY A/B YAW REPLAY v15.8"
echo "============================================================"
echo "Dataset:"
echo "  combined: $COMBINED"
echo "  camera:   $CAMINDEX"
echo "  mjpeg:    $MJPEG"
echo "  params:   $PARAMS"
echo
echo "Source A/B diff:"
echo "$DIFF_LINES"

REMOVED="$(echo "$DIFF_LINES" | grep -c '^-.*return corrected;' || true)"
ADDED="$(echo "$DIFF_LINES" | grep -c '^+.*return gyro_in;' || true)"
if [[ "$REMOVED" != "1" || "$ADDED" != "1" ]]; then
  echo "[FATAL] OBSERVE_ONLY patch verification failed" >&2
  exit 2
fi

if echo "$DIFF_LINES" | grep -q 'allow_gravity_feedback'; then
  echo "[FATAL] gravity enable/disable logic was unexpectedly modified" >&2
  exit 2
fi

if echo "$DIFF_LINES" | grep -q 'gravity_body_.*AngleAxis'; then
  echo "[FATAL] gravity state propagation was unexpectedly modified" >&2
  exit 2
fi

echo "[CHECK] PASS: B changes only the value returned to Kimera: corrected -> gyro_in"
echo "[CHECK] Gravity detector/state/correction calculation/propagation remain identical"

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
g++ "${COMMON[@]}" "$OBS_SRC" -o "$OBS_BIN" $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"

echo
echo "================ RUN A: CURRENT ================================"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$CUR_BIN" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$CUR_LOG"
cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$CUR_CSV"

echo
echo "================ RUN B: OBSERVE_ONLY ==========================="
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$OBS_BIN" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$OBS_LOG"
cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$OBS_CSV"

echo
echo "============================================================"
echo "GRAVITY OBSERVE-ONLY A/B v15.8 COMPLETE"
echo "============================================================"
echo "A log: $CUR_LOG"
echo "B log: $OBS_LOG"
echo "A CSV: $CUR_CSV"
echo "B CSV: $OBS_CSV"
echo
echo "Interpretation:"
echo "  If OBSERVE_ONLY approaches NO_GRAVITY (~274 mm), the tiny gravity"
echo "  correction sent to Kimera is sufficient to alter the VIO solution."
echo "  If OBSERVE_ONLY stays near CURRENT (~442 mm), the v15.6 difference"
echo "  came mainly from changing gravity subsystem state/detection history."
