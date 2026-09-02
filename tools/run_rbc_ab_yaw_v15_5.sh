#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-$HOME/jtzero-kimera-sync}"
REPLAY="${REPLAY:-/tmp/replay_mono_imu_zxy_ab_v11}"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"

BASE="$ROOT/params/JTZeroMonoFLU"
CAL="/tmp/JTZeroMonoFLU_RBC_CALIBRATED_v15_5"
IDEAL="/tmp/JTZeroMonoFLU_RBC_IDEAL_v15_5"
LOG_CAL="/home/vio/jtzero_rbc_v15_5_CALIBRATED.txt"
LOG_IDEAL="/home/vio/jtzero_rbc_v15_5_IDEAL.txt"

if [[ ! -d "$BASE" ]]; then
  echo "[FAIL] params directory not found: $BASE" >&2
  exit 2
fi
if [[ ! -x "$REPLAY" ]]; then
  echo "[FAIL] replay binary not found/executable: $REPLAY" >&2
  echo "Build tools/replay_mono_imu_zxy_ab_v11.cpp first." >&2
  exit 2
fi
for f in "$COMBINED" "$CAMINDEX" "$MJPEG"; do
  if [[ ! -f "$f" ]]; then
    echo "[FAIL] dataset file not found: $f" >&2
    exit 2
  fi
done

rm -rf "$CAL" "$IDEAL"
cp -a "$BASE" "$CAL"
cp -a "$BASE" "$IDEAL"

cat > "$IDEAL/LeftCameraParams.yaml" <<'EOF'
%YAML:1.0
# JT-Zero v15.5 R_BC A/B diagnostic ONLY.
# All parameters are copied from params/JTZeroMonoFLU except the rotational
# 3x3 part of T_BS below. Translation remains exactly [0, 0, -0.055] m.
#
# Body B_FLU: +X forward, +Y left, +Z up
# Camera C:    +X right, +Y down, +Z optical forward
# Ideal downward-facing mapping used only for A/B:
#   C_X -> +B_Y
#   C_Y -> +B_X
#   C_Z -> -B_Z

camera_id: left_cam

T_BS:
  cols: 4
  rows: 4
  data: [ 0.000000000,  1.000000000,  0.000000000,  0.000,
          1.000000000,  0.000000000,  0.000000000,  0.000,
          0.000000000,  0.000000000, -1.000000000, -0.055,
          0.000000000,  0.000000000,  0.000000000,  1.000 ]

rate_hz: 30
resolution: [640, 480]
camera_model: pinhole
intrinsics: [568.53170752165227, 569.68005562865858, 315.98271077441063, 239.88148589100641]
distortion_model: radial-tangential
distortion_coefficients: [0.073569192194028493, -0.095253893789117, -0.010810530757187299, -0.0022843373576970235, 0.082177400802757483]
EOF

echo "============================================================"
echo "JT-ZERO R_BC A/B YAW REPLAY v15.5"
echo "============================================================"
echo "Dataset:"
echo "  combined: $COMBINED"
echo "  camera:   $CAMINDEX"
echo "  mjpeg:    $MJPEG"
echo

echo "CALIBRATED T_BS:"
grep -A6 '^T_BS:' "$CAL/LeftCameraParams.yaml"
echo
echo "IDEAL T_BS:"
grep -A6 '^T_BS:' "$IDEAL/LeftCameraParams.yaml"
echo

echo "Checking that no other parameter file differs..."
if diff -qr --exclude=LeftCameraParams.yaml "$CAL" "$IDEAL"; then
  echo "[CHECK] PASS: only LeftCameraParams.yaml differs"
else
  echo "[CHECK] FAIL: files other than LeftCameraParams.yaml differ" >&2
  exit 3
fi

echo
echo "================ RUN A: CALIBRATED R_BC ================"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$REPLAY" "$CAL" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$LOG_CAL"

# V11 uses a fixed CURRENT CSV output name. Preserve each run separately.
if [[ -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then
  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv /home/vio/jtzero_rbc_v15_5_CALIBRATED.csv
fi

echo
echo "================ RUN B: IDEAL R_BC ====================="
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
  "$REPLAY" "$IDEAL" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$LOG_IDEAL"

if [[ -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then
  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv /home/vio/jtzero_rbc_v15_5_IDEAL.csv
fi

echo
echo "============================================================"
echo "R_BC A/B v15.5 COMPLETE"
echo "============================================================"
echo "A log: /home/vio/jtzero_rbc_v15_5_CALIBRATED.txt"
echo "B log: /home/vio/jtzero_rbc_v15_5_IDEAL.txt"
echo "A CSV: /home/vio/jtzero_rbc_v15_5_CALIBRATED.csv"
echo "B CSV: /home/vio/jtzero_rbc_v15_5_IDEAL.csv"
echo
echo "Compare the two ZXY A/B REPLAY V11 summary blocks above."
