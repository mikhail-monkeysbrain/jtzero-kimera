#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"
TMP_SRC=/tmp/replay_gravity_scale_v15_12.cpp

COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
PARAMS="${4:-$ROOT/params/JTZeroMonoFLU}"

cp "$SRC" "$TMP_SRC"

python3 - "$TMP_SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text()
a='    Eigen::Vector3d corrected=gyro_in;\n'
b='    Eigen::Vector3d corrected=gyro_in;\n    Eigen::Vector3d jtzero_correction=Eigen::Vector3d::Zero();\n'
if s.count(a)!=1: raise SystemExit('[FATAL] corrected declaration match count != 1')
s=s.replace(a,b,1)
a='        corrected-=correction;\n'
b='        jtzero_correction=correction;\n        corrected-=correction;\n'
if s.count(a)!=1: raise SystemExit('[FATAL] correction application match count != 1')
s=s.replace(a,b,1)
a='    return corrected;\n'
b='    return gyro_in-JTZERO_GRAVITY_OUTPUT_SCALE*jtzero_correction;\n'
if s.count(a)!=1: raise SystemExit('[FATAL] final return corrected match count != 1')
s=s.replace(a,b,1)
p.write_text(s)
PY

echo "============================================================"
echo "JT-ZERO GRAVITY OUTPUT SCALE RESPONSE SWEEP v15.12"
echo "============================================================"
echo "Internal gravity detector/state/correction propagation remains CURRENT."
echo "Only correction returned to Kimera is scaled."
echo "Dataset: $COMBINED"
echo "Params:  $PARAMS"
echo

grep -n -E 'jtzero_correction|JTZERO_GRAVITY_OUTPUT_SCALE|corrected-=correction|theta=-corrected' "$TMP_SRC"

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

SCALES=(0 0.25 0.5 1 2)
TAGS=(S0 S025 S05 S1 S2)
SUMMARY=/home/vio/jtzero_gravity_scale_v15_12_summary.txt
: > "$SUMMARY"

for i in "${!SCALES[@]}"; do
  scale="${SCALES[$i]}"
  tag="${TAGS[$i]}"
  bin="/tmp/replay_gravity_scale_v15_12_${tag}"
  log="/home/vio/jtzero_gravity_scale_v15_12_${tag}.txt"
  csv="/home/vio/jtzero_gravity_scale_v15_12_${tag}.csv"

  echo
  echo "================ SCALE $scale ($tag) ============================="
  # shellcheck disable=SC2046
  g++ "${COMMON[@]}" -DJTZERO_GRAVITY_OUTPUT_SCALE="$scale" "$TMP_SRC" -o "$bin" \
    $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"

  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$bin" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$log"

  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"

  {
    echo "SCALE=$scale TAG=$tag"
    grep -E '^(backend states:|integral FED gyro deg XYZ:|integral FED-RAW deg XYZ:|final dP mm:|final \|dP\| mm:|path length mm:|max excursion mm:|max speed mm/s:|orientation span deg|final dRPY deg:)' "$log"
    echo
  } >> "$SUMMARY"
done

echo
echo "================ RESPONSE CURVE SUMMARY ==========================="
cat "$SUMMARY"

echo "============================================================"
echo "v15.12 COMPLETE"
echo "============================================================"
echo "Summary: $SUMMARY"
echo "State CSVs: /home/vio/jtzero_gravity_scale_v15_12_S*.csv"
echo
echo "Sanity checks:"
echo "  SCALE=0 should reproduce v15.8 OBSERVE_ONLY (~274.446 mm)."
echo "  SCALE=1 should reproduce CURRENT exactly (~442.545 mm)."
echo "  If those anchors fail, do not interpret the response curve."
