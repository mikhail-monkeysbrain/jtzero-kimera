#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp"
TMP_SRC=/tmp/replay_gravity_pre_scale_v15_13.cpp

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
b='    if (std::abs(gyro_in.z()) >= (5.0*M_PI/180.0)) jtzero_yaw_seen_=true;\n    Eigen::Vector3d corrected=gyro_in;\n    Eigen::Vector3d jtzero_correction=Eigen::Vector3d::Zero();\n'
if s.count(a)!=1: raise SystemExit('[FATAL] corrected declaration match count != 1')
s=s.replace(a,b,1)

a='        corrected-=correction;\n'
b='        jtzero_correction=correction;\n        corrected-=correction;\n'
if s.count(a)!=1: raise SystemExit('[FATAL] correction application match count != 1')
s=s.replace(a,b,1)

a='    return corrected;\n'
b='    const double jtzero_output_scale = jtzero_yaw_seen_ ? 1.0 : JTZERO_PRE_GRAVITY_OUTPUT_SCALE;\n    return gyro_in-jtzero_output_scale*jtzero_correction;\n'
if s.count(a)!=1: raise SystemExit('[FATAL] final return corrected match count != 1')
s=s.replace(a,b,1)

a='  bool use_zxy_=true;\n'
b='  bool use_zxy_=true;\n  bool jtzero_yaw_seen_=false;\n'
if s.count(a)!=1: raise SystemExit('[FATAL] private use_zxy match count != 1')
s=s.replace(a,b,1)

p.write_text(s)
PY

echo "============================================================"
echo "JT-ZERO PRE GRAVITY OUTPUT SCALE SWEEP v15.13"
echo "============================================================"
echo "Only PRE-yaw gravity correction returned to Kimera is scaled."
echo "Once |ZXY gyro Z| >= 5 deg/s is seen, output scale is forced to 1.0 forever."
echo "Internal gravity detector/state/correction/current propagation remain CURRENT."
echo "Dataset: $COMBINED"
echo "Params:  $PARAMS"
echo

grep -n -E 'jtzero_yaw_seen_|jtzero_correction|JTZERO_PRE_GRAVITY_OUTPUT_SCALE|corrected-=correction|theta=-corrected' "$TMP_SRC"

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
SUMMARY=/home/vio/jtzero_gravity_pre_scale_v15_13_summary.txt
: > "$SUMMARY"

for i in "${!SCALES[@]}"; do
  scale="${SCALES[$i]}"
  tag="${TAGS[$i]}"
  bin="/tmp/replay_gravity_pre_scale_v15_13_${tag}"
  log="/home/vio/jtzero_gravity_pre_scale_v15_13_${tag}.txt"
  csv="/home/vio/jtzero_gravity_pre_scale_v15_13_${tag}.csv"

  echo
  echo "================ PRE SCALE $scale ($tag) =========================="
  g++ "${COMMON[@]}" -DJTZERO_PRE_GRAVITY_OUTPUT_SCALE="$scale" "$TMP_SRC" -o "$bin" \
    $(pkg-config --cflags --libs opencv4) "${LIBS[@]}"

  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$bin" "$PARAMS" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$log"

  cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"

  {
    echo "PRE_SCALE=$scale TAG=$tag"
    grep -E '^(backend states:|integral FED gyro deg XYZ:|integral FED-RAW deg XYZ:|final dP mm:|final \|dP\| mm:|path length mm:|max excursion mm:|max speed mm/s:|orientation span deg|final dRPY deg:)' "$log"
    echo
  } >> "$SUMMARY"
done

echo
echo "================ PRE RESPONSE CURVE SUMMARY ======================="
cat "$SUMMARY"

echo "============================================================"
echo "v15.13 COMPLETE"
echo "============================================================"
echo "Summary: /home/vio/jtzero_gravity_pre_scale_v15_13_summary.txt"
echo "State CSVs: /home/vio/jtzero_gravity_pre_scale_v15_13_S*.csv"
echo
echo "Sanity check:"
echo "  PRE_SCALE=1 must reproduce CURRENT exactly (~442.545 mm)."
echo "Interpretation:"
echo "  If PRE scale strongly changes the yaw trajectory while POST remains 1x,"
echo "  the four microscopic PRE events are sufficient to select different VIO trajectories."
