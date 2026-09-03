#!/usr/bin/env bash
set -u -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUTROOT=/home/vio/jtzero_yaw_matrix_v15_17
BIN=/tmp/replay_yaw_matrix_v15_17
mkdir -p "$OUTROOT"
rm -rf /tmp/jtzero_v15_17_*

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

printf '%s\n' \
  '============================================================' \
  'JT-ZERO YAW MULTI-HYPOTHESIS MATRIX v15.17' \
  '============================================================' \
  'One dataset; every replay variant starts from CURRENT params.' \
  'Each variant changes one mechanism only.' \
  "Dataset: $COMBINED" \
  "Params:  $BASE" \
  "Output:  $OUTROOT"

echo '[BUILD] replay binary'
g++ "${COMMON[@]}" "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4) "${LIBS[@]}" || exit 2

# Independent visual observability controls from v15.14/v15.15.
VIS14=/tmp/analyze_visual_homography_yaw_v15_14
VIS15=/tmp/analyze_essential_translation_stability_v15_15
if [[ -f "$ROOT/tools/analyze_visual_homography_yaw_v15_14.cpp" ]]; then
  g++ -std=c++17 -O2 "$ROOT/tools/analyze_visual_homography_yaw_v15_14.cpp" -o "$VIS14" $(pkg-config --cflags --libs opencv4) && \
    "$VIS14" "$COMBINED" "$CAMINDEX" "$MJPEG" | tee "$OUTROOT/visual_homography.txt"
fi
if [[ -f "$ROOT/tools/analyze_essential_translation_stability_v15_15.cpp" ]]; then
  g++ -std=c++17 -O2 "$ROOT/tools/analyze_essential_translation_stability_v15_15.cpp" -o "$VIS15" $(pkg-config --cflags --libs opencv4) && \
    "$VIS15" "$COMBINED" "$CAMINDEX" "$MJPEG" "$BASE/LeftCameraParams.yaml" | tee "$OUTROOT/visual_essential.txt"
fi

replace_key() {
  local file="$1" key="$2" value="$3"
  local n
  n=$(grep -Ec "^[[:space:]]*${key}:" "$file" || true)
  if [[ "$n" != "1" ]]; then
    echo "[FATAL] key '$key' match count=$n in $file" >&2
    return 1
  fi
  sed -E -i "s|^([[:space:]]*${key}:[[:space:]]*).*|\\1${value}|" "$file"
}

mkvariant() {
  local tag="$1"; shift
  local dir="/tmp/jtzero_v15_17_${tag}"
  rm -rf "$dir"
  cp -a "$BASE" "$dir"
  while (( "$#" >= 3 )); do
    local rel="$1" key="$2" value="$3"
    shift 3
    replace_key "$dir/$rel" "$key" "$value" || return 1
  done
  echo "$dir"
}

# tag|group|description|file:key:value [file:key:value ...]
# Every entry is an orthogonal hypothesis relative to CURRENT.
VARIANTS=(
  'BASE|CONTROL|CURRENT'
  'LIN_IMPLICIT|SMART_LINEARIZATION|linearizationMode 0->1 (Implicit Schur)|BackendParams.yaml:linearizationMode:1'
  'LIN_SVD|SMART_LINEARIZATION|linearizationMode 0->3 (Jacobian SVD)|BackendParams.yaml:linearizationMode:3'
  'DEG_IGNORE|DEGENERACY|degeneracyMode 1->0 (ignore)|BackendParams.yaml:degeneracyMode:0'
  'DEG_INFINITY|DEGENERACY|degeneracyMode 1->2 (handle infinity)|BackendParams.yaml:degeneracyMode:2'
  'RANK_LOOSE|TRIANGULATION|rankTolerance 1->0.1|BackendParams.yaml:rankTolerance:0.1'
  'RANK_STRICT|TRIANGULATION|rankTolerance 1->10|BackendParams.yaml:rankTolerance:10'
  'DIST_NEAR|TRIANGULATION|landmarkDistanceThreshold 10->2 m|BackendParams.yaml:landmarkDistanceThreshold:2'
  'DIST_FAR|TRIANGULATION|landmarkDistanceThreshold 10->100 m|BackendParams.yaml:landmarkDistanceThreshold:100'
  'OUTLIER_STRICT|VISION_OUTLIER|outlierRejection 3->0.5 px|BackendParams.yaml:outlierRejection:0.5'
  'OUTLIER_OFF|VISION_OUTLIER|outlierRejection 3->-1 disabled|BackendParams.yaml:outlierRejection:-1'
  'VISION_STRONG|VISION_WEIGHT|smartNoiseSigma 3->1|BackendParams.yaml:smartNoiseSigma:1.0'
  'VISION_WEAK|VISION_WEIGHT|smartNoiseSigma 3->10|BackendParams.yaml:smartNoiseSigma:10.0'
  'RETRIANG_SLOW|TRIANGULATION|retriangulationThreshold 0.001->0.1|BackendParams.yaml:retriangulationThreshold:0.1'
  'KF_FAST|KEYFRAME|keyframes much faster|FrontendParams.yaml:min_intra_keyframe_time:0.05|FrontendParams.yaml:max_intra_keyframe_time:0.20'
  'KF_SLOW|KEYFRAME|keyframes much slower|FrontendParams.yaml:min_intra_keyframe_time:0.40|FrontendParams.yaml:max_intra_keyframe_time:1.00'
  'FLOW_STATIC|TRACKING|rotational optical-flow predictor -> static|FrontendParams.yaml:optical_flow_predictor_type:0'
  'POSE_MONO|INITIAL_GUESS|pose_guess_source IMU->MONO|BackendParams.yaml:pose_guess_source:1'
  'ACC_RW_LOW|IMU_WEIGHT|accelerometer random walk 3e-2->3e-3|ImuParams.yaml:accelerometer_random_walk:3.0000e-3'
  'ACC_RW_HIGH|IMU_WEIGHT|accelerometer random walk 3e-2->3e-1|ImuParams.yaml:accelerometer_random_walk:3.0000e-1'
  'ACC_NOISE_LOW|IMU_WEIGHT|accelerometer noise x0.1|ImuParams.yaml:accelerometer_noise_density:2.0000e-4'
  'ACC_NOISE_HIGH|IMU_WEIGHT|accelerometer noise x10|ImuParams.yaml:accelerometer_noise_density:2.0000e-2'
  'GYRO_NOISE_LOW|IMU_WEIGHT|gyro noise x0.1|ImuParams.yaml:gyroscope_noise_density:1.6968e-05'
  'GYRO_NOISE_HIGH|IMU_WEIGHT|gyro noise x10|ImuParams.yaml:gyroscope_noise_density:1.6968e-03'
  'NO_MOTION_WEAK|MOTION_PRIOR|weaken zero/no-motion priors|BackendParams.yaml:zero_velocity_precision:1|BackendParams.yaml:no_motion_position_precision:1|BackendParams.yaml:no_motion_rotation_precision:1'
  'HORIZON_SHORT|OPTIMIZER|nr_states 25->10|BackendParams.yaml:nr_states:10'
  'HORIZON_LONG|OPTIMIZER|nr_states 25->50|BackendParams.yaml:nr_states:50'
  'RELIN_TIGHT|OPTIMIZER|relinearizeThreshold 0.01->0.001|BackendParams.yaml:relinearizeThreshold:0.001'
  'RELIN_LOOSE|OPTIMIZER|relinearizeThreshold 0.01->0.1|BackendParams.yaml:relinearizeThreshold:0.1'
)

SUMMARY="$OUTROOT/summary.tsv"
echo -e 'tag\tgroup\tdescription\texit\tstates\tfinal_dp_mm\tpath_mm\tmax_exc_mm\tmax_speed_mm_s\troll_span_deg\tpitch_span_deg\tyaw_span_deg\tfinal_roll_deg\tfinal_pitch_deg\tfinal_yaw_deg' > "$SUMMARY"

run_one() {
  local spec="$1"
  IFS='|' read -r -a f <<< "$spec"
  local tag="${f[0]}" group="${f[1]}" desc="${f[2]}"
  local params="$BASE"
  if [[ "$tag" != 'BASE' ]]; then
    local args=()
    local i token rel key value
    for ((i=3;i<${#f[@]};i++)); do
      token="${f[$i]}"
      rel="${token%%:*}"; token="${token#*:}"
      key="${token%%:*}"; value="${token#*:}"
      args+=("$rel" "$key" "$value")
    done
    params=$(mkvariant "$tag" "${args[@]}") || {
      echo -e "$tag\t$group\t$desc\tPARAM_ERROR" >> "$SUMMARY"
      return
    }
  fi

  local log="$OUTROOT/${tag}.txt"
  local csv="$OUTROOT/${tag}.csv"
  echo
  echo "================ $tag [$group] ================================"
  echo "$desc"
  if [[ "$tag" != 'BASE' ]]; then
    diff -ru "$BASE" "$params" || true
  fi

  set +e
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$params" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" > "$log" 2>&1
  local rc=$?
  set -e
  cat "$log"

  if [[ $rc -eq 0 && -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then
    cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"
  fi

  local states dp path exc speed span drpy
  states=$(awk -F': ' '/^backend states:/{print $2; exit}' "$log")
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2; exit}' "$log")
  path=$(awk -F': ' '/^path length mm:/{print $2; exit}' "$log")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2; exit}' "$log")
  speed=$(awk -F': ' '/^max speed mm\/s:/{print $2; exit}' "$log")
  span=$(awk -F': ' '/^orientation span deg/{print $2; exit}' "$log" | tr -d '[]')
  drpy=$(awk -F': ' '/^final dRPY deg:/{print $2; exit}' "$log" | tr -d '[]')
  read -r sr sp sy <<< "$span"
  read -r rr rp ry <<< "$drpy"
  echo -e "$tag\t$group\t$desc\t$rc\t${states:-NA}\t${dp:-NA}\t${path:-NA}\t${exc:-NA}\t${speed:-NA}\t${sr:-NA}\t${sp:-NA}\t${sy:-NA}\t${rr:-NA}\t${rp:-NA}\t${ry:-NA}" >> "$SUMMARY"
}

set -e
for spec in "${VARIANTS[@]}"; do
  run_one "$spec"
done

BASE_DP=$(awk -F'\t' '$1=="BASE"{print $6}' "$SUMMARY")
RANKED="$OUTROOT/ranked_by_final_dp.tsv"
{
  head -n1 "$SUMMARY"
  tail -n +2 "$SUMMARY" | awk -F'\t' '$4==0 && $6!="NA"' | sort -t $'\t' -k6,6n
} > "$RANKED"

echo
echo '================ v15.17 COMPACT RANKING =========================='
printf '%-18s %-20s %12s %10s %12s\n' TAG GROUP FINAL_DP_MM VS_BASE_PCT MAX_EXC_MM
awk -F'\t' -v b="$BASE_DP" 'NR>1 && $4==0 && $6!="NA" {printf "%-18s %-20s %12.3f %10.1f %12.3f\n",$1,$2,$6,100*($6-b)/b,$8}' "$RANKED"

echo
echo '================ HYPOTHESIS GATES ================================'
echo 'Interpret groups, not single lucky minima:'
echo '  SMART_LINEARIZATION / DEGENERACY / TRIANGULATION -> smart-factor numerical/geometry handling.'
echo '  VISION_WEIGHT / VISION_OUTLIER -> visual constraints are driving the false translation.'
echo '  KEYFRAME / TRACKING -> temporal baseline / feature tracking is the amplifier.'
echo '  INITIAL_GUESS -> optimizer basin selected by pose initialization.'
echo '  IMU_WEIGHT -> accelerometer/gyro weighting or bias freedom is the amplifier.'
echo '  MOTION_PRIOR -> hidden zero/no-motion constraints participate.'
echo '  OPTIMIZER -> horizon/relinearization numerical sensitivity.'
echo
if [[ -n "${BASE_DP:-}" ]]; then
  echo "BASE final |dP|: $BASE_DP mm"
fi
echo "Summary: $SUMMARY"
echo "Ranking: $RANKED"
echo "Logs/CSVs: $OUTROOT"
echo 'RESULT: COMPLETE'
