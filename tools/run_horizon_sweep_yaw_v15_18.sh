#!/usr/bin/env bash
set -u -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUTROOT=/home/vio/jtzero_horizon_sweep_v15_18
BIN=/tmp/replay_horizon_sweep_v15_18

mkdir -p "$OUTROOT"
rm -f "$OUTROOT"/*.txt "$OUTROOT"/*.csv "$OUTROOT"/*.tsv 2>/dev/null || true
rm -rf /tmp/jtzero_v15_18_h*

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
  'JT-ZERO BACKEND HORIZON SWEEP YAW v15.18' \
  '============================================================' \
  'One dataset. Only BackendParams.yaml:nr_states changes.' \
  'Full logs are written to disk; console output is intentionally compact.' \
  "Dataset: $COMBINED" \
  "Params:  $BASE" \
  "Output:  $OUTROOT"

echo '[BUILD] replay binary'
g++ "${COMMON[@]}" "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  $(pkg-config --cflags --libs opencv4) "${LIBS[@]}" || exit 2

replace_nr_states() {
  local file="$1" value="$2"
  local n
  n=$(grep -Ec '^[[:space:]]*nr_states:' "$file" || true)
  if [[ "$n" != "1" ]]; then
    echo "[FATAL] nr_states match count=$n in $file" >&2
    return 1
  fi
  sed -E -i "s|^([[:space:]]*nr_states:[[:space:]]*).*|\\1${value}|" "$file"
}

extract_metrics() {
  local log="$1"
  local states dp path exc speed span drpy
  states=$(awk -F': ' '/^backend states:/{print $2; exit}' "$log")
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2; exit}' "$log")
  path=$(awk -F': ' '/^path length mm:/{print $2; exit}' "$log")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2; exit}' "$log")
  speed=$(awk -F': ' '/^max speed mm\/s:/{print $2; exit}' "$log")
  span=$(awk -F': ' '/^orientation span deg/{print $2; exit}' "$log" | tr -d '[]')
  drpy=$(awk -F': ' '/^final dRPY deg:/{print $2; exit}' "$log" | tr -d '[]')
  read -r sr sp sy <<< "${span:-NA NA NA}"
  read -r rr rp ry <<< "${drpy:-NA NA NA}"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${states:-NA}" "${dp:-NA}" "${path:-NA}" "${exc:-NA}" "${speed:-NA}" \
    "${sr:-NA}" "${sp:-NA}" "${sy:-NA}" "${rr:-NA}" "${rp:-NA}" "${ry:-NA}"
}

SUMMARY="$OUTROOT/summary.tsv"
echo -e 'nr_states\texit\tbackend_states\tcomplete\tfinal_dp_mm\tpath_mm\tmax_exc_mm\tmax_speed_mm_s\troll_span_deg\tpitch_span_deg\tyaw_span_deg\tfinal_roll_deg\tfinal_pitch_deg\tfinal_yaw_deg\tbackend_error' > "$SUMMARY"

run_one() {
  local h="$1"
  local params="$BASE"
  local dir=""
  if [[ "$h" != "25" ]]; then
    dir="/tmp/jtzero_v15_18_h${h}"
    rm -rf "$dir"
    cp -a "$BASE" "$dir"
    replace_nr_states "$dir/BackendParams.yaml" "$h" || return 1
    params="$dir"
  fi

  local tag="H${h}"
  local log="$OUTROOT/${tag}.txt"
  local csv="$OUTROOT/${tag}.csv"

  printf '[RUN] nr_states=%-3s ... ' "$h"
  set +e
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} \
    "$BIN" "$params" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" > "$log" 2>&1
  local rc=$?
  set -e

  if [[ $rc -eq 0 && -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then
    cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"
  fi

  local metrics states dp path exc speed sr sp sy rr rp ry
  metrics=$(extract_metrics "$log")
  IFS=$'\t' read -r states dp path exc speed sr sp sy rr rp ry <<< "$metrics"

  local berr=0
  if grep -Eq 'Backend did not return an output|Attempting to at the key|F[0-9]{8}|E[0-9]{8}.*VioBackend' "$log"; then
    berr=1
  fi

  # complete is filled after BASE reference is known; temporary PENDING here.
  echo -e "$h\t$rc\t$states\tPENDING\t$dp\t$path\t$exc\t$speed\t$sr\t$sp\t$sy\t$rr\t$rp\t$ry\t$berr" >> "$SUMMARY"
  printf 'states=%s yaw_span=%s dp=%s mm backend_error=%s\n' "$states" "$sy" "$dp" "$berr"
}

# Sweep around CURRENT=25 and far enough to reveal monotonicity / branch changes.
HORIZONS=(10 15 20 25 30 35 40 50 60 75 100)
set -e
for h in "${HORIZONS[@]}"; do
  run_one "$h"
done

BASE_STATES=$(awk -F'\t' '$1==25{print $3}' "$SUMMARY")
BASE_YAW=$(awk -F'\t' '$1==25{print $11}' "$SUMMARY")
BASE_DP=$(awk -F'\t' '$1==25{print $5}' "$SUMMARY")

if [[ -z "${BASE_STATES:-}" || "$BASE_STATES" == "NA" || -z "${BASE_YAW:-}" || "$BASE_YAW" == "NA" ]]; then
  echo '[FATAL] could not extract BASE completion metrics.' >&2
  exit 3
fi

# Valid/full replay gate: at least 95% of BASE backend states, at least 90% of BASE yaw span,
# zero process error, and no backend internal shutdown/error signature.
TMP="$OUTROOT/summary.tmp.tsv"
awk -F'\t' -v OFS='\t' -v bs="$BASE_STATES" -v by="$BASE_YAW" '
NR==1 {print; next}
{
  complete="NO";
  if ($2==0 && $3!="NA" && $11!="NA" && $15==0 && ($3+0)>=0.95*bs && ($11+0)>=0.90*by) complete="YES";
  $4=complete;
  print;
}' "$SUMMARY" > "$TMP"
mv "$TMP" "$SUMMARY"

VALID="$OUTROOT/valid_only.tsv"
{
  head -n1 "$SUMMARY"
  tail -n +2 "$SUMMARY" | awk -F'\t' '$4=="YES"'
} > "$VALID"

RANKED="$OUTROOT/valid_ranked_by_final_dp.tsv"
{
  head -n1 "$SUMMARY"
  tail -n +2 "$SUMMARY" | awk -F'\t' '$4=="YES" && $5!="NA"' | sort -t $'\t' -k5,5n
} > "$RANKED"

echo
echo '================ v15.18 VALID HORIZON SWEEP ======================'
printf '%-10s %-8s %-12s %-12s %-12s %-10s\n' NR_STATES STATES FINAL_DP_MM MAX_EXC_MM YAW_SPAN_DEG VS_BASE_PCT
awk -F'\t' -v b="$BASE_DP" 'NR>1 && $4=="YES" {printf "%-10s %-8s %-12.3f %-12.3f %-12.3f %+9.1f%%\n",$1,$3,$5,$7,$11,100*($5-b)/b}' "$SUMMARY"

echo
echo '================ INVALID / INCOMPLETE ============================'
awk -F'\t' 'NR>1 && $4!="YES" {printf "nr_states=%-3s exit=%s states=%s yaw_span=%s backend_error=%s final_dp=%s\n",$1,$2,$3,$11,$15,$5}' "$SUMMARY"

echo
echo '================ INTERPRETATION GATE ============================='
echo 'If full valid runs improve smoothly or mostly consistently as nr_states grows:'
echo '  -> fixed-lag horizon / marginalization is a primary amplifier of false translation.'
echo 'If results jump non-monotonically between otherwise full valid runs:'
echo '  -> optimizer basin / numerical conditioning dominates; nr_states=50 is not a robust fix by itself.'
echo 'If only short horizons fail while >=N converge to a stable low-error plateau:'
echo '  -> there is a minimum history length needed for this planar yaw geometry.'
echo
echo "BASE(25): states=$BASE_STATES yaw_span=$BASE_YAW final_dp=$BASE_DP mm"
echo "Summary: $SUMMARY"
echo "Valid only: $VALID"
echo "Valid ranking: $RANKED"
echo "Full logs/CSVs: $OUTROOT"
echo 'RESULT: COMPLETE'
