#!/usr/bin/env bash
set -u -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="$ROOT/params/JTZeroMonoFLU"
COMBINED="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAMINDEX="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPEG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
OUTROOT=/home/vio/jtzero_horizon_threshold_v15_19
BIN=/tmp/replay_horizon_threshold_v15_19
mkdir -p "$OUTROOT"
rm -rf /tmp/jtzero_v15_19_H*

COMMON=(-std=c++17 -O2 -I"$ROOT/tools" -I/home/vio/Kimera-VIO/include -I/home/vio/Kimera-VIO/build -I/home/vio/Kimera-VIO/third_party/mavlink -I/usr/include/eigen3)
LIBS=(-L/home/vio/Kimera-VIO/build -L/usr/local/lib -lkimera_vio -lgtsam -lglog -lgflags -lpthread)

echo '============================================================'
echo 'JT-ZERO BACKEND HORIZON THRESHOLD YAW v15.19'
echo '============================================================'
echo 'Fine sweep around the nr_states=25 -> 30 discontinuity.'
echo "Output: $OUTROOT"

echo '[BUILD] replay binary'
g++ "${COMMON[@]}" "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4) "${LIBS[@]}" || exit 2

replace_nr_states() {
  local file="$1" value="$2"
  local n
  n=$(grep -Ec '^[[:space:]]*nr_states:' "$file" || true)
  [[ "$n" == 1 ]] || { echo "[FATAL] nr_states matches=$n in $file" >&2; return 1; }
  sed -E -i "s|^([[:space:]]*nr_states:[[:space:]]*).*|\\1${value}|" "$file"
}

SUMMARY="$OUTROOT/summary.tsv"
echo -e 'nr_states\texit\tstates\tbackend_error\tfinal_dp_mm\tpath_mm\tmax_exc_mm\tmax_speed_mm_s\troll_span_deg\tpitch_span_deg\tyaw_span_deg\tfinal_roll_deg\tfinal_pitch_deg\tfinal_yaw_deg\tstatus' > "$SUMMARY"

# Dense scan across the cliff plus a few anchors.
VALUES=(21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 40 50 75 100)

for n in "${VALUES[@]}"; do
  dir="/tmp/jtzero_v15_19_H${n}"
  rm -rf "$dir"; cp -a "$BASE" "$dir"
  replace_nr_states "$dir/BackendParams.yaml" "$n" || exit 3
  log="$OUTROOT/H${n}.txt"; csv="$OUTROOT/H${n}.csv"
  printf '[RUN] nr_states=%-3s ... ' "$n"
  set +e
  LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN" "$dir" CURRENT "$COMBINED" "$CAMINDEX" "$MJPEG" > "$log" 2>&1
  rc=$?
  set -e
  if [[ $rc -eq 0 && -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv ]]; then cp /home/vio/jtzero_zxy_replay_v11_CURRENT.csv "$csv"; fi
  states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log")
  dp=$(awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$log")
  path=$(awk -F': ' '/^path length mm:/{print $2;exit}' "$log")
  exc=$(awk -F': ' '/^max excursion mm:/{print $2;exit}' "$log")
  speed=$(awk -F': ' '/^max speed mm\/s:/{print $2;exit}' "$log")
  span=$(awk -F': ' '/^orientation span deg/{print $2;exit}' "$log" | tr -d '[]')
  drpy=$(awk -F': ' '/^final dRPY deg:/{print $2;exit}' "$log" | tr -d '[]')
  read -r sr sp sy <<< "${span:-NA NA NA}"; read -r rr rp ry <<< "${drpy:-NA NA NA}"
  berr=0
  grep -Eqi 'Backend.*(error|exception|failed)|IndeterminantLinearSystem|CheiralityException|terminate called|Aborted' "$log" && berr=1
  status=VALID
  if [[ $rc -ne 0 || $berr -ne 0 || -z "${states:-}" || -z "${sy:-}" ]]; then status=INVALID; fi
  if [[ "$status" == VALID ]]; then
    awk -v s="$states" -v y="$sy" 'BEGIN{exit !(s>=154 && y>=77.5)}' || status=INCOMPLETE
  fi
  echo "states=${states:-NA} yaw=${sy:-NA} dp=${dp:-NA} mm backend_error=$berr status=$status"
  echo -e "$n\t$rc\t${states:-NA}\t$berr\t${dp:-NA}\t${path:-NA}\t${exc:-NA}\t${speed:-NA}\t${sr:-NA}\t${sp:-NA}\t${sy:-NA}\t${rr:-NA}\t${rp:-NA}\t${ry:-NA}\t$status" >> "$SUMMARY"
done

echo
echo '================ v15.19 HORIZON THRESHOLD ======================='
printf '%-10s %-8s %-12s %-12s %-12s %-12s\n' NR_STATES STATUS STATES FINAL_DP_MM MAX_EXC_MM YAW_SPAN_DEG
awk -F'\t' 'NR>1 {printf "%-10s %-8s %-12s %-12s %-12s %-12s\n",$1,$15,$3,$5,$7,$11}' "$SUMMARY"

echo
echo '================ CLIFF DETECTOR =================================='
awk -F'\t' 'NR==2{pn=$1;ps=$15;pdp=$5; next} NR>2 {if(ps!=$15 || (ps=="VALID" && $15=="VALID" && pdp!="NA" && $5!="NA" && (pdp/$5>3 || $5/pdp>3))) printf "transition H%s -> H%s : status %s -> %s, dP %s -> %s mm\n",pn,$1,ps,$15,pdp,$5; pn=$1;ps=$15;pdp=$5}' "$SUMMARY"
echo "Summary: $SUMMARY"
echo "Logs/CSVs: $OUTROOT"
echo 'RESULT: COMPLETE'
