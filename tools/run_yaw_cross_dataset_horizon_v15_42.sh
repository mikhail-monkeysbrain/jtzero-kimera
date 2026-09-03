#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
TMP=/tmp/jtzero_v15_42_tools
REC_SRC="$TMP/record_mono_imu_yaw_only_gui_v15_42.cpp"
REC_BIN=/tmp/record_mono_imu_yaw_only_gui_v15_42
REPLAY_BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_42
OUT=/home/vio/jtzero_yaw_cross_dataset_v15_42
REC_LOG="$OUT/physical_capture.txt"

OLD_IMU=/home/vio/jtzero_yaw_only_v13.csv
OLD_CAM=/home/vio/jtzero_yaw_only_v13_camera.csv
OLD_MJPG=/home/vio/jtzero_yaw_only_v13.mjpg
NEW_IMU=/home/vio/jtzero_yaw_only_v15_42.csv
NEW_CAM=/home/vio/jtzero_yaw_only_v15_42_camera.csv
NEW_MJPG=/home/vio/jtzero_yaw_only_v15_42.mjpg
NEW_ATT=/home/vio/jtzero_yaw_only_v15_42_attitude.csv

CAM_BY_ID=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0

mkdir -p "$OUT"
for f in "$OLD_IMU" "$OLD_CAM" "$OLD_MJPG"; do
  [[ -s "$f" ]] || { echo "FATAL: missing baseline dataset file $f" >&2; exit 2; }
done
[[ -e "$CAM_BY_ID" ]] || { echo "FATAL: camera by-id missing: $CAM_BY_ID" >&2; exit 3; }
REAL_CAM="$(readlink -f "$CAM_BY_ID")"
[[ -c "$REAL_CAM" ]] || { echo "FATAL: resolved camera is not a video device: $REAL_CAM" >&2; exit 4; }
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then
  echo 'FATAL: /home/vio/Kimera-VIO/src/backend/VioBackend.cpp is dirty' >&2
  exit 5
fi

echo '============================================================'
echo 'JT-ZERO v15.42 CROSS-DATASET YAW HORIZON VALIDATION'
echo '1) Новый replayable yaw-only тест с русским GUI'
echo '2) Offline H28/H29/H30/H32/H35/H40/H50 на старом и новом dataset'
echo '3) Ищем один horizon, проходящий оба dataset'
echo '============================================================'
echo "[CAM] $CAM_BY_ID -> $REAL_CAM"

# Build a temporary recorder from the already validated v13 Russian GUI.
# Only temporary copies are changed: stable camera path and v15.42 output names.
rm -rf "$TMP"; mkdir -p "$TMP"
cp "$ROOT/tools/camera_imu_extrinsics_logger.cpp" "$TMP/"
cp "$ROOT/tools/camera_imu_timestamp_policy.hpp" "$TMP/"
cp "$ROOT/tools/record_mono_imu_yaw_only_gui_v13.cpp" "$REC_SRC"

sed -i "s#constexpr const char\* CAMERA_DEVICE=\"/dev/video0\";#constexpr const char* CAMERA_DEVICE=\"$REAL_CAM\";#" "$TMP/camera_imu_extrinsics_logger.cpp"
sed -i 's#/home/vio/jtzero_yaw_only_v13.csv#/home/vio/jtzero_yaw_only_v15_42.csv#' "$REC_SRC"
sed -i 's#/home/vio/jtzero_yaw_only_v13_camera.csv#/home/vio/jtzero_yaw_only_v15_42_camera.csv#' "$REC_SRC"
sed -i 's#/home/vio/jtzero_yaw_only_v13.mjpg#/home/vio/jtzero_yaw_only_v15_42.mjpg#' "$REC_SRC"
sed -i 's#/home/vio/jtzero_yaw_only_v13_attitude.csv#/home/vio/jtzero_yaw_only_v15_42_attitude.csv#' "$REC_SRC"

# Verify that the temporary substitutions actually happened.
grep -Fq "CAMERA_DEVICE=\"$REAL_CAM\"" "$TMP/camera_imu_extrinsics_logger.cpp" || { echo 'FATAL: camera substitution failed'; exit 6; }
grep -Fq "$NEW_IMU" "$REC_SRC" || { echo 'FATAL: output substitution failed'; exit 7; }

free_kb=$(df -Pk /home/vio | awk 'NR==2{print $4}')
if (( free_kb < 1200000 )); then
  echo "FATAL: insufficient free space on /home/vio: ${free_kb} KiB; need at least 1200000 KiB" >&2
  exit 8
fi

echo '[1/5] Build clean Kimera + recorder + replay'
cmake --build "$K/build" -j2 --target kimera_vio

g++ -std=c++17 -O2 "$REC_SRC" -o "$REC_BIN" \
  -I"$TMP" -I"$ROOT/tools" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  -lpthread $(pkg-config --libs opencv4)

g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$REPLAY_BIN" \
  -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)

echo '[2/5] Physical replayable yaw-only capture'
echo 'GUI: 2 c калибровка нуля -> 10 c покой -> 15 c YAW примерно +90° -> 10 c покой.'
echo 'ROLL/PITCH держите в зелёной зоне.'
rm -f "$NEW_IMU" "$NEW_CAM" "$NEW_MJPG" "$NEW_ATT" "$REC_LOG"
set +e
"$REC_BIN" 2>&1 | tee "$REC_LOG"
rec_rc=${PIPESTATUS[0]}
set -e
if (( rec_rc != 0 )); then
  echo "RESULT: INVALID_PHYSICAL_CAPTURE rc=$rec_rc"
  exit "$rec_rc"
fi
for f in "$NEW_IMU" "$NEW_CAM" "$NEW_MJPG" "$NEW_ATT"; do
  [[ -s "$f" ]] || { echo "RESULT: INVALID_MISSING_CAPTURE_FILE $f"; exit 20; }
done
if ! grep -q '^RESULT: PASS$' "$REC_LOG"; then
  echo 'RESULT: INVALID_PHYSICAL_YAW_QUALITY'
  echo 'Offline matrix intentionally not started.'
  exit 21
fi
printf '[CAPTURE] IMU=%s bytes CAM=%s bytes MJPG=%s bytes ATT=%s bytes\n' \
  "$(stat -c%s "$NEW_IMU")" "$(stat -c%s "$NEW_CAM")" "$(stat -c%s "$NEW_MJPG")" "$(stat -c%s "$NEW_ATT")"

metric(){ awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$1"; }
pathmetric(){ awk -F': ' '/^path mm:/{print $2;exit}' "$1"; }
maxmetric(){ awk -F': ' '/^max excursion mm:/{print $2;exit}' "$1"; }
statesmetric(){ awk -F': ' '/^backend states:/{print $2;exit}' "$1"; }

run_one(){
  local ds="$1" h="$2" imu="$3" cam="$4" mjpg="$5"
  local p="$OUT/params_${ds}_H${h}"
  local log="$OUT/${ds}_H${h}.txt"
  rm -rf "$p"; cp -a "$ROOT/params/JTZeroMonoFLU" "$p"
  sed -i -E "s/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: $h/" "$p/BackendParams.yaml"
  rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
  set +e
  LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$REPLAY_BIN" "$p" CURRENT "$imu" "$cam" "$mjpg" >"$log" 2>&1
  local rc=$?
  set -e
  local dp st path mx
  dp="$(metric "$log")"; st="$(statesmetric "$log")"; path="$(pathmetric "$log")"; mx="$(maxmetric "$log")"
  printf '%-5s H=%-3s rc=%d states=%-4s dP=%-10s path=%-10s maxexc=%s\n' \
    "$ds" "$h" "$rc" "${st:-NA}" "${dp:-NA}" "${path:-NA}" "${mx:-NA}"
  [[ $rc -eq 0 && -n "$dp" ]] || { echo "FATAL replay $ds H$h"; tail -100 "$log"; exit 30; }
}

echo '[3/5] Baseline yaw_only_v13 horizon matrix'
for h in 28 29 30 32 35 40 50; do run_one OLD "$h" "$OLD_IMU" "$OLD_CAM" "$OLD_MJPG"; done

echo '[4/5] New v15.42 horizon matrix'
for h in 28 29 30 32 35 40 50; do run_one NEW "$h" "$NEW_IMU" "$NEW_CAM" "$NEW_MJPG"; done

echo '[5/5] Cross-dataset report'
printf '\n%-4s %-4s %-8s %-12s %-12s %-12s\n' DS H STATES FINAL_DP_MM PATH_MM MAXEXC_MM
for ds in OLD NEW; do
  for h in 28 29 30 32 35 40 50; do
    log="$OUT/${ds}_H${h}.txt"
    printf '%-4s %-4s %-8s %-12s %-12s %-12s\n' "$ds" "$h" \
      "$(statesmetric "$log")" "$(metric "$log")" "$(pathmetric "$log")" "$(maxmetric "$log")"
  done
done

python3 - "$OUT" <<'PY'
import pathlib,re,sys
out=pathlib.Path(sys.argv[1])
horizons=[28,29,30,32,35,40,50]

def read(ds,h):
    s=(out/f'{ds}_H{h}.txt').read_text(errors='replace')
    def grab(p,default=float('inf')):
        m=re.search(p,s,re.M)
        return float(m.group(1)) if m else default
    st=grab(r'^backend states:\s*([0-9]+)',0)
    dp=grab(r'^final \|dP\| mm:\s*([0-9.eE+-]+)')
    mx=grab(r'^max excursion mm:\s*([0-9.eE+-]+)')
    return int(st),dp,mx

print('\n================ V15.42 CROSS-DATASET GATE ================')
print('Gate per dataset: full replay + final false translation <= 35 mm.')
rows=[]
for h in horizons:
    os,od,om=read('OLD',h)
    ns,nd,nm=read('NEW',h)
    # Do not force equal state counts across datasets; require each replay to have a meaningful full trajectory.
    complete=(os>=100 and ns>=100)
    op=od<=35.0
    np=nd<=35.0
    both=complete and op and np
    rows.append((h,os,od,ns,nd,both))
    print(f'H{h}: OLD={od:.3f} mm ({"PASS" if op else "FAIL"})  NEW={nd:.3f} mm ({"PASS" if np else "FAIL"})  BOTH={"PASS" if both else "FAIL"}')

passing=[r for r in rows if r[-1]]
if passing:
    # Prefer the smallest passing horizon >=30, but only after both datasets pass.
    cand=[r for r in passing if r[0]>=30]
    pick=(cand or passing)[0]
    print(f'CROSS_DATASET_FIXED_HORIZON_CANDIDATE=H{pick[0]}')
    print('ITEM_11_STATUS=FIXED_HORIZON_CANDIDATE_NEEDS_GENERAL_MOTION_VALIDATION')
else:
    print('CROSS_DATASET_FIXED_HORIZON_CANDIDATE=NONE')
    print('ITEM_11_STATUS=FIXED_HORIZON_MITIGATION_REJECTED')
    print('NEXT_DIRECTION=ROBUST_LOW_PARALLAX_ROTATION_HANDLING')
print('RESULT: COMPLETE')
PY

echo "New replayable dataset: $NEW_IMU $NEW_CAM $NEW_MJPG $NEW_ATT"
echo "Full logs: $OUT"
