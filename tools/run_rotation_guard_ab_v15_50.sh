#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
KIMERA="/home/vio/Kimera-VIO"
SRC="${KIMERA}/src/frontend/MonoVisionImuFrontend.cpp"
BAK="/tmp/MonoVisionImuFrontend.cpp.v15_50.bak"
PARAMS="${ROOT}/params/JTZeroMonoFLU"
IMU="/home/vio/jtzero_yaw_only_v15_42.csv"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
TMP="/tmp/jtzero_v15_50"
BIN="${TMP}/replay_v15_50"
OUT="/home/vio/jtzero_rotation_guard_v15_50"
THRESHOLDS=(0.25 0.50 0.75 1.00 1.50 2.00 3.00 5.00)

cd "$ROOT"
for f in "$SRC" "$IMU" "$CAM" "$MJPG" tools/replay_mono_imu_zxy_ab_v11.cpp; do
  [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }
done
mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/report.tsv

cp "$SRC" "$BAK"
restore() {
  if [[ -s "$BAK" ]]; then
    cp "$BAK" "$SRC"
    echo "[RESTORE] Restored clean MonoVisionImuFrontend.cpp"
    cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_50_restore_build.log 2>&1 || {
      echo "[WARN] clean rebuild after restore failed; see /tmp/jtzero_v15_50_restore_build.log"
    }
  fi
}
trap restore EXIT

echo "============================================================"
echo "JT-ZERO v15.50 ROTATION-GUARD CAUSAL A/B"
echo "Dataset: v15.42"
echo "Diagnostic-only guard: when keyframe IMU rotation exceeds runtime threshold,"
echo "clear ONLY smart mono measurements for that keyframe."
echo "No LOW_DISPARITY status, no zero-velocity/no-motion prior injection."
echo "One Kimera rebuild; thresholds swept at runtime via env var."
echo "Production source restored automatically at exit."
echo "============================================================"

# Insert a diagnostic-only runtime guard immediately after smart mono measurements are packed.
python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); s=p.read_text()
if '#include <cstdlib>' not in s:
    s=s.replace('#include <memory>\n', '#include <cstdlib>\n#include <memory>\n', 1)
needle='    getSmartMonoMeasurements(mono_frame_k_, &smart_mono_measurements);\n'
insert='''    getSmartMonoMeasurements(mono_frame_k_, &smart_mono_measurements);\n\n    // JT-ZERO v15.50 diagnostic-only runtime guard.\n    // It suppresses only new monocular smart measurements when the IMU\n    // indicates a sufficiently large keyframe-to-keyframe rotation.\n    // It deliberately does NOT change TrackingStatus and therefore does NOT\n    // inject LOW_DISPARITY zero-velocity or no-motion priors.\n    if (const char* jtzero_guard = std::getenv("JTZERO_DIAG_ROT_GUARD_DEG")) {\n      const double threshold_deg = std::atof(jtzero_guard);\n      const double rot_deg =\n          gtsam::Rot3::Logmap(keyframe_R_cur_frame).norm() * 180.0 / M_PI;\n      if (threshold_deg >= 0.0 && rot_deg >= threshold_deg) {\n        VLOG(1) << "JTZERO_V15_50_ROT_GUARD rot_deg=" << rot_deg\n                << " threshold_deg=" << threshold_deg\n                << " cleared_measurements=" << smart_mono_measurements.size();\n        smart_mono_measurements.clear();\n      }\n    }\n'''
if needle not in s:
    raise SystemExit('[FATAL] patch anchor not found')
s=s.replace(needle,insert,1)
p.write_text(s)
PY

grep -q 'JTZERO_DIAG_ROT_GUARD_DEG' "$SRC" || { echo "[FATAL] guard patch failed"; exit 1; }

echo "[BUILD] Kimera diagnostic build..."
cmake --build "$KIMERA/build" -j"$(nproc)"

echo "[BUILD] replay binary..."
g++ -std=c++17 -O2 \
  -I"$ROOT/tools" \
  -I"$KIMERA/include" -I"$KIMERA/build" \
  -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread \
  $(pkg-config --libs opencv4)

printf 'MODE\tTHRESHOLD_DEG\tRC\tSTATES\tFINAL_DP_MM\tMAXEXC_MM\tDROLL\tDPITCH\tDYAW\n' > "$OUT/report.tsv"

run_one() {
  local mode="$1" threshold="$2" tag="$3"
  local log="$OUT/${tag}.log"
  echo "[RUN] $mode threshold=$threshold"
  set +e
  if [[ "$mode" == "CONTROL" ]]; then
    env -u JTZERO_DIAG_ROT_GUARD_DEG \
      LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
      "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  else
    JTZERO_DIAG_ROT_GUARD_DEG="$threshold" \
      LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
      "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  fi
  local rc=$?
  set -e
  local states dp mx rpy vr vp vy
  states=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  rpy=$(sed -n 's/.*final dRPY deg: \[\([^]]*\)\].*/\1/p' "$log" | tail -1)
  read -r vr vp vy <<<"${rpy:-NA NA NA}"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$mode" "$threshold" "$rc" "$states" "${dp:-NA}" "${mx:-NA}" "$vr" "$vp" "$vy" >> "$OUT/report.tsv"
  echo "      rc=$rc states=$states dP=${dp:-NA} mm maxexc=${mx:-NA} mm dRPY=[$vr $vp $vy]"
}

run_one CONTROL NA control
for t in "${THRESHOLDS[@]}"; do
  tag="guard_${t/./p}"
  run_one GUARD "$t" "$tag"
done

echo
echo "================ v15.50 SWEEP ================"
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

echo
python3 - "$OUT/report.tsv" <<'PY'
import csv,sys,math
p=sys.argv[1]
rows=list(csv.DictReader(open(p),delimiter='\t'))
valid=[]
for r in rows:
    try:
        if int(r['RC'])==0 and int(r['STATES'])>=140:
            valid.append((r['MODE'],r['THRESHOLD_DEG'],float(r['FINAL_DP_MM']),int(r['STATES'])))
    except: pass
ctrl=next((x for x in valid if x[0]=='CONTROL'),None)
guards=[x for x in valid if x[0]=='GUARD']
if not ctrl:
    verdict='INVALID_CONTROL'
    print('V15_50_VERDICT='+verdict);print('RESULT: COMPLETE');raise SystemExit
best=min(guards,key=lambda x:x[2]) if guards else None
print('CONTROL_DP_MM=%.3f CONTROL_STATES=%d'%(ctrl[2],ctrl[3]))
if best:
    print('BEST_GUARD_THRESHOLD_DEG=%s BEST_GUARD_DP_MM=%.3f BEST_GUARD_STATES=%d'%(best[1],best[2],best[3]))
    improvement=ctrl[2]/best[2] if best[2]>1e-9 else float('inf')
    print('IMPROVEMENT_FACTOR=%.3f'%improvement)
    if best[2] <= 0.25*ctrl[2] and best[2] <= 75:
        verdict='ROTATION_VISUAL_MEASUREMENTS_CAUSALLY_DRIVE_FALSE_TRANSLATION'
    elif best[2] <= 0.60*ctrl[2]:
        verdict='ROTATION_VISUAL_MEASUREMENTS_SIGNIFICANT_AMPLIFIER'
    else:
        verdict='ROTATION_ONLY_GUARD_NOT_SUFFICIENT'
else:
    verdict='NO_VALID_GUARD_RUNS'
print('V15_50_VERDICT='+verdict)
print('NOTE=Diagnostic causal guard only; NOT a production mitigation because real rotation+translation must remain observable.')
print('RESULT: COMPLETE')
PY

echo "Logs: $OUT"
echo "Source will now be restored and Kimera rebuilt clean by trap."
