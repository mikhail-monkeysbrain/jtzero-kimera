#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
SRC_ORIG="$ROOT/tools/analyze_yaw_rotation_parallax_v15_45.cpp"
TMP="/tmp/jtzero_v15_46"
SRC="$TMP/analyze_v15_46.cpp"
BIN="$TMP/analyze_v15_46"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
OUT="/home/vio/jtzero_rotation_timeshift_v15_46"
OFFSETS_MS=(-100 -80 -60 -40 -20 0 20 40 60 80 100)

mkdir -p "$TMP" "$OUT"
cp "$SRC_ORIG" "$SRC"

# Diagnostic-only patch:
# 1) use the correct inverse relative camera rotation from v15.45b;
# 2) shift ATTITUDE lookup relative to camera timestamps by JTZERO_ATT_OFFSET_NS.
perl -0777 -i -pe 's/cv::Matx33d RC1C0=RBC\.t\(\)\*RWB1\.t\(\)\*RWB0\*RBC;/cv::Matx33d RC1C0=(RBC.t()*RWB1.t()*RWB0*RBC).t();/g' "$SRC"
perl -0777 -i -pe 's/const auto&a0=nearestAtt\(A,C\[i-1\]\.ts,ah0\); const auto&a1=nearestAtt\(A,C\[i\]\.ts,ah1\);/const char* oe=std::getenv("JTZERO_ATT_OFFSET_NS"); const int64_t ofs=oe?std::stoll(oe):0; const auto\&a0=nearestAtt(A,C[i-1].ts+ofs,ah0); const auto\&a1=nearestAtt(A,C[i].ts+ofs,ah1);/g' "$SRC"

grep -q 'JTZERO_ATT_OFFSET_NS' "$SRC" || { echo "[FATAL] time-offset patch failed"; exit 1; }

g++ -std=c++17 -O2 $(pkg-config --cflags opencv4) "$SRC" -o "$BIN" $(pkg-config --libs opencv4)

printf 'OFFSET_MS\tROT_RAW_PX\tROT_PRED_PX\tROT_RESIDUAL_PX\tROT_RESID_RAW\tH_INLIER\tH_ERR_PX\n' > "$OUT/report.tsv"

echo "============================================================"
echo "JT-ZERO v15.46 CAMERA<->ATTITUDE TIME OFFSET SWEEP"
echo "Dataset: v15.42"
echo "Correct inverse rotation direction from v15.45b"
echo "No production source/parameter changes"
echo "============================================================"

for ms in "${OFFSETS_MS[@]}"; do
  ns=$((ms*1000000))
  log="$OUT/offset_${ms}ms.log"
  csv="$OUT/offset_${ms}ms.csv"
  echo "[RUN] ATTITUDE offset=${ms} ms"
  JTZERO_ATT_OFFSET_NS="$ns" "$BIN" "$CAM" "$MJPG" "$ATT" "$csv" > "$log" 2>&1
  line=$(grep '^ROTATION pairs=' "$log" | tail -1)
  raw=$(sed -n 's/.*raw_flow=\([0-9.]*\)px.*/\1/p' <<<"$line")
  pred=$(sed -n 's/.*rot_pred=\([0-9.]*\)px.*/\1/p' <<<"$line")
  res=$(sed -n 's/.*residual=\([0-9.]*\)px.*/\1/p' <<<"$line")
  rat=$(sed -n 's/.*residual\/raw=\([0-9.]*\).*/\1/p' <<<"$line")
  hin=$(sed -n 's/.*H_inlier=\([0-9.]*\).*/\1/p' <<<"$line")
  herr=$(sed -n 's/.*H_err=\([0-9.]*\)px.*/\1/p' <<<"$line")
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$ms" "$raw" "$pred" "$res" "$rat" "$hin" "$herr" >> "$OUT/report.tsv"
  echo "      raw=$raw px pred=$pred px residual=$res px ratio=$rat"
done

echo
echo "================ v15.46 SWEEP ================"
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

python3 - "$OUT/report.tsv" <<'PY'
import csv, sys
rows=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))
vals=[]
for r in rows:
    try: vals.append((int(r['OFFSET_MS']),float(r['ROT_RESIDUAL_PX']),float(r['ROT_RESID_RAW'])))
    except: pass
if not vals:
    print('V15_46_VERDICT=NO_VALID_RESULTS')
    print('RESULT: COMPLETE')
    raise SystemExit
best=min(vals,key=lambda x:x[1])
zero=min(vals,key=lambda x:abs(x[0]))
print(f'BEST_OFFSET_MS={best[0]} BEST_RESIDUAL_PX={best[1]:.3f} BEST_RATIO={best[2]:.3f}')
print(f'ZERO_OFFSET_RESIDUAL_PX={zero[1]:.3f} ZERO_OFFSET_RATIO={zero[2]:.3f}')
if abs(best[0])>=20 and best[1] < 0.70*zero[1]:
    verdict='CAMERA_ATTITUDE_TIME_OFFSET_IS_SIGNIFICANT'
elif best[1] < 0.5:
    verdict='ROTATION_MODEL_EXPLAINS_MOST_FLOW_AFTER_TIMING_CHECK'
else:
    verdict='TIMING_NOT_DOMINANT_RESIDUAL_REMAINS'
print('V15_46_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo "Logs: $OUT"
