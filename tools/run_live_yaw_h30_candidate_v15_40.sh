#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$ROOT/tools/live_mono_imu_yaw_only_hud_v15_4_utf8.cpp"
BIN=/tmp/live_mono_imu_yaw_h30_candidate_v15_40
P=/tmp/JTZeroMonoFLU_v15_40_H30
CSV=/home/vio/jtzero_live_yaw_only_hud_v15_3.csv
OUT=/home/vio/jtzero_live_yaw_h30_v15_40.csv
[[ -f "$SRC" ]] || { echo "FATAL missing $SRC" >&2; exit 2; }
[[ -d "$ROOT/params/JTZeroMonoFLU" ]] || { echo 'FATAL missing params/JTZeroMonoFLU' >&2; exit 2; }
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then echo 'FATAL dirty Kimera VioBackend.cpp' >&2; exit 3; fi
rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"
sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 30/' "$P/BackendParams.yaml"
echo '============================================================'
echo 'JT-ZERO v15.40 LIVE H30 CANDIDATE VALIDATION'
echo 'GUI: 10 s ПОКОЙ -> 15 s YAW ~90 deg -> 10 s ПОКОЙ'
echo 'Production source is NOT modified. Temporary params use nr_states=30.'
echo '============================================================'
grep -nE '^[[:space:]]*nr_states:' "$P/BackendParams.yaml" || true

echo '[1/3] Build clean Kimera + UTF-8 live HUD'
cmake --build "$K/build" -j2 --target kimera_vio
g++ -std=c++17 -O2 "$SRC" -o "$BIN" \
 -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
 $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib \
 -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4) -lopencv_freetype

echo '[2/3] Run physical GUI test'
echo 'Во время YAW вращайте только вокруг вертикальной оси примерно до 90 градусов.'
echo 'В фазах ПОКОЙ стенд не двигать.'
rm -f "$CSV" "$OUT"
set +e
LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P"
rc=$?
set -e
[[ $rc -eq 0 ]] || { echo "RESULT: LIVE_RUN_FAILED rc=$rc"; exit "$rc"; }
[[ -s "$CSV" ]] || { echo "RESULT: MISSING_CSV $CSV"; exit 4; }
cp "$CSV" "$OUT"

echo '[3/3] Analyze H30 live trajectory'
python3 - "$OUT" <<'PY'
import csv,math,sys
p=sys.argv[1]
r=list(csv.DictReader(open(p,newline='')))
if not r:
 print('RESULT: INVALID_EMPTY_CSV'); raise SystemExit(5)
# The recorder labels the initial 10 s as INIT, then YAW, then SETTLE.
def vals(x): return [float(z[x]) for z in r]
px,py,pz=map(vals,('px','py','pz'))
roll,pitch,yaw=map(vals,('roll_deg','pitch_deg','yaw_deg'))
x0,y0,z0=px[0],py[0],pz[0]
d=[math.hypot((x-x0)*1000,(y-y0)*1000) for x,y in zip(px,py)]
dp=[math.sqrt(((x-x0)*1000)**2+((y-y0)*1000)**2+((z-z0)*1000)**2) for x,y,z in zip(px,py,pz)]
wrap=lambda a:(a+180)%360-180
r0,p0,y0a=roll[0],pitch[0],yaw[0]
dr=[abs(wrap(x-r0)) for x in roll]; dpt=[abs(wrap(x-p0)) for x in pitch]; dy=[abs(wrap(x-y0a)) for x in yaw]
ph={}
for i,z in enumerate(r): ph.setdefault(z['phase'],[]).append(i)
print('================ V15.40 LIVE H30 ================')
print(f'rows={len(r)} keyframes={r[0]["keyframe"]}..{r[-1]["keyframe"]}')
for name in ('INIT','YAW','SETTLE'):
 ix=ph.get(name,[])
 if not ix: print(f'{name}: MISSING'); continue
 print(f'{name}: n={len(ix)} maxXY={max(d[i] for i in ix):.3f} mm maxDP={max(dp[i] for i in ix):.3f} mm max|dR|={max(dr[i] for i in ix):.3f} deg max|dPITCH|={max(dpt[i] for i in ix):.3f} deg max|dYAW|={max(dy[i] for i in ix):.3f} deg')
finalxy=d[-1]; maxxy=max(d); finaldp=dp[-1]
print(f'FINAL_XY_MM={finalxy:.3f}')
print(f'MAX_XY_MM={maxxy:.3f}')
print(f'FINAL_DP_MM={finaldp:.3f}')
# Dataset-local candidate threshold inherited from v15.39. Physical motion quality must also be inspected from GUI/FC attitude.
print('H30_LIVE_XY_GATE=' + ('PASS' if finalxy<=35.0 and maxxy<=50.0 else 'FAIL'))
print('NOTE: This gate validates false VIO translation only. Accept the run only if GUI showed approximately 90 deg yaw and roll/pitch stayed inside marked zones.')
print('RESULT: COMPLETE')
PY
echo "Saved CSV: $OUT"
