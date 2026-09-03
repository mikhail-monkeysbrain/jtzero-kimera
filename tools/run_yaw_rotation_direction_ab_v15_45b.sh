#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
ATT="/home/vio/jtzero_yaw_only_v15_42_attitude.csv"
TMP="/tmp/jtzero_v15_45b"
SRC0="$ROOT/tools/analyze_yaw_rotation_parallax_v15_45.cpp"
SRCF="$TMP/analyze_forward.cpp"
SRCI="$TMP/analyze_inverse.cpp"
BINF="$TMP/analyze_forward"
BINI="$TMP/analyze_inverse"
OUTF="/home/vio/jtzero_rotation_parallax_v15_45b_forward.csv"
OUTI="/home/vio/jtzero_rotation_parallax_v15_45b_inverse.csv"
LOGF="$TMP/forward.log"
LOGI="$TMP/inverse.log"

cd "$ROOT"
for f in "$CAM" "$MJPG" "$ATT" "$SRC0"; do
  [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }
done
mkdir -p "$TMP"
cp "$SRC0" "$SRCF"
cp "$SRC0" "$SRCI"

# v15.45 used C1<-C0 = RBC^T * RWB1^T * RWB0 * RBC.
# This A/B runs the exact inverse hypothesis as well. Nothing else changes.
perl -0777 -i -pe 's/cv::Matx33d RC1C0=RBC\.t\(\)\*RWB1\.t\(\)\*RWB0\*RBC;/cv::Matx33d RC1C0=RBC.t()*RWB0.t()*RWB1*RBC;/g' "$SRCI"
grep -q 'RBC.t()\*RWB0.t()\*RWB1\*RBC' "$SRCI" || { echo "[FATAL] inverse transform patch failed"; exit 1; }

g++ -std=c++17 -O2 $(pkg-config --cflags opencv4) "$SRCF" -o "$BINF" $(pkg-config --libs opencv4)
g++ -std=c++17 -O2 $(pkg-config --cflags opencv4) "$SRCI" -o "$BINI" $(pkg-config --libs opencv4)

echo "============================================================"
echo "JT-ZERO v15.45b ROTATION DIRECTION A/B"
echo "Same v15.42 frames/tracks/FC ATTITUDE; only relative-rotation direction changes."
echo "No production source/parameter changes."
echo "============================================================"

"$BINF" "$CAM" "$MJPG" "$ATT" "$OUTF" | tee "$LOGF"
echo
"$BINI" "$CAM" "$MJPG" "$ATT" "$OUTI" | tee "$LOGI"

python3 - "$LOGF" "$LOGI" <<'PY'
import re,sys

def parse(path):
    s=open(path,errors='replace').read()
    m=re.search(r'^ROTATION pairs=.*?raw_flow=([0-9.eE+-]+)px rot_pred=([0-9.eE+-]+)px residual=([0-9.eE+-]+)px.*?residual/raw=([0-9.eE+-]+).*?H_inlier=([0-9.eE+-]+).*?H_err=([0-9.eE+-]+)px',s,re.M)
    if not m: raise SystemExit('Cannot parse ROTATION line from '+path)
    return tuple(map(float,m.groups()))

f=parse(sys.argv[1]); i=parse(sys.argv[2])
print('\n================ v15.45b A/B ================')
print('MODE      RAW_PX  ROT_PRED_PX  RESIDUAL_PX  RESID/RAW  H_INLIER  H_ERR_PX')
print('FORWARD   %7.3f  %11.3f  %11.3f  %9.3f  %8.3f  %8.3f'%f)
print('INVERSE   %7.3f  %11.3f  %11.3f  %9.3f  %8.3f  %8.3f'%i)
if i[2] < 0.5*f[2] and i[3] < 0.7:
    verdict='V15_45_ROTATION_DIRECTION_WAS_REVERSED'
elif f[2] < 0.5*i[2] and f[3] < 0.7:
    verdict='V15_45_FORWARD_DIRECTION_CONFIRMED'
elif min(f[3],i[3]) < 0.7:
    verdict='ROTATION_DOMINATED_WITH_DIRECTION_AMBIGUITY'
else:
    verdict='NON_ROTATIONAL_RESIDUAL_REMAINS_IN_BOTH_DIRECTIONS'
print('V15_45B_VERDICT='+verdict)
print('RESULT: COMPLETE')
PY

echo "FORWARD_CSV=$OUTF"
echo "INVERSE_CSV=$OUTI"
