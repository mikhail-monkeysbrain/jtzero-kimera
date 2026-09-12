#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-$HOME/venv-jtzero-mav/bin/python}"
TOOL="$ROOT/tools/set_fc_param_checked.py"
DEVICE="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FLOW_FC_BAUD:-460800}"

# Current temporary physical mount estimate, BODY FRD, relative to FC IMU.
CAM_X="${JTZERO_CAM_POS_X:-0.0625}"
CAM_Y="${JTZERO_CAM_POS_Y:-0.0000}"
CAM_Z="${JTZERO_CAM_POS_Z:-0.0500}"

LUNA_X="${JTZERO_LUNA_POS_X:--0.1300}"
LUNA_Y="${JTZERO_LUNA_POS_Y:-0.0000}"
LUNA_Z="${JTZERO_LUNA_POS_Z:-0.0260}"

cat <<EOF
======================================================================
JT-ZERO — APPLY OPTICAL FLOW / RANGEFINDER SENSOR OFFSETS
======================================================================
BODY FRD: +X forward, +Y right, +Z down
Reference: FC IMU (assuming INS_POS* are zero).

OV9281 focal point:
  FLOW_POS_X = $CAM_X m
  FLOW_POS_Y = $CAM_Y m
  FLOW_POS_Z = $CAM_Z m

TF-Luna zero-range datum:
  RNGFND1_POS_X = $LUNA_X m
  RNGFND1_POS_Y = $LUNA_Y m
  RNGFND1_POS_Z = $LUNA_Z m

These are approximate current-mount values for the table validation.
New housing geometry must be re-entered after final assembly.
======================================================================
EOF

# Reference-frame guard: do not silently mix IMU-relative measurements with a
# configured CG-relative sensor frame.
for p in INS_POS1_X INS_POS1_Y INS_POS1_Z; do
  v="$("$PY" "$TOOL" "$p" --device "$DEVICE" --baud "$BAUD" --value-only)"
  echo "$p=$v"
  "$PY" - "$p" "$v" <<'PY'
import sys
name=sys.argv[1]; v=float(sys.argv[2])
if abs(v)>0.005:
    print(f"ОШИБКА: {name}={v}; sensor offsets then refer to CG/body origin, not directly to IMU.")
    raise SystemExit(1)
PY
done

echo
read -r -p "Записать эти 6 offsets в FC? [Enter=yes, Ctrl+C=no] " _

setp(){
  "$PY" "$TOOL" "$1" "$2" --device "$DEVICE" --baud "$BAUD"
}
setp FLOW_POS_X "$CAM_X"
setp FLOW_POS_Y "$CAM_Y"
setp FLOW_POS_Z "$CAM_Z"
setp RNGFND1_POS_X "$LUNA_X"
setp RNGFND1_POS_Y "$LUNA_Y"
setp RNGFND1_POS_Z "$LUNA_Z"

echo
echo "OFFSETS APPLIED AND VERIFIED"
