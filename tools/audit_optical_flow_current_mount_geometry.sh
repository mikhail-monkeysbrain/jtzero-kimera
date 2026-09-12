#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
DEVICE="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FLOW_FC_BAUD:-460800}"
SYSID="${JTZERO_FC_SYSID:-1}"
COMPID="${JTZERO_FC_COMPID:-1}"

BIN="/tmp/jtzero_fc_param_batch_checked"
SRC="$ROOT/tools/fc_param_batch_checked.cpp"

MAVLINK_INC=""
for d in   "$KIMERA_ROOT/third_party/mavlink"   "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0"   "/usr/local/include/mavlink/v2.0"
do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }

g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member   $MAVLINK_INC "$SRC" -o "$BIN"

OUT="$("$BIN" "$DEVICE" "$BAUD" "$SYSID" "$COMPID" read   INS_POS1_X INS_POS1_Y INS_POS1_Z   FLOW_POS_X FLOW_POS_Y FLOW_POS_Z   RNGFND1_POS_X RNGFND1_POS_Y RNGFND1_POS_Z)"

echo "======================================================================"
echo "JT-ZERO — CURRENT-MOUNT GEOMETRY AUDIT"
echo "======================================================================"
echo "$OUT"
echo

python3 - "$OUT" <<'PY'
import math,sys
text=sys.argv[1]
vals={}
for line in text.splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    k=k.strip(); v=v.strip()
    try: vals[k]=float(v)
    except ValueError: pass

exp={
 "INS_POS1_X":0.0, "INS_POS1_Y":0.0, "INS_POS1_Z":0.0,
 "FLOW_POS_X":0.0625, "FLOW_POS_Y":0.0, "FLOW_POS_Z":0.0500,
 "RNGFND1_POS_X":-0.0150, "RNGFND1_POS_Y":0.0, "RNGFND1_POS_Z":0.0710,
}
failed=[]
for k,e in exp.items():
    if k not in vals:
        failed.append(f"{k}: MISSING")
        continue
    v=vals[k]
    tol=max(1e-6,abs(e)*1e-5)
    if not math.isclose(v,e,rel_tol=0.0,abs_tol=tol):
        failed.append(f"{k}: {v} expected {e}")

if failed:
    print("RESULT: FAIL")
    for s in failed: print("  "+s)
    raise SystemExit(1)
print("RESULT: PASS")
print("Current temporary-mount sensor offsets match the validated bench geometry.")
PY
