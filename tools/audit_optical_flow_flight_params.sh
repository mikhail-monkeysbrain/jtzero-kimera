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

PARAMS=(
  FLOW_TYPE
  FLOW_OPTIONS
  FLOW_ORIENT_YAW
  FLOW_FXSCALER
  FLOW_FYSCALER
  EK3_FLOW_DELAY
  EK3_FLOW_MAX
  EK3_SRC1_POSXY
  EK3_SRC1_VELXY
  EK3_SRC1_POSZ
  EK3_SRC1_VELZ
  EK3_SRC1_YAW
)

OUT="$("$BIN" "$DEVICE" "$BAUD" "$SYSID" "$COMPID" read "${PARAMS[@]}")"

echo "======================================================================"
echo "JT-ZERO — OPTICAL FLOW FLIGHT PREFLIGHT"
echo "======================================================================"
echo "FC: $DEVICE @ $BAUD"
echo "Reader: native C++ MAVLink batch utility"
echo "Параметры только ЧИТАЮТСЯ. Ничего не изменяется."
echo
echo "$OUT"
echo

python3 - "$OUT" <<'PY'
import math,sys

expected={
  "FLOW_TYPE":5,
  "FLOW_OPTIONS":0,
  "FLOW_ORIENT_YAW":0,
  "FLOW_FXSCALER":0,
  "FLOW_FYSCALER":0,
  "EK3_FLOW_DELAY":0,
  "EK3_FLOW_MAX":4.0,
  "EK3_SRC1_POSXY":0,
  "EK3_SRC1_VELXY":5,
  "EK3_SRC1_POSZ":2,
  "EK3_SRC1_VELZ":0,
  "EK3_SRC1_YAW":0,
}
order=list(expected)

vals={}
for line in sys.argv[1].splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    k=k.strip(); v=v.strip()
    try: vals[k]=float(v)
    except ValueError: pass

fail=False
for name in order:
    if name not in vals:
        print(f"FAIL  {name:<18} : не прочитан")
        fail=True
        continue
    v=vals[name]; e=float(expected[name])
    ok=math.isclose(v,e,rel_tol=0.0,abs_tol=max(1e-6,abs(e)*1e-5))
    if ok:
        print(f"PASS  {name:<18} = {v:g}")
    else:
        print(f"FAIL  {name:<18} = {v:g} expected={e:g}")
        fail=True

print()
if fail:
    print("RESULT: FAIL")
    print("Flight launcher НЕ запускать до устранения несоответствий.")
    raise SystemExit(1)

print("RESULT: PASS")
print("FC source configuration соответствует проверенному OpticalFlow flight-контуру.")
PY
