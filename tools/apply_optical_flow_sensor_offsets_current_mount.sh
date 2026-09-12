#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-$HOME/venv-jtzero-mav/bin/python}"
DEVICE="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FLOW_FC_BAUD:-460800}"

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
======================================================================
EOF

"$PY" - "$DEVICE" "$BAUD" "$CAM_X" "$CAM_Y" "$CAM_Z" "$LUNA_X" "$LUNA_Y" "$LUNA_Z" <<'PY'
import math, sys, time
from pymavlink import mavutil

dev=sys.argv[1]
baud=int(sys.argv[2])
vals=list(map(float,sys.argv[3:]))
targets={
    "FLOW_POS_X":vals[0],"FLOW_POS_Y":vals[1],"FLOW_POS_Z":vals[2],
    "RNGFND1_POS_X":vals[3],"RNGFND1_POS_Y":vals[4],"RNGFND1_POS_Z":vals[5],
}

m=mavutil.mavlink_connection(dev,baud=baud,source_system=191,source_component=199,autoreconnect=False)

deadline=time.monotonic()+10
fc=None
while time.monotonic()<deadline:
    hb=m.recv_match(type="HEARTBEAT",blocking=True,timeout=0.5)
    if hb is None: continue
    if int(getattr(hb,"autopilot",-1))==int(mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
        fc=(int(hb.get_srcSystem()),int(hb.get_srcComponent()))
        break
if fc is None:
    raise SystemExit("ОШИБКА: HEARTBEAT ArduPilot не найден")
sysid,compid=fc
print(f"FC sys={sysid} comp={compid}")

def pname(msg):
    p=msg.param_id
    if isinstance(p,bytes): p=p.decode("ascii","ignore")
    return str(p).rstrip("\x00")

def read_param(name,timeout=3.0):
    m.mav.param_request_read_send(sysid,compid,name.encode("ascii"),-1)
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        q=m.recv_match(type="PARAM_VALUE",blocking=True,timeout=0.25)
        if q is None: continue
        if int(q.get_srcSystem())!=sysid: continue
        if pname(q)==name:
            return float(q.param_value),int(q.param_type)
    return None

def set_checked(name,value):
    cur=read_param(name)
    if cur is None: raise SystemExit(f"ОШИБКА: {name} не прочитан")
    old,ptype=cur
    m.mav.param_set_send(sysid,compid,name.encode("ascii"),float(value),ptype)
    end=time.monotonic()+4.0
    got=None
    while time.monotonic()<end:
        r=read_param(name,0.8)
        if r is None: continue
        got=r[0]
        if math.isclose(got,value,rel_tol=0,abs_tol=max(1e-6,abs(value)*1e-5)):
            break
    if got is None or not math.isclose(got,value,rel_tol=0,abs_tol=max(1e-6,abs(value)*1e-5)):
        raise SystemExit(f"ОШИБКА: read-back {name}={got}, ожидалось {value}")
    print(f"{name}: {old:.9g} -> {got:.9g} VERIFIED")

print("Проверка INS_POS1_*:")
for name in ("INS_POS1_X","INS_POS1_Y","INS_POS1_Z"):
    r=read_param(name)
    if r is None: raise SystemExit(f"ОШИБКА: {name} не прочитан")
    v=r[0]
    print(f"  {name}={v:.9g}")
    if abs(v)>0.005:
        raise SystemExit(
            f"ОШИБКА: {name}={v}; sensor offsets тогда относятся к body/CG origin, "
            "а не напрямую к IMU. Запись offsets остановлена."
        )

print()
input("Записать эти 6 offsets в FC? [Enter=yes, Ctrl+C=no] ")

for name,value in targets.items():
    set_checked(name,value)

print()
print("OFFSETS APPLIED AND VERIFIED")
PY
