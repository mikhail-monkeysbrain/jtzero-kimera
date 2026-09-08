#!/usr/bin/env python3
from pymavlink import mavutil
from pathlib import Path
import glob, time, sys

CANDIDATES = [
    "/dev/ttyAMA0",
    "/dev/serial0",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]
for p in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyAMA*")):
    if p not in CANDIDATES:
        CANDIDATES.append(p)

BAUDS = [115200, 57600]

def connect():
    last = []
    for dev in CANDIDATES:
        if not Path(dev).exists():
            continue
        for baud in BAUDS:
            try:
                print(f"[TRY] {dev} @ {baud}")
                m = mavutil.mavlink_connection(dev, baud=baud, autoreconnect=False)
                hb = m.wait_heartbeat(timeout=2.5)
                if hb:
                    print(f"[OK] HEARTBEAT on {dev} @ {baud}, sys={m.target_system}, comp={m.target_component}")
                    return m, dev, baud
            except Exception as e:
                last.append((dev, baud, str(e)))
    print("Не удалось найти FC по MAVLink.")
    for x in last[-8:]:
        print(" ", x)
    sys.exit(2)

m, dev, baud = connect()

print("\n=== RNGFND* PARAMETERS ===")
m.mav.param_request_list_send(m.target_system, m.target_component)

params = {}
deadline = time.time() + 8.0
last_rx = time.time()
while time.time() < deadline:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
    if msg is None:
        if time.time() - last_rx > 1.5 and params:
            break
        continue
    last_rx = time.time()
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="ignore")
    pid = pid.rstrip("\x00")
    if pid.startswith("RNGFND"):
        params[pid] = float(msg.param_value)

if not params:
    print("Параметры RNGFND* не получены.")
else:
    for k in sorted(params):
        print(f"{k:24s} = {params[k]:.6f}")

print("\n=== CURRENT DISTANCE_SENSOR ===")
# ask for data stream; FC may already stream it
try:
    m.mav.request_data_stream_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )
except Exception:
    pass

vals = []
deadline = time.time() + 5.0
while time.time() < deadline and len(vals) < 20:
    msg = m.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=0.5)
    if msg is None:
        continue
    vals.append((int(msg.current_distance), int(msg.min_distance), int(msg.max_distance), int(msg.orientation), int(getattr(msg, "id", 0))))

if not vals:
    print("DISTANCE_SENSOR не получен.")
else:
    print("current_cm  min_cm  max_cm  orientation  id")
    for v in vals:
        print(f"{v[0]:10d} {v[1]:7d} {v[2]:7d} {v[3]:12d} {v[4]:3d}")
    mean = sum(v[0] for v in vals)/len(vals)
    print(f"mean current_distance = {mean:.2f} cm = {mean*10:.1f} mm")

print(f"\nPORT={dev} BAUD={baud}")
