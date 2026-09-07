#!/usr/bin/env python3
import csv, math
from pathlib import Path

HOME = Path("/home/vio")
IMU = HOME / "jtzero_500mm_v25.csv"
ATT = HOME / "jtzero_500mm_v25_attitude.csv"
EVENTS = HOME / "jtzero_500mm_v25_events.csv"
BACKEND = HOME / "jtzero_500mm_v25_backend.csv"
WINDOW_NS = 5_000_000_000

def load(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

def F(r, k): return float(r[k])
def I(r, k): return int(float(r[k]))
def mean(v): return sum(v) / len(v)
def std(v):
    m = mean(v)
    return math.sqrt(sum((x-m)*(x-m) for x in v) / max(1, len(v)-1))

events = load(EVENTS)
start = next((r for r in events if r.get("event") == "START" and r.get("leg") == "1"), None)
if start is None:
    raise SystemExit("LEG1 START event not found")
t_end = I(start, "event_wall_ns")
t_begin = t_end - WINDOW_NS

imu = [r for r in load(IMU)
       if r.get("type") == "IMU" and t_begin <= I(r, "recv_ns") <= t_end]
att = [r for r in load(ATT)
       if t_begin <= I(r, "recv_ns") <= t_end]
be_all = load(BACKEND)
state_ts = I(start, "state_timestamp_ns")
be = [r for r in be_all if I(r, "timestamp_ns") <= state_ts][-8:]

if not imu:
    raise SystemExit("No IMU samples in final 5 s before LEG1 START")
if not att:
    raise SystemExit("No ATTITUDE samples in final 5 s before LEG1 START")
if not be:
    raise SystemExit("No backend states before LEG1 START")

# Raw CSV is FRD; V25 feeds FLU = [x,-y,-z].
ax = mean([F(r, "ax") for r in imu])
ay = -mean([F(r, "ay") for r in imu])
az = -mean([F(r, "az") for r in imu])
an = math.sqrt(ax*ax + ay*ay + az*az)

# Gravity-derived FLU tilt for positive specific-force vector at rest.
acc_roll = math.degrees(math.atan2(ay, az))
acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

fc_roll = mean([F(r, "roll_deg") for r in att])
fc_pitch = mean([F(r, "pitch_deg") for r in att])
fc_yaw = mean([F(r, "yaw_deg") for r in att])
fc_roll_std = std([F(r, "roll_deg") for r in att])
fc_pitch_std = std([F(r, "pitch_deg") for r in att])

vio_roll = mean([F(r, "roll_deg") for r in be])
vio_pitch = mean([F(r, "pitch_deg") for r in be])
vio_yaw = mean([F(r, "yaw_deg") for r in be])

print("================ V25 FC / ACCEL / VIO ALIGNMENT ================")
print(f"window: final {WINDOW_NS/1e9:.1f}s before LEG1 START")
print(f"IMU samples={len(imu)} ATTITUDE samples={len(att)} backend states={len(be)}")
print()
print(f"RAW ACC FLU mean = [{ax:+.6f}, {ay:+.6f}, {az:+.6f}] m/s^2  |a|={an:.6f}")
print(f"ACC gravity tilt = roll {acc_roll:+.3f} deg  pitch {acc_pitch:+.3f} deg")
print()
print(f"FC ATTITUDE mean = roll {fc_roll:+.3f} deg  pitch {fc_pitch:+.3f} deg  yaw {fc_yaw:+.3f} deg")
print(f"FC R/P std       = {fc_roll_std:.4f} / {fc_pitch_std:.4f} deg")
print()
print(f"VIO late mean    = roll {vio_roll:+.3f} deg  pitch {vio_pitch:+.3f} deg  yaw {vio_yaw:+.3f} deg")
print()
print(f"ACC - FC         = dRoll {acc_roll-fc_roll:+.3f} deg  dPitch {acc_pitch-fc_pitch:+.3f} deg")
print(f"VIO - FC         = dRoll {vio_roll-fc_roll:+.3f} deg  dPitch {vio_pitch-fc_pitch:+.3f} deg")
print(f"VIO - ACC        = dRoll {vio_roll-acc_roll:+.3f} deg  dPitch {vio_pitch-acc_pitch:+.3f} deg")
print()
print("INTERPRETATION:")
print("  ACC≈FC but VIO differs -> VIO initialization/attitude-bias observability is the primary suspect.")
print("  ACC differs from FC similarly to VIO -> inspect FC accel calibration/frame semantics.")
print("  All three agree -> large BA must come from another factor/bias-state coupling.")
