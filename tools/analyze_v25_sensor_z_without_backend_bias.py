#!/usr/bin/env python3
import csv, math, sys, bisect, statistics
from pathlib import Path

ONSET_XY_M=0.005
BASELINE_SEC=0.75

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def median(xs): return statistics.median(xs) if xs else float("nan")

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_sensor_z_without_backend_bias.py RUN_DIR")

root=Path(sys.argv[1])
raw=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
back=sorted(load(root/"jtzero_500mm_v25_backend.csv"), key=lambda r:I(r,"timestamp_ns"))
legs=load(root/"jtzero_500mm_v25_legs.csv")

if not raw:
    raise SystemExit("no IMU rows")

raw_keys=set(raw[0].keys())
mapped_candidates=["mapped_ns","mapped_imu_ns","timestamp_ns","kimera_ns","mapped_timestamp_ns","rpi_mapped_ns"]
mapped_key=next((k for k in mapped_candidates if k in raw_keys),None)
if mapped_key is None:
    mapped_key=next((k for k in raw_keys if k and "mapped" in k.lower() and "ns" in k.lower()),None)
if mapped_key is None:
    raise SystemExit("cannot find mapped IMU timestamp column")

raw=sorted(raw,key=lambda r:I(r,mapped_key))
rts=[I(r,mapped_key) for r in raw]
bykf={I(r,"keyframe"):r for r in back}

def raw_window(t0,t1):
    i=bisect.bisect_left(rts,t0)
    j=bisect.bisect_right(rts,t1)
    return raw[i:j]

print("================ V25 SENSOR Z WITHOUT BACKEND BIAS ================")
print("run:",root)
print("Uses raw FC HIGHRES_IMU only for the signal value.")
print("FLU body-Z = -raw FRD az.")
print("For each leg, subtracts a stationary median from the 0.75 s immediately before detected motion onset.")
print("No backend accelerometer bias is subtracted.")
print()

summary=[]
for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s=bykf.get(ks)
    if not s: continue

    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    onset_kf=ks
    for r in seg:
        dx=F(r,"px_m")-F(s,"px_m"); dy=F(r,"py_m")-F(s,"py_m")
        if math.hypot(dx,dy)>=ONSET_XY_M:
            onset_kf=I(r,"keyframe")
            break

    onset_t=I(bykf[onset_kf],"timestamp_ns")
    end_t=I(bykf[ke],"timestamp_ns")
    baseline=raw_window(onset_t-int(BASELINE_SEC*1e9),onset_t)
    moving=raw_window(onset_t,end_t)
    if len(baseline)<20 or len(moving)<20:
        print(f"LEG {leg}: insufficient raw samples"); continue

    base_z=[-F(r,"az") for r in baseline]
    base_norm=[math.sqrt(F(r,"ax")**2+F(r,"ay")**2+F(r,"az")**2) for r in baseline]
    z0=median(base_z)
    n0=median(base_norm)

    zres=[-F(r,"az")-z0 for r in moving]
    normres=[math.sqrt(F(r,"ax")**2+F(r,"ay")**2+F(r,"az")**2)-n0 for r in moving]

    # First 1.5 s after onset is most relevant to the birth of Vz.
    early=raw_window(onset_t,min(end_t,onset_t+int(1.5e9)))
    ez=[-F(r,"az")-z0 for r in early]
    en=[math.sqrt(F(r,"ax")**2+F(r,"ay")**2+F(r,"az")**2)-n0 for r in early]

    row=(L["direction"],mean(ez),mean(en),mean(zres),mean(normres))
    summary.append(row)

    print(f"LEG {leg} {L['direction']}: endpoint dz={F(L,'dz_m')*1000:+.1f}mm onsetKF={onset_kf}")
    print(f"  stationary raw FLU-Z median = {z0:+.5f} m/s^2")
    print(f"  EARLY 1.5s sensor-only Z residual mean = {mean(ez):+.5f} m/s^2")
    print(f"  EARLY 1.5s accel-norm residual mean     = {mean(en):+.5f} m/s^2")
    print(f"  whole-leg sensor-only Z residual mean   = {mean(zres):+.5f} m/s^2")
    print(f"  whole-leg accel-norm residual mean      = {mean(normres):+.5f} m/s^2")
    print()

print("================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in summary if r[0]==d]
    if rr:
        print(f"{d}: early sensor-Z={mean([r[1] for r in rr]):+.5f} "
              f"early norm={mean([r[2] for r in rr]):+.5f} "
              f"whole sensor-Z={mean([r[3] for r in rr]):+.5f} m/s^2")

print()
print("INTERPRETATION:")
print("- If sensor-only Z residual already flips A->B negative / B->A positive, the directional signal exists in the physical IMU measurement before backend bias correction.")
print("- If sensor-only Z is symmetric but bias-corrected rawZres flips, backend bias handling is creating most of the sign.")
print("- Accel-norm residual helps distinguish a true change in measured specific-force magnitude from a pure axis-redistribution effect.")
