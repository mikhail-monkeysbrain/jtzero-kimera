#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def median(xs):return statistics.median(xs) if xs else float("nan")
def norm(v):return math.sqrt(sum(x*x for x in v))
def angle(a,b):
    na,nb=norm(a),norm(b)
    if na<=0 or nb<=0:return float("nan")
    c=sum(x*y for x,y in zip(a,b))/(na*nb)
    return math.degrees(math.acos(max(-1.0,min(1.0,c))))
def wrap(x):return (x+180.0)%360.0-180.0
def nearest(rows,ts,key):
    return min(rows,key=lambda r:abs(I(r,key)-ts)) if rows else None
def central(rows):
    rows=sorted(rows,key=lambda r:I(r,"mapped_ns"))
    k=len(rows)//4
    return rows[k:len(rows)-k] if len(rows)-2*k>=3 else rows

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_raw_gravity_vs_fc_attitude.py RUN [RUN ...]")

print("================ V25 RAW GRAVITY vs FC ATTITUDE ================")
print("Compares stationary raw accelerometer gravity-vector direction before and after each leg")
print("with FC ATTITUDE roll/pitch change. No Kimera attitude is used for the gravity-vector angle.")

allr=[]
for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    back=load(root/"jtzero_500mm_v25_backend.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    att.sort(key=lambda r:I(r,"recv_ns"))

    print("\nRUN:",root)
    bykf={I(r,"keyframe"):r for r in back}
    for L in legs:
        ks,ke=I(L,"start_settled_kf"),I(L,"end_settled_kf")
        s,e=bykf.get(ks),bykf.get(ke)
        if not s or not e:continue

        # Use +/-0.6s neighborhoods around settled start/end source timestamps.
        w=int(0.6e9)
        i0=central([r for r in imu if abs(I(r,"mapped_ns")-I(s,"timestamp_ns"))<=w])
        i1=central([r for r in imu if abs(I(r,"mapped_ns")-I(e,"timestamp_ns"))<=w])
        if not i0 or not i1:
            print(" insufficient IMU samples"); continue

        a0=tuple(mean([F(r,k) for r in i0]) for k in ("ax","ay","az"))
        a1=tuple(mean([F(r,k) for r in i1]) for k in ("ax","ay","az"))
        grav_ang=angle(a0,a1)

        fc0=nearest(att,I(s,"callback_wall_ns"),"recv_ns")
        fc1=nearest(att,I(e,"callback_wall_ns"),"recv_ns")
        if not fc0 or not fc1:continue
        dr=wrap(F(fc1,"roll_deg")-F(fc0,"roll_deg"))
        dp=wrap(F(fc1,"pitch_deg")-F(fc0,"pitch_deg"))
        dy=wrap(F(fc1,"yaw_deg")-F(fc0,"yaw_deg"))
        tilt=math.hypot(dr,dp)
        ratio=grav_ang/tilt if tilt>1e-9 else float("nan")

        print(f"LEG {I(L,'leg')} {L['direction']}:")
        print(f"  raw gravity angle change = {grav_ang:.4f} deg")
        print(f"  FC dRPY=[{dr:+.4f},{dp:+.4f},{dy:+.4f}] |dTilt|={tilt:.4f} deg")
        print(f"  RAW/FC tilt ratio = {ratio:.3f}")
        print(f"  |a0|={norm(a0):.5f} |a1|={norm(a1):.5f} m/s^2")
        allr.append((L["direction"],grav_ang,tilt,ratio))

print("\n================ DIRECTION SUMMARY ================")
for d in ("A->B","B->A"):
    rr=[x for x in allr if x[0]==d]
    if rr:
        print(f"{d}: n={len(rr)} raw gravity median={median([x[1] for x in rr]):.4f}deg "
              f"FC tilt median={median([x[2] for x in rr]):.4f}deg "
              f"RAW/FC median={median([x[3] for x in rr]):.3f}")

print("\nDECISION:")
print("- RAW gravity-vector angle ~= FC tilt: the FC tilt change is supported by the accelerometer gravity direction at stationary endpoints.")
print("- RAW gravity-vector angle << FC tilt: FC attitude reports substantially more tilt than raw stationary gravity supports.")
print("- This still cannot distinguish whole-rig tilt from local FC/IMU mechanical motion; external video is the whole-rig reference.")
