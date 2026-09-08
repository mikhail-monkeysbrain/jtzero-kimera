#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def median(xs):return statistics.median(xs) if xs else float("nan")
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))
def nearest(rows,ts,key):
    vals=[I(r,key) for r in rows]; j=bisect.bisect_left(vals,ts); cand=[]
    if j<len(rows):cand.append(rows[j])
    if j>0:cand.append(rows[j-1])
    return min(cand,key=lambda r:abs(I(r,key)-ts)) if cand else None

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_fc_attitude_raw_world_z.py RUN [RUN ...]")

print("================ V25 FC ATTITUDE + RAW IMU WORLD-Z ================")
print("Uses FC attitude and raw HIGHRES_IMU only for the world-frame acceleration.")
print("Kimera backend attitude and backend bias are not used.")
print("A stationary pre-motion world-Z mean is subtracted to remove static offset.")
for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    events=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    imu.sort(key=lambda r:I(r,"recv_ns")); att.sort(key=lambda r:I(r,"recv_ns"))
    ev={(I(r,"leg"),r["event"]):r for r in events}

    print("\nRUN:",root)
    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee:continue
        t0=I(es,"event_wall_ns"); t1=I(ee,"event_wall_ns")
        seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
        if len(seg)<3:continue

        # pre-motion baseline: first 0.75 s after START; START is settled and motion begins later
        base_end=t0+int(0.75e9)

        vals=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"),"recv_ns")
            if not a:continue
            # FC raw FRD -> FLU
            af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
            # Match established convention used by existing raw-profile analysis.
            rr=math.radians(F(a,"roll_deg"))
            pp=math.radians(-F(a,"pitch_deg"))
            yy=math.radians(-F(a,"yaw_deg"))
            aw=mv(RzRyRx(rr,pp,yy),af)
            vals.append((I(q,"recv_ns"),aw[2]-G))

        base=[z for t,z in vals if t<=base_end]
        b=median(base) if base else 0.0
        dyn=[(t,z-b) for t,z in vals]
        early=[z for t,z in dyn if t<=t0+int(1.5e9)]
        whole=[z for _,z in dyn]

        # integrate residual acceleration with zero initial vz/pz over operator interval
        v=0.0; p=0.0; prev=None
        for t,z in dyn:
            if prev is not None:
                dt=(t-prev[0])*1e-9
                if 0<dt<=0.03:
                    zmid=0.5*(z+prev[1])
                    p += v*dt + 0.5*zmid*dt*dt
                    v += zmid*dt
            prev=(t,z)

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  FC+raw static world-Z residual median = {b:+.5f} m/s^2")
        print(f"  FC+raw early residual mean            = {mean(early):+.5f} m/s^2")
        print(f"  FC+raw whole residual mean            = {mean(whole):+.5f} m/s^2")
        print(f"  FC+raw integrated Vz/Pz               = {v*1000:+.1f} mm/s / {p*1000:+.1f} mm")

print("\nDECISION:")
print("- FC+raw integrated Z with the same direction-dependent sign as Kimera => the effect is already present in FC attitude + physical IMU signal.")
print("- FC+raw near zero while Kimera PIM shows large signed Z => Kimera-side orientation/extrinsic/preintegration handling becomes primary.")
print("- External video remains the independent mechanical reference for whether FC attitude change is physically real.")
