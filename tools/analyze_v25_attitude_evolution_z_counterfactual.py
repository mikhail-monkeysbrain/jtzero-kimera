#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(float(r[k]))
def med(x): return statistics.median(x) if x else float("nan")
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v): return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))
def nearest(rows,ts):
    vals=[I(r,"recv_ns") for r in rows]; j=bisect.bisect_left(vals,ts); c=[]
    if j<len(rows): c.append(rows[j])
    if j: c.append(rows[j-1])
    return min(c,key=lambda r:abs(I(r,"recv_ns")-ts)) if c else None
def integ(vals):
    v=p=0.; prev=None
    for t,a in vals:
        if prev:
            dt=(t-prev[0])*1e-9
            if 0<dt<=0.03:
                am=.5*(a+prev[1]); p += v*dt+.5*am*dt*dt; v += am*dt
        prev=(t,a)
    return v,p

if len(sys.argv)<2: raise SystemExit("usage: analyze_v25_attitude_evolution_z_counterfactual.py RUN [RUN ...]")
print("================ V25 ATTITUDE-EVOLUTION Z COUNTERFACTUAL ================")
print("ACTUAL: raw FC accel rotated by time-varying FC attitude.")
print("FROZEN: same accel samples, but attitude frozen at settled START.")
print("This is attribution only. FROZEN is NOT a proposed correction if the measured rotation is physical.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=sorted([r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"],key=lambda r:I(r,"recv_ns"))
    att=sorted(load(root/"jtzero_500mm_v25_attitude.csv"),key=lambda r:I(r,"recv_ns"))
    ev={(I(r,"leg"),r["event"]):r for r in load(root/"jtzero_500mm_v25_events.csv")}
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    print("\nRUN:",root)
    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee: continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
        if len(seg)<3: continue
        astart=nearest(att,t0)
        if not astart: continue
        def rot(a,q):
            af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
            rr=math.radians(F(a,"roll_deg")); pp=math.radians(-F(a,"pitch_deg")); yy=math.radians(-F(a,"yaw_deg"))
            return mv(RzRyRx(rr,pp,yy),af)[2]-G
        actual=[]; frozen=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"))
            if not a: continue
            actual.append((I(q,"recv_ns"),rot(a,q)))
            frozen.append((I(q,"recv_ns"),rot(astart,q)))
        base_end=t0+int(.75e9)
        ba=med([z for t,z in actual if t<=base_end]); bf=med([z for t,z in frozen if t<=base_end])
        actual=[(t,z-ba) for t,z in actual]; frozen=[(t,z-bf) for t,z in frozen]
        va,pa=integ(actual); vf,pf=integ(frozen)
        dz=F(L,"dz_m")
        print(f"LEG {leg} {L['direction']}: backend dz={dz*1000:+.1f}mm")
        print(f"  ACTUAL FC attitude: Vz={va*1000:+.1f}mm/s Pz={pa*1000:+.1f}mm")
        print(f"  FROZEN start att : Vz={vf*1000:+.1f}mm/s Pz={pf*1000:+.1f}mm")
        print(f"  attitude-evolution contribution ACTUAL-FROZEN: dPz={(pa-pf)*1000:+.1f}mm")
print("\nINTERPRETATION:")
print("- Large ACTUAL-FROZEN difference: time-varying FC orientation materially changes world-Z inferred from the same accelerometer samples.")
print("- Similar ACTUAL and FROZEN: Z error is mainly in accelerometer dynamics/static calibration rather than attitude evolution.")
print("- Do not treat FROZEN as physically correct when dual-IMU gyro/gravity evidence supports real rotation.")
