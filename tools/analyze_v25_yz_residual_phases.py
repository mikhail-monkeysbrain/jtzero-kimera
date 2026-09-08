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
def nearest(rows,ts,key):
    vals=[I(r,key) for r in rows]; j=bisect.bisect_left(vals,ts); c=[]
    if j<len(rows):c.append(rows[j])
    if j>0:c.append(rows[j-1])
    return min(c,key=lambda r:abs(I(r,key)-ts)) if c else None
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def integ(vals):
    v=p=0.0; prev=None
    for t,a in vals:
        if prev:
            dt=(t-prev[0])*1e-9
            if 0<dt<=0.03:
                am=.5*(a+prev[1]); p += v*dt+.5*am*dt*dt; v += am*dt
        prev=(t,a)
    return v,p

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_yz_residual_phases.py RUN [RUN ...]")

print("================ V25 Y/Z RESIDUAL BY MOTION PHASE ================")
print("Splits each START->END interval into time quartiles.")
print("Reports body-Y->world-Z, body-Z->world-Z and their residual after a settled-start baseline.")
print("Quartiles are temporal diagnostics, not asserted physical accel/cruise/brake labels.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=sorted([r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"],key=lambda r:I(r,"recv_ns"))
    att=sorted(load(root/"jtzero_500mm_v25_attitude.csv"),key=lambda r:I(r,"recv_ns"))
    evrows=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    ev={(I(r,"leg"),r["event"]):r for r in evrows}
    print("\nRUN:",root)

    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee:continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        seg=[q for q in imu if t0<=I(q,"recv_ns")<=t1]
        vals=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"),"recv_ns")
            if not a:continue
            ax,ay,az=F(q,"ax"),-F(q,"ay"),-F(q,"az")
            rr=math.radians(F(a,"roll_deg")); pp=math.radians(-F(a,"pitch_deg")); yy=math.radians(-F(a,"yaw_deg"))
            R=RzRyRx(rr,pp,yy)
            zy=R[2][1]*ay
            zz=R[2][2]*az-G
            vals.append((I(q,"recv_ns"),zy,zz))

        if len(vals)<20:continue
        base_end=t0+int(.75e9)
        by=median([y for t,y,z in vals if t<=base_end])
        bz=median([z for t,y,z in vals if t<=base_end])
        vals=[(t,y-by,z-bz,(y-by)+(z-bz)) for t,y,z in vals]

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm dur={(t1-t0)*1e-9:.2f}s")
        total=[]
        for qi in range(4):
            qa=t0+(t1-t0)*qi//4; qb=t0+(t1-t0)*(qi+1)//4
            q=[x for x in vals if qa<=x[0] <= qb]
            if not q:continue
            my=mean([x[1] for x in q]); mz=mean([x[2] for x in q]); mr=mean([x[3] for x in q])
            vr,pr=integ([(x[0],x[3]) for x in q])
            print(f"  Q{qi+1} {qi*25:02d}-{(qi+1)*25:03d}%: Y={my:+.5f} Z={mz:+.5f} residual={mr:+.5f} m/s^2 local_dPz={pr*1000:+.1f}mm")
            total.extend(q)
        vt,pt=integ([(x[0],x[3]) for x in vals])
        print(f"  WHOLE: residual mean={mean([x[3] for x in vals]):+.5f} m/s^2 integrated Pz={pt*1000:+.1f}mm")

print("\nINTERPRETATION:")
print("- Residual concentrated in early/late quartiles points toward acceleration/braking transients, but quartiles alone do not prove those physical phases.")
print("- Residual persisting through middle quartiles points toward sustained calibration/geometry/filtering or sustained non-horizontal specific force.")
print("- Opposite signed residuals between A->B and B->A identify the phase where direction dependence is generated.")
print("- Y and Z that are individually large but cancel closely indicate the remaining error is a small cancellation defect, not a large vertical acceleration.")
