#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def med(xs):return statistics.median(xs) if xs else float("nan")
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def nearest(rows,ts,key):
    vals=[I(r,key) for r in rows]; j=bisect.bisect_left(vals,ts); c=[]
    if j<len(rows):c.append(rows[j])
    if j>0:c.append(rows[j-1])
    return min(c,key=lambda r:abs(I(r,key)-ts)) if c else None
def integ(vals):
    v=p=0.0; prev=None
    for t,a in vals:
        if prev is not None:
            dt=(t-prev[0])*1e-9
            if 0<dt<=0.03:
                am=0.5*(a+prev[1]); p+=v*dt+0.5*am*dt*dt; v+=am*dt
        prev=(t,a)
    return v,p

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_world_z_axis_decomposition.py RUN [RUN ...]")

print("================ V25 WORLD-Z AXIS DECOMPOSITION ================")
print("Decomposes reconstructed world-Z acceleration into body-axis terms:")
print("  Zx = R31*ax, Zy = R32*ay, Zz = R33*az - g")
print("Then subtracts each term's own settled-start baseline before integration.")
print("This is attribution only; no estimator state is modified.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=sorted([r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"], key=lambda r:I(r,"recv_ns"))
    att=sorted(load(root/"jtzero_500mm_v25_attitude.csv"), key=lambda r:I(r,"recv_ns"))
    events=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    ev={(I(r,"leg"),r["event"]):r for r in events}
    print("\nRUN:",root)

    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee:continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
        if len(seg)<3:continue

        rows=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"),"recv_ns")
            if not a:continue
            # raw FC FRD -> FLU convention used by prior analyzers
            ax,ay,az=F(q,"ax"),-F(q,"ay"),-F(q,"az")
            rr=math.radians(F(a,"roll_deg"))
            pp=math.radians(-F(a,"pitch_deg"))
            yy=math.radians(-F(a,"yaw_deg"))
            R=RzRyRx(rr,pp,yy)
            zx=R[2][0]*ax
            zy=R[2][1]*ay
            zz=R[2][2]*az-G
            rows.append((I(q,"recv_ns"),zx,zy,zz))

        base_end=t0+int(0.75e9)
        bx=med([x for t,x,y,z in rows if t<=base_end]); by=med([y for t,x,y,z in rows if t<=base_end]); bz=med([z for t,x,y,z in rows if t<=base_end])
        sx=[(t,x-bx) for t,x,y,z in rows]
        sy=[(t,y-by) for t,x,y,z in rows]
        sz=[(t,z-bz) for t,x,y,z in rows]
        st=[(t,(x-bx)+(y-by)+(z-bz)) for t,x,y,z in rows]

        vx,px=integ(sx); vy,py=integ(sy); vz,pz=integ(sz); vt,pt=integ(st)
        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  body-X -> world-Z: mean={mean([a for _,a in sx]):+.5f} m/s^2  Pz={px*1000:+.1f}mm")
        print(f"  body-Y -> world-Z: mean={mean([a for _,a in sy]):+.5f} m/s^2  Pz={py*1000:+.1f}mm")
        print(f"  body-Z -> world-Z: mean={mean([a for _,a in sz]):+.5f} m/s^2  Pz={pz*1000:+.1f}mm")
        print(f"  SUM                : mean={mean([a for _,a in st]):+.5f} m/s^2  Pz={pt*1000:+.1f}mm")
        print(f"  contribution fractions by |Pz|: X={abs(px)/(abs(px)+abs(py)+abs(pz)+1e-12):.2f} "
              f"Y={abs(py)/(abs(px)+abs(py)+abs(pz)+1e-12):.2f} "
              f"Z={abs(pz)/(abs(px)+abs(py)+abs(pz)+1e-12):.2f}")

print("\nINTERPRETATION:")
print("- Dominant body-Y term with several-degree roll means horizontal/specific-force Y is leaking into world Z through static attitude geometry.")
print("- Dominant body-X term points to pitch-mediated projection.")
print("- Dominant body-Z term points to vertical-axis accelerometer bias/dynamics rather than horizontal projection.")
print("- Similar signed axis contribution across directions identifies the physical channel to investigate next.")
