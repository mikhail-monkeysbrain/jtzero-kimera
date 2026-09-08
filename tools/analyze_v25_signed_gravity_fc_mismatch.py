#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def median(xs):return statistics.median(xs) if xs else float("nan")
def wrap(x):return (x+180.0)%360.0-180.0
def central(rows,key):
    rows=sorted(rows,key=lambda r:I(r,key))
    k=len(rows)//4
    return rows[k:len(rows)-k] if len(rows)-2*k>=3 else rows
def nearest(rows,ts,key):
    return min(rows,key=lambda r:abs(I(r,key)-ts)) if rows else None
def acc_rp(ax,ay,az):
    # FLU gravity-direction tilt convention for signed comparison.
    r=math.degrees(math.atan2(ay,az))
    p=math.degrees(math.atan2(-ax,math.hypot(ay,az)))
    return r,p
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_signed_gravity_fc_mismatch.py RUN [RUN ...]")

print("================ V25 SIGNED GRAVITY vs FC MISMATCH ================")
print("Compares signed accel-derived roll/pitch with FC roll/pitch at settled endpoints.")
print("Also reports equivalent small tilt corresponding to mean reconstructed world-Z residual.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    back=load(root/"jtzero_500mm_v25_backend.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    events=load(root/"jtzero_500mm_v25_events.csv")
    bykf={I(r,"keyframe"):r for r in back}
    ev={(I(r,"leg"),r["event"]):r for r in events}

    print("\nRUN:",root)
    for L in legs:
        leg=I(L,"leg")
        ks,ke=I(L,"start_settled_kf"),I(L,"end_settled_kf")
        s,e=bykf.get(ks),bykf.get(ke)
        if not s or not e:continue

        w=int(0.6e9)
        i0=central([r for r in imu if abs(I(r,"mapped_ns")-I(s,"timestamp_ns"))<=w],"mapped_ns")
        i1=central([r for r in imu if abs(I(r,"mapped_ns")-I(e,"timestamp_ns"))<=w],"mapped_ns")
        if not i0 or not i1:continue

        # FC CSV ax/ay/az stored FRD; convert to FLU as in prior analyzers.
        a0=(mean([F(r,"ax") for r in i0]), -mean([F(r,"ay") for r in i0]), -mean([F(r,"az") for r in i0]))
        a1=(mean([F(r,"ax") for r in i1]), -mean([F(r,"ay") for r in i1]), -mean([F(r,"az") for r in i1]))
        ar0,ap0=acc_rp(*a0); ar1,ap1=acc_rp(*a1)
        adr=wrap(ar1-ar0); adp=wrap(ap1-ap0)

        fc0=nearest(att,I(s,"callback_wall_ns"),"recv_ns")
        fc1=nearest(att,I(e,"callback_wall_ns"),"recv_ns")
        if not fc0 or not fc1:continue
        fr0,fp0=F(fc0,"roll_deg"),F(fc0,"pitch_deg")
        fr1,fp1=F(fc1,"roll_deg"),F(fc1,"pitch_deg")
        fdr=wrap(fr1-fr0); fdp=wrap(fp1-fp0)

        # Reconstruct mean world-Z residual over operator START->END.
        es,ee=ev.get((leg,"START")),ev.get((leg,"END"))
        mean_z=float("nan"); equiv=float("nan")
        if es and ee:
            t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
            seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
            vals=[]
            for q in seg:
                a=nearest(att,I(q,"recv_ns"),"recv_ns")
                if not a:continue
                af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
                rr=math.radians(F(a,"roll_deg"))
                pp=math.radians(-F(a,"pitch_deg"))
                yy=math.radians(-F(a,"yaw_deg"))
                aw=mv(RzRyRx(rr,pp,yy),af)
                vals.append((I(q,"recv_ns"),aw[2]-G))
            if vals:
                base_end=t0+int(0.75e9)
                b=median([z for t,z in vals if t<=base_end])
                resid=[z-b for _,z in vals]
                mean_z=mean(resid)
                equiv=math.degrees(math.asin(max(-1.0,min(1.0,mean_z/G))))

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  ACC start R/P=[{ar0:+.4f},{ap0:+.4f}] end=[{ar1:+.4f},{ap1:+.4f}] d=[{adr:+.4f},{adp:+.4f}] deg")
        print(f"  FC  start R/P=[{fr0:+.4f},{fp0:+.4f}] end=[{fr1:+.4f},{fp1:+.4f}] d=[{fdr:+.4f},{fdp:+.4f}] deg")
        print(f"  delta mismatch ACC-FC = dR={wrap(adr-fdr):+.4f} dP={wrap(adp-fdp):+.4f} deg")
        print(f"  mean world-Z residual={mean_z:+.5f} m/s^2 => equivalent tilt={equiv:+.4f} deg")

print("\nINTERPRETATION:")
print("- If ACC-FC signed roll mismatch is of the same order as equivalent tilt (~0.05..0.2 deg), sub-degree attitude/gravity mismatch can explain the residual Z.")
print("- Large mismatch only at endpoints suggests calibration/reference mismatch; mismatch during motion would require time-series/lag analysis next.")
print("- This does not assume the full ~2.5 deg physical roll is erroneous; it tests only the small uncancelled remainder.")
