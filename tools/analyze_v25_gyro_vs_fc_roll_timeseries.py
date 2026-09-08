#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def median(xs):return statistics.median(xs) if xs else float("nan")
def wrap(x):return (x+180.0)%360.0-180.0
def corr(a,b):
    if len(a)<3 or len(a)!=len(b):return float("nan")
    ma,mb=mean(a),mean(b)
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    if va<=0 or vb<=0:return float("nan")
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)
def nearest(rows,ts,key):
    vals=[I(r,key) for r in rows]; j=bisect.bisect_left(vals,ts); c=[]
    if j<len(rows):c.append(rows[j])
    if j>0:c.append(rows[j-1])
    return min(c,key=lambda r:abs(I(r,key)-ts)) if c else None

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_gyro_vs_fc_roll_timeseries.py RUN [RUN ...]")

print("================ V25 GYRO vs FC ROLL TIME SERIES ================")
print("Uses gyro X (roll rate) instead of accel-derived roll during motion.")
print("This avoids contamination by translational acceleration.")
print("Searches lag between integrated gyro roll and FC roll.")

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
        if len(seg)<20:continue

        # Gyro-X bias from first 0.75 s settled window.
        bseg=[r for r in seg if I(r,"recv_ns")<=t0+int(.75e9)]
        bgx=median([F(r,"gx") for r in bseg]) if bseg else 0.0

        ts=[]; grot=[]; froll=[]
        integ=0.0; prev=None
        fc0=None
        for q in seg:
            t=I(q,"recv_ns")
            if prev is not None:
                dt=(t-prev[0])*1e-9
                if 0<dt<=0.03:
                    gxmid=0.5*((F(q,"gx")-bgx)+(prev[1]-bgx))
                    integ += math.degrees(gxmid*dt)
            prev=(t,F(q,"gx"))
            a=nearest(att,t,"recv_ns")
            if not a:continue
            if fc0 is None: fc0=F(a,"roll_deg")
            ts.append(t); grot.append(integ); froll.append(wrap(F(a,"roll_deg")-fc0))

        if len(ts)<20:continue
        # Estimate median IMU sample dt for index shifts.
        dts=[(b-a)*1e-9 for a,b in zip(ts,ts[1:]) if b>a]
        dt_med=median(dts) if dts else .005

        best=(-2.0,0)
        for lag_ms in range(-300,301,10):
            sh=round((lag_ms/1000.0)/dt_med)
            if sh>=0:
                gg=grot[sh:]; ff=froll[:len(froll)-sh] if sh else froll[:]
            else:
                s=-sh; gg=grot[:len(grot)-s]; ff=froll[s:]
            if len(gg)<20:continue
            c=corr(gg,ff)
            if math.isfinite(c) and c>best[0]:best=(c,lag_ms)

        # endpoint comparison
        gyro_end=grot[-1]; fc_end=froll[-1]
        err=wrap(gyro_end-fc_end)

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  gyro-X bias={bgx:+.7f} rad/s")
        print(f"  endpoint: integrated gyro roll={gyro_end:+.4f} deg  FC dRoll={fc_end:+.4f} deg  diff={err:+.4f} deg")
        print(f"  best correlation={best[0]:+.3f} at lag={best[1]:+d} ms")

print("\nINTERPRETATION:")
print("- Similar lag across all runs with high correlation supports timing delay in FC attitude.")
print("- Near-zero lag and close endpoint agreement means FC roll timing is not the main Z mechanism.")
print("- Large endpoint disagreement with no stable lag suggests gyro bias/filtering or attitude-estimator dynamics rather than simple timestamp offset.")
