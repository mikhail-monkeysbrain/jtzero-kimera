#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def median(xs):return statistics.median(xs) if xs else float("nan")
def sd(xs):return statistics.pstdev(xs) if len(xs)>1 else 0.0
def wrap(x):return (x+180.0)%360.0-180.0
def nearest(rows,ts,key):
    vals=[I(r,key) for r in rows]; j=bisect.bisect_left(vals,ts); c=[]
    if j<len(rows):c.append(rows[j])
    if j>0:c.append(rows[j-1])
    return min(c,key=lambda r:abs(I(r,key)-ts)) if c else None
def acc_roll_flu(ax,ay,az):
    # CSV is FC FRD; convert to FLU first.
    y=-ay; z=-az
    return math.degrees(math.atan2(y,z))
def corr(a,b):
    if len(a)<3 or len(a)!=len(b):return float("nan")
    ma,mb=mean(a),mean(b)
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb) if va>0 and vb>0 else float("nan")

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_roll_mismatch_timeseries.py RUN [RUN ...]")

print("================ V25 ROLL-MISMATCH TIME SERIES ================")
print("Compares accel-derived roll with FC roll through each operator START->END interval.")
print("Searches lag that maximizes correlation and checks whether signed mismatch tracks reconstructed world-Z residual.")

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
        if len(seg)<10:continue

        # Build accel-roll samples and matched FC roll.
        ts=[]; ar=[]; fr=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"),"recv_ns")
            if not a:continue
            ts.append(I(q,"recv_ns"))
            ar.append(acc_roll_flu(F(q,"ax"),F(q,"ay"),F(q,"az")))
            fr.append(F(a,"roll_deg"))

        # Compare relative roll from settled start to remove fixed convention/offset.
        ar0=median(ar[:max(3,len(ar)//10)])
        fr0=median(fr[:max(3,len(fr)//10)])
        ard=[wrap(x-ar0) for x in ar]
        frd=[wrap(x-fr0) for x in fr]
        mm=[wrap(a-b) for a,b in zip(ard,frd)]

        # Search lag +/-250 ms in 10 ms steps. Shift FC relative to accel.
        best=(float("-inf"),0)
        dt_med=median([(b-a)/1e9 for a,b in zip(ts,ts[1:]) if b>a])
        if not math.isfinite(dt_med) or dt_med<=0: dt_med=0.005
        for lag_ms in range(-250,251,10):
            sh=round((lag_ms/1000.0)/dt_med)
            if sh>=0:
                aa=ard[sh:]; ff=frd[:len(frd)-sh] if sh else frd[:]
            else:
                s=-sh; aa=ard[:len(ard)-s]; ff=frd[s:]
            if len(aa)<20:continue
            c=corr(aa,ff)
            if math.isfinite(c) and c>best[0]:best=(c,lag_ms)

        # world-Z residual using same FC attitude convention as prior analyzer
        # For correlation attribution, use dominant Y-term residual only:
        # Zy ~= cos(pitch)*sin(roll)*ay_FLU; subtract start baseline.
        zy=[]
        for q in seg:
            a=nearest(att,I(q,"recv_ns"),"recv_ns")
            if not a:continue
            rr=math.radians(F(a,"roll_deg"))
            pp=math.radians(-F(a,"pitch_deg"))
            ay=-F(q,"ay")
            zy.append(math.cos(pp)*math.sin(rr)*ay)
        bz=median(zy[:max(3,len(zy)//10)])
        zyr=[z-bz for z in zy]
        n=min(len(mm),len(zyr))
        c_mz=corr(mm[:n],zyr[:n])

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  relative roll mismatch mean/median/sd = {mean(mm):+.4f}/{median(mm):+.4f}/{sd(mm):.4f} deg")
        print(f"  best accel-roll vs FC-roll correlation = {best[0]:+.3f} at lag={best[1]:+d} ms")
        print(f"  corr(roll mismatch, body-Y->world-Z residual) = {c_mz:+.3f}")
        print(f"  mismatch p10/p90 = {sorted(mm)[max(0,int(.10*len(mm))-1)]:+.4f}/"
              f"{sorted(mm)[min(len(mm)-1,int(.90*len(mm)) )]:+.4f} deg")

print("\nINTERPRETATION:")
print("- Consistent non-zero best lag across runs supports timing misalignment between accel-derived gravity direction and FC attitude.")
print("- Large corr(mismatch, Y->Z residual) supports roll mismatch as the mechanism of uncancelled world-Z.")
print("- No repeatable lag/correlation means residual Z is not explained by simple roll timing mismatch; inspect accel filtering/calibration dynamics next.")
