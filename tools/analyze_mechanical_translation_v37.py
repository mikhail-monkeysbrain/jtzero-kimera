#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def norm(v):return math.sqrt(sum(x*x for x in v))
def angle(a,b):
    na,nb=norm(a),norm(b)
    c=sum(x*y for x,y in zip(a,b))/(na*nb)
    return math.degrees(math.acos(max(-1.0,min(1.0,c))))
def acc_tilt(v):
    x,y,z=v
    return (math.degrees(math.atan2(y,z)),
            math.degrees(math.atan2(-x,math.hypot(y,z))))
def central(rr):
    k=len(rr)//4
    return rr[k:len(rr)-k] if len(rr)-2*k>=3 else rr
def wrap(x):return (x+180.0)%360.0-180.0
def circ_mean(xs):
    s=mean([math.sin(math.radians(x)) for x in xs]); c=mean([math.cos(math.radians(x)) for x in xs])
    return math.degrees(math.atan2(s,c))
def trapz_vec(rr, bias):
    rr=sorted(rr,key=lambda r:F(r,"imu_us"))
    out=[0.0,0.0,0.0]
    for a,b in zip(rr,rr[1:]):
        dt=(F(b,"imu_us")-F(a,"imu_us"))*1e-6
        if dt<=0 or dt>0.03: continue
        for j,k in enumerate(("gx_flu","gy_flu","gz_flu")):
            out[j]+=0.5*((F(a,k)-bias[j])+(F(b,k)-bias[j]))*dt
    return tuple(math.degrees(x) for x in out)

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_mechanical_translation_v37.py jtzero_mechanical_translation_v37.csv")
rows=load(Path(sys.argv[1]))
phases={i:[r for r in rows if int(r["phase"])==i] for i in range(5)}

print("================ V37 МЕХАНИЧЕСКИЙ ЭТАЛОН ================")
summ={}
for p in (0,2,4):
    rr=central(phases[p])
    acc=tuple(mean([F(r,k) for r in rr]) for k in ("ax_flu","ay_flu","az_flu"))
    gyro=tuple(mean([F(r,k) for r in rr]) for k in ("gx_flu","gy_flu","gz_flu"))
    fc=(mean([F(r,"fc_roll_deg") for r in rr]),
        mean([F(r,"fc_pitch_deg") for r in rr]),
        circ_mean([F(r,"fc_yaw_deg") for r in rr]))
    at=acc_tilt(acc)
    summ[p]=(acc,gyro,fc,at)
    print(f"{phases[p][0]['phase_name']}: n={len(rr)}")
    print(f"  ACC=[{acc[0]:+.5f},{acc[1]:+.5f},{acc[2]:+.5f}] |a|={norm(acc):.5f}")
    print(f"  ACC tilt R/P=[{at[0]:+.4f},{at[1]:+.4f}] deg")
    print(f"  FC  R/P=[{fc[0]:+.4f},{fc[1]:+.4f}] deg")

for a,b,name in ((0,2,"A->B"),(2,4,"B->A"),(0,4,"A closure")):
    aa,_,fa,ta=summ[a]; ab,_,fb,tb=summ[b]
    raw=angle(aa,ab)
    dr=fb[0]-fa[0]; dp=fb[1]-fa[1]
    fc_tilt=math.hypot(dr,dp)
    dar=tb[0]-ta[0]; dap=tb[1]-ta[1]
    print(f"\n{name}:")
    print(f"  raw gravity angle={raw:.4f} deg")
    print(f"  ACC tilt delta R/P=[{dar:+.4f},{dap:+.4f}] deg")
    print(f"  FC  tilt delta R/P=[{dr:+.4f},{dp:+.4f}] |dTilt|={fc_tilt:.4f} deg")

bias_ab=summ[0][1]
bias_ba=summ[2][1]
for p,bias,name in ((1,bias_ab,"A->B"),(3,bias_ba,"B->A")):
    rv=trapz_vec(phases[p],bias)
    print(f"\n{name} raw gyro integral R/P/Y=[{rv[0]:+.4f},{rv[1]:+.4f},{rv[2]:+.4f}] deg |rotvec|={norm(rv):.4f} deg")

print("\nDECISION:")
print("- raw gravity ~= FC tilt ~= raw gyro magnitude => orientation change of the FC/IMU frame is physically supported by independent inertial observables.")
print("- small A closure => the orientation change is largely reversible with the A/B translation.")
print("- external video is still required to distinguish whole visible assembly tilt from local FC/IMU mount motion.")
