#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def rms(xs): return math.sqrt(mean([x*x for x in xs])) if xs else float("nan")
def unwrap_deg(xs):
    if not xs: return []
    out=[xs[0]]
    for x in xs[1:]:
        y=x
        while y-out[-1] > 180: y-=360
        while y-out[-1] < -180: y+=360
        out.append(y)
    return out
def linreg(xs,ys):
    if len(xs)<3: return float("nan"),float("nan")
    mx,my=mean(xs),mean(ys); sxx=sum((x-mx)**2 for x in xs)
    if sxx<=0: return float("nan"),float("nan")
    k=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sxx
    b=my-k*mx
    sst=sum((y-my)**2 for y in ys); ssr=sum((y-(k*x+b))**2 for x,y in zip(xs,ys))
    return k,1-ssr/sst if sst>0 else float("nan")

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_direction_yaw_summary.py RUN_DIR [RUN_DIR ...]")

allrows=[]
for arg in sys.argv[1:]:
    root=Path(arg)
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    events=load(root/"jtzero_500mm_v25_events.csv")
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    ev={(I(r,"leg"),r["event"]):r for r in events}
    print("\n================ RUN ================")
    print(root)
    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee: continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        aa=[r for r in att if t0<=I(r,"recv_ns")<=t1]
        ii=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
        y=unwrap_deg([F(r,"yaw_deg") for r in aa])
        signed_yaw=(y[-1]-y[0]) if len(y)>=2 else float("nan")
        yaw_span=(max(y)-min(y)) if y else float("nan")
        gz=[]
        for q in ii:
            for k in ("gz","zgyro"):
                if k in q and q[k] not in ("",None):
                    gz.append(F(q,k)); break
        row=dict(run=root.name,leg=leg,direction=L["direction"],
                 scale=F(L,"scale_horizontal"),dz_mm=F(L,"dz_m")*1000,
                 yaw_delta=signed_yaw,yaw_span=yaw_span,gz_rms=rms(gz))
        allrows.append(row)
        print(f"LEG {leg} {row['direction']}: scale={row['scale']:.4f} dz={row['dz_mm']:+.2f}mm "
              f"signed_dYaw={signed_yaw:+.3f}deg yawSpan={yaw_span:.3f}deg gzRMS={row['gz_rms']:.5f}rad/s")

print("\n================ ALL-RUN DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in allrows if r["direction"]==d]
    if rr:
        print(f"{d}: n={len(rr)} scale={mean([r['scale'] for r in rr]):.4f} "
              f"dz={mean([r['dz_mm'] for r in rr]):+.2f}mm "
              f"signed_dYaw={mean([r['yaw_delta'] for r in rr]):+.3f}deg "
              f"yawSpan={mean([r['yaw_span'] for r in rr]):.3f}deg "
              f"gzRMS={mean([r['gz_rms'] for r in rr]):.5f}")

print("\n================ SIMPLE ASSOCIATIONS ================")
# With only 12 legs, these are diagnostics, not causal statistics.
for target in ("scale","dz_mm"):
    ys=[r[target] for r in allrows]
    for xname in ("yaw_delta","yaw_span","gz_rms"):
        xs=[r[xname] for r in allrows]
        k,r2=linreg(xs,ys)
        print(f"{target} vs {xname}: slope={k:+.5f} R2={r2:.3f}")
print("\nINTERPRETATION:")
print("- signed_dYaw tells whether yaw actually changed in a consistent direction; yawSpan alone cannot.")
print("- Low R2 with yaw metrics weakens a simple 'yaw alone causes the error' explanation.")
print("- High R2 is only an association: direction and operator motion can still be confounded.")
print("- Compare A->B and B->A across both normal-order and B-first runs before changing estimator parameters.")
