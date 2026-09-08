#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

BINS=[0.0,0.25,0.50,0.75,1.0]

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(float(r[k]))
def mean(xs): return statistics.mean(xs) if xs else float("nan")
def sd(xs): return statistics.pstdev(xs) if len(xs)>1 else 0.0
def wrap(x): return (x+180.0)%360.0-180.0
def nearest(rows, ts, key):
    vals=[I(r,key) for r in rows]
    j=bisect.bisect_left(vals,ts)
    cand=[]
    if j<len(rows): cand.append(rows[j])
    if j>0: cand.append(rows[j-1])
    return min(cand,key=lambda r:abs(I(r,key)-ts)) if cand else None

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_vio_vs_fc_attitude.py RUN [RUN ...]")

out=[]
for arg in sys.argv[1:]:
    root=Path(arg)
    back=load(root/"jtzero_500mm_v25_backend.csv")
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    att.sort(key=lambda r:I(r,"recv_ns"))
    bykf={I(r,"keyframe"):r for r in back}

    for L in legs:
        ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
        s,e=bykf.get(ks),bykf.get(ke)
        if not s or not e: continue
        seg=sorted([r for r in back if I(s,"timestamp_ns")<=I(r,"timestamp_ns")<=I(e,"timestamp_ns")],
                   key=lambda r:I(r,"timestamp_ns"))
        if len(seg)<2: continue

        fc0=nearest(att,I(s,"callback_wall_ns"),"recv_ns")
        if not fc0: continue
        v0=(F(s,"roll_deg"),F(s,"pitch_deg"),F(s,"yaw_deg"))
        f0=(F(fc0,"roll_deg"),F(fc0,"pitch_deg"),F(fc0,"yaw_deg"))

        for frac in BINS:
            target=I(s,"timestamp_ns")+int(frac*(I(e,"timestamp_ns")-I(s,"timestamp_ns")))
            b=min(seg,key=lambda r:abs(I(r,"timestamp_ns")-target))
            fc=nearest(att,I(b,"callback_wall_ns"),"recv_ns")
            if not fc: continue
            vd=(wrap(F(b,"roll_deg")-v0[0]),wrap(F(b,"pitch_deg")-v0[1]),wrap(F(b,"yaw_deg")-v0[2]))
            fd=(wrap(F(fc,"roll_deg")-f0[0]),wrap(F(fc,"pitch_deg")-f0[1]),wrap(F(fc,"yaw_deg")-f0[2]))
            out.append(dict(
                run=root.name,direction=L["direction"],bin=frac,kf=I(b,"keyframe"),
                vio_dr=vd[0],vio_dp=vd[1],vio_dy=vd[2],
                fc_dr=fd[0],fc_dp=fd[1],fc_dy=fd[2],
                vio_tilt=math.hypot(vd[0],vd[1]),fc_tilt=math.hypot(fd[0],fd[1]),
                match_ms=(I(fc,"recv_ns")-I(b,"callback_wall_ns"))/1e6
            ))

print("================ V25 VIO vs FC ATTITUDE ================")
for d in ("A->B","B->A"):
    print("\n"+d)
    rr=[r for r in out if r["direction"]==d]
    for b in BINS:
        x=[r for r in rr if r["bin"]==b]
        if not x: continue
        print(f" {int(b*100):3d}%: VIO dRP=[{mean([r['vio_dr'] for r in x]):+6.3f},{mean([r['vio_dp'] for r in x]):+6.3f}] "
              f"|dTilt|={mean([r['vio_tilt'] for r in x]):5.3f}deg | "
              f"FC dRP=[{mean([r['fc_dr'] for r in x]):+6.3f},{mean([r['fc_dp'] for r in x]):+6.3f}] "
              f"|dTilt|={mean([r['fc_tilt'] for r in x]):5.3f}deg match={mean([abs(r['match_ms']) for r in x]):.1f}ms")

print("\n================ PER-RUN END ================")
for r in [x for x in out if x["bin"]==1.0]:
    ratio=r["vio_tilt"]/r["fc_tilt"] if r["fc_tilt"]>1e-9 else float("inf")
    print(f"{r['run']} {r['direction']}: VIO dRP=[{r['vio_dr']:+.3f},{r['vio_dp']:+.3f}] tilt={r['vio_tilt']:.3f}deg | "
          f"FC dRP=[{r['fc_dr']:+.3f},{r['fc_dp']:+.3f}] tilt={r['fc_tilt']:.3f}deg | ratio={ratio:.2f}")

print("\nDECISION:")
print("- FC and VIO tilt changes comparable in sign/magnitude => attitude motion is already present in FC estimate, not invented only by Kimera.")
print("- FC nearly stable while VIO tilts by degrees => Kimera orientation/gravity handling becomes primary suspect.")
print("- This compares estimators; it does not prove the rig physically tilted. Use raw gravity-vector and external video as independent references.")
