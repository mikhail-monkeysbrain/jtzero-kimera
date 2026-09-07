#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def summarize(run):
    run=Path(run)
    legs=load(run/"jtzero_500mm_v25_legs.csv")
    back=load(run/"jtzero_500mm_v25_backend.csv")
    front=load(run/"jtzero_500mm_v25_frontend.csv")
    bykf={I(r,"keyframe"):r for r in back}
    fk=[r for r in front if I(r,"is_keyframe")==1]

    out=[]
    for L in legs:
        ks=I(L,"start_settled_kf"); ke=I(L,"end_settled_kf")
        br=[r for r in back if ks<=I(r,"keyframe")<=ke]
        if not br: continue
        t0=I(br[0],"timestamp_ns"); t1=I(br[-1],"timestamp_ns")
        fr=[r for r in fk if t0<=I(r,"timestamp_ns")<=t1]
        statuses={}
        for r in fr: statuses[r["mono_status"]]=statuses.get(r["mono_status"],0)+1
        valid=statuses.get("VALID",0); low=statuses.get("LOW_DISPARITY",0)
        ratios=[F(r,"mono_inlier_ratio") for r in fr if r["mono_status"]=="VALID"]
        speeds=[F(r,"speed_m_s")*1000 for r in br]
        dur=(t1-t0)*1e-9
        xy=F(L,"horizontal_m")*1000
        out.append(dict(
            leg=I(L,"leg"),direction=L["direction"],xy_mm=xy,scale=xy/500.0,
            dz_mm=F(L,"dz_m")*1000,duration_s=dur,
            mean_speed_mm_s=mean(speeds),max_speed_mm_s=max(speeds) if speeds else float("nan"),
            valid=valid,low=low,valid_ratio=valid/max(1,len(fr)),
            valid_inlier_mean=mean(ratios),valid_inlier_min=min(ratios) if ratios else float("nan")
        ))
    return out

if len(sys.argv)!=3:
    raise SystemExit("usage: compare_v25_archived_runs.py <runA> <runB>")

A=summarize(sys.argv[1]); B=summarize(sys.argv[2])
print("================ V25 ARCHIVED RUN COMPARISON ================")
print("A:",sys.argv[1]); print("B:",sys.argv[2])
for a,b in zip(A,B):
    print(f"\nLEG {a['leg']} {a['direction']}")
    for label,r in (("A",a),("B",b)):
        print(f"  {label}: XY={r['xy_mm']:.2f}mm scale={r['scale']:.4f} dz={r['dz_mm']:+.1f}mm "
              f"dur={r['duration_s']:.2f}s mean/max speed={r['mean_speed_mm_s']:.1f}/{r['max_speed_mm_s']:.1f}mm/s "
              f"VALID={r['valid']} LOW={r['low']} validFrac={r['valid_ratio']*100:.1f}% "
              f"VALID inlier mean/min={r['valid_inlier_mean']:.3f}/{r['valid_inlier_min']:.3f}")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    aa=[r for r in A if r["direction"]==d]; bb=[r for r in B if r["direction"]==d]
    if aa and bb:
        def m(rr,k): return mean([r[k] for r in rr])
        print(f"{d}: A scale={m(aa,'scale'):.4f} dur={m(aa,'duration_s'):.2f}s maxSpd={m(aa,'max_speed_mm_s'):.1f} "
              f"| B scale={m(bb,'scale'):.4f} dur={m(bb,'duration_s'):.2f}s maxSpd={m(bb,'max_speed_mm_s'):.1f}")

print("\nINTERPRETATION:")
print("- Large scale changes together with large duration/speed/visual-quality changes => run-to-run motion excitation is a major confounder.")
print("- Similar motion/visual metrics but different scale => optimizer parameter difference is more likely causal.")
print("- A single manual A-B-A run is not sufficient for final parameter ranking if motion profiles differ strongly.")
