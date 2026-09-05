#!/usr/bin/env python3
import csv, glob, math, os, sys

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def dist3(a,b):
    return math.sqrt(sum((F(a[k])-F(b[k]))**2 for k in ("px_m","py_m","pz_m")))
def distxy(a,b):
    return math.hypot(F(a["px_m"])-F(b["px_m"]),F(a["py_m"])-F(b["py_m"]))

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v25_*")))
if not roots:
    raise SystemExit("No archived V25 runs found")

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v25_backend.csv")
    lp=os.path.join(root,"jtzero_500mm_v25_legs.csv")
    ap=os.path.join(root,"jtzero_500mm_v25_attitude.csv")
    if not(os.path.exists(bp) and os.path.exists(lp)):
        continue
    backend=load(bp); legs=load(lp); bykf={I(r["keyframe"]):r for r in backend}
    print(f"================ {os.path.basename(root)} ================")
    if len(legs)<4:
        print(f"INCOMPLETE: legs={len(legs)} expected=4")
        continue

    for cyc,(i,j) in enumerate(((0,1),(2,3)),1):
        l1,l2=legs[i],legs[j]
        s=bykf.get(I(l1["start_settled_kf"]))
        b=bykf.get(I(l1["end_settled_kf"]))
        ret=bykf.get(I(l2["end_settled_kf"]))
        if not s or not b or not ret:
            print(f"CYCLE {cyc}: missing backend state")
            continue
        cxy=distxy(s,ret)*1000.0
        c3=dist3(s,ret)*1000.0
        dz=(F(ret["pz_m"])-F(s["pz_m"]))*1000.0
        ab=distxy(s,b)*1000.0
        ba=distxy(b,ret)*1000.0
        print(f"CYCLE {cyc}: A->B={ab:.2f}mm B->A={ba:.2f}mm | RETURN closureXY={cxy:.2f}mm closure3D={c3:.2f}mm dz={dz:+.2f}mm")
        print(f"  Astart KF={I(l1['start_settled_kf'])} B KF={I(l1['end_settled_kf'])} Areturn KF={I(l2['end_settled_kf'])}")
    print()
