#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_bias_warmup_crossrun.py RUN_DIR [RUN_DIR ...]")

for arg in sys.argv[1:]:
    root=Path(arg)
    back=load(root/"jtzero_500mm_v25_backend.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    bykf={I(r,"keyframe"):r for r in back}
    first=min(legs,key=lambda r:I(r,"leg"))
    ks=I(first,"start_settled_kf")
    start=bykf.get(ks)
    pre=[r for r in back if I(r,"keyframe")<ks]
    tail=pre[-8:] if len(pre)>=8 else pre

    print("\n================ RUN ================")
    print(root)
    print(f"first leg: {first['direction']} startKF={ks}")
    if start:
        print(f"bias at first-leg settled start: bax={F(start,'bax'):+.5f} bay={F(start,'bay'):+.5f} baz={F(start,'baz'):+.5f} m/s^2")
        print(f"Vz at first-leg settled start: {F(start,'vz_m_s')*1000:+.2f} mm/s")
    if tail:
        print("last pre-leg backend states:")
        for r in tail:
            print(f"  KF={I(r,'keyframe'):3d} baz={F(r,'baz'):+.5f} Vz={F(r,'vz_m_s')*1000:+.2f}mm/s Pz={F(r,'pz_m')*1000:+.2f}mm")
        print(f"pre-leg mean baz={mean([F(r,'baz') for r in tail]):+.5f} m/s^2")
        print(f"pre-leg baz span={(max(F(r,'baz') for r in tail)-min(F(r,'baz') for r in tail)):.5f} m/s^2")
