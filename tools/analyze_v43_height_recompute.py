#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

FX=568.53170752165227
FY=569.68005562865858

p=argparse.ArgumentParser(description="V43 height-only recomputation from logged tx/ty")
p.add_argument("--file",required=True)
p.add_argument("--truth-mm",type=float,default=500.0)
args=p.parse_args()

with Path(args.file).open(newline="") as f:
    rows=list(csv.DictReader(f))
if not rows:
    raise SystemExit("empty forensic CSV")

tx=[float(r["tx_px"]) for r in rows]
ty=[float(r["ty_px"]) for r in rows]
h=[float(r["height_m"]) for r in rows]

def integrate(hsel):
    sx=sy=path=0.0
    if isinstance(hsel,(int,float)):
        hs=[float(hsel)]*len(rows)
    else:
        hs=hsel
    for a,b,hh in zip(tx,ty,hs):
        dx=-a*hh/FX
        dy=-b*hh/FY
        sx+=dx; sy+=dy; path+=math.hypot(dx,dy)
    return sx,sy,math.hypot(sx,sy),path

truth=args.truth_mm/1000.0
cases=[
    ("logged dynamic height",h),
    ("fixed 195 mm",0.195),
    ("fixed 190 mm",0.190),
    ("fixed 185 mm",0.185),
    ("fixed 180 mm",0.180),
]
print("="*104)
print("V43 — HEIGHT RECOMPUTATION FROM REAL LOGGED tx/ty")
print("="*104)
print(f"rows={len(rows)}  logged h mean={statistics.mean(h)*1000:.2f} mm median={statistics.median(h)*1000:.2f} mm min/max={min(h)*1000:.1f}/{max(h)*1000:.1f} mm")
print()
for name,hh in cases:
    sx,sy,net,path=integrate(hh)
    print(f"{name:24s}: net={net*1000:8.2f} mm  error={(net-truth)*1000:+7.2f} mm  scale={net/truth:7.4f}  path={path*1000:8.2f} mm")

# Because every step is linear in a common fixed height, infer the exact fixed h that would make net=truth.
_,_,unit_net,_=integrate(1.0)
h_req=truth/unit_net
sx,sy,net,path=integrate(h_req)
print()
print("REQUIRED FIXED HEIGHT")
print("-"*104)
print(f"fixed height required for exactly {args.truth_mm:.1f} mm with the same logged tx/ty: {h_req*1000:.2f} mm")
print(f"relative to 180 mm TF-Luna reference: {(h_req-0.180)*1000:+.2f} mm")
print(f"relative to 185 mm camera-plane estimate: {(h_req-0.185)*1000:+.2f} mm")
print()
print("INTERPRETATION")
print("-"*104)
print("This test changes ONLY the height term; tx/ty are the actual V43 affine translations.")
print("If fixed 180/185 mm still leaves a large excess, dynamic TF-Luna quantization/height choice is not sufficient.")
print("The required fixed height is an empirical scale reconciliation, not proof that the camera is physically at that height.")
print("="*104)
