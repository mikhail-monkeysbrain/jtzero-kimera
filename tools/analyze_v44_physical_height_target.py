#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

FX,FY=568.53170752165227,569.68005562865858
p=argparse.ArgumentParser(description="V44.2 physical-height closure and focal plausibility from same run")
p.add_argument("--run",required=True)
p.add_argument("--truth-mm",type=float,default=500.0)
a=p.parse_args(); R=Path(a.run); truth=a.truth_mm/1000.0
rows=list(csv.DictReader((R/"jtzero_v43_camera_forensic.csv").open()))
if not rows: raise SystemExit("empty forensic CSV")

# Reuse empirically measured rotation aggregate factor from V44 exact same run.
ROT_K=0.985263
tx=sum(float(r["tx_px"]) for r in rows); ty=sum(float(r["ty_px"]) for r in rows)
unit=math.hypot(tx/FX,ty/FY)*ROT_K
req_h=truth/unit
print("="*110)
print("V44.2 — PHYSICAL HEIGHT CLOSURE TARGET")
print("="*110)
print(f"rows={len(rows)} rotation factor={ROT_K:.6f}")
print(f"required optical-center height for exact 500 mm with calibrated focal + rotation correction: {req_h*1000:.2f} mm")
print()
for h in [0.170,0.173,0.175,0.180,0.185]:
    net=unit*h
    print(f"h={h*1000:6.1f} mm -> corrected camera-only={net*1000:7.2f} mm  residual={(net/truth-1)*100:+6.2f}%")
print()
print("DECISION")
print("- If the physically measured OV9281 optical center is ~173 mm above the floor during the run, height geometry closes the camera-only scale without changing focal.")
print("- If physical optical-center height is materially nearer 180-185 mm, the remaining 4-7% must be assigned to effective focal/runtime image geometry or another unmodelled projection effect.")
print("- This script defines the exact physical measurement needed next; no A->B run is required.")
print("="*110)
