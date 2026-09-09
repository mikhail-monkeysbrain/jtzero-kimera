#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path
FX=568.53170752165227
FY=569.68005562865858
p=argparse.ArgumentParser(description="Infer effective focal length required by real V43 tx/ty")
p.add_argument("--file",required=True)
p.add_argument("--truth-mm",type=float,default=500.0)
a=p.parse_args()
with Path(a.file).open(newline="") as f:
    rows=list(csv.DictReader(f))
tx=sum(float(r["tx_px"]) for r in rows)
ty=sum(float(r["ty_px"]) for r in rows)
truth=a.truth_mm/1000.0
print("="*100)
print("V43 — EFFECTIVE FOCAL LENGTH RECONCILIATION")
print("="*100)
print(f"summed tx={tx:.3f} px  ty={ty:.3f} px")
print(f"calibration fx={FX:.3f} px fy={FY:.3f} px")
for hmm in [170.0,171.0,175.0,180.0,185.0,190.0,195.0]:
    h=hmm/1000.0
    # scale both focal lengths by a common k; net metric translation scales as 1/k.
    dx=-tx*h/FX
    dy=-ty*h/FY
    base=math.hypot(dx,dy)
    k=base/truth
    print(f"h={hmm:6.1f} mm -> current net={base*1000:7.2f} mm, required focal scale k={k:7.4f}, "
          f"fx_eff={FX*k:7.2f}px fy_eff={FY*k:7.2f}px")
print()
print("Interpretation: if physically plausible h=180..185 mm requires fx/fy ~5..8% larger than calibration,")
print("the next check is whether the actual runtime 640x480 USB stream uses the same optical/crop geometry as calibration.")
print("="*100)
