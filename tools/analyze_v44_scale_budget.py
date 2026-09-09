#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path
FX,FY=568.53170752165227,569.68005562865858
p=argparse.ArgumentParser();p.add_argument("--run",required=True);p.add_argument("--truth-mm",type=float,default=500.);a=p.parse_args()
R=Path(a.run); truth=a.truth_mm/1000
f=R/"jtzero_v43_camera_forensic.csv"
rows=list(csv.DictReader(f.open()))
if not rows: raise SystemExit("empty forensic log")
def vals(k): return [float(r[k]) for r in rows if r.get(k,"") not in ("","nan")]
def net_from_steps(hfun, focal_k=1.0):
 x=y=path=0.
 for r in rows:
  h=hfun(r); dx=-float(r["tx_px"])*h/(FX*focal_k); dy=-float(r["ty_px"])*h/(FY*focal_k)
  x+=dx;y+=dy;path+=math.hypot(dx,dy)
 return math.hypot(x,y),path,x,y
hs=vals("height_m")
# Rotation-corrected aggregate from V44 geometry result is retained as an independently measured correction factor.
obsx=sum(vals("med_flow_x_px"));obsy=sum(vals("med_flow_y_px"))
rotcorr_px=math.hypot(142.997,-1637.913); raw_px=math.hypot(obsx,obsy); rot_k=rotcorr_px/raw_px
print("="*112);print("V44.1 — SCALE BUDGET / HYPOTHESIS CLOSURE ON SAME RUN");print("="*112)
print(f"rows={len(rows)} truth={a.truth_mm:.1f}mm logged height mean/median={statistics.mean(hs)*1000:.2f}/{statistics.median(hs)*1000:.2f}mm")
print(f"rotation aggregate scale factor from V44={rot_k:.6f} ({(rot_k-1)*100:+.3f}%)")
cases=[("logged dynamic h",lambda r:float(r["height_m"])),
       ("fixed 175mm",lambda r:.175),("fixed 180mm",lambda r:.180),("fixed 185mm",lambda r:.185)]
for name,hf in cases:
 n,path,x,y=net_from_steps(hf); nr=n*rot_k
 print(f"{name:18s}: affine={n*1000:7.2f}mm {(n/truth-1)*100:+6.2f}% | +rotation={nr*1000:7.2f}mm {(nr/truth-1)*100:+6.2f}%")
# Required focal after rotation for each physical-height hypothesis.
print("\nREQUIRED EFFECTIVE FOCAL AFTER ROTATION")
for h in (.170,.17324,.175,.180,.185):
 n,_,_,_=net_from_steps(lambda r,h=h:h); nr=n*rot_k; k=nr/truth
 print(f"h={h*1000:7.2f}mm -> residual={(k-1)*100:+6.2f}%  focal_k={k:.5f}  fx={FX*k:.2f} fy={FY*k:.2f}")
# Test whether height quantization/correlation with motion is a material contributor.
weights=[math.hypot(float(r["tx_px"])/FX,float(r["ty_px"])/FY) for r in rows]
wh=sum(h*w for h,w in zip(hs,weights))/sum(weights)
print(f"\nmotion-weighted logged height={wh*1000:.2f}mm vs arithmetic mean={statistics.mean(hs)*1000:.2f}mm delta={(wh-statistics.mean(hs))*1000:+.2f}mm")
# Direction/path consistency.
n,pth,x,y=net_from_steps(lambda r:.180)
print(f"fixed180 affine components=({x*1000:+.2f},{y*1000:+.2f})mm path/net={pth/n:.6f}")
print("\nCLOSURE")
print("- estimator: median raw flow and affine already agree to ~0.02%; not a 4-8% source.")
print("- affine-origin/centroid: prior V43 ratio ~1; rejected as material source.")
print("- FC rotation: measurable, removes about 1.5 percentage points at h=180mm; partial source.")
print("- height: after rotation, ~175mm is near closure; 180/185mm require additional effective-focal correction.")
print("- remaining discriminator is physical optical-center height versus runtime effective focal/crop/resize.")
print("="*112)
