#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path
FX,FY=568.53170752165227,569.68005562865858
def mean(v): return statistics.mean(v) if v else float("nan")
def med(v): return statistics.median(v) if v else float("nan")
p=argparse.ArgumentParser();p.add_argument("--file",required=True);p.add_argument("--truth-mm",type=float,default=500.0);a=p.parse_args()
rows=list(csv.DictReader(Path(a.file).open()))
req=["tx_px","ty_px","height_m","med_flow_x_px","med_flow_y_px","flow_left_px","flow_right_px","flow_top_px","flow_bottom_px"]
miss=[x for x in req if not rows or x not in rows[0]]
if miss: raise SystemExit("missing V44 columns: "+",".join(miss))
f=lambda k:[float(r[k]) for r in rows if r.get(k,"") not in ("","nan")]
tx=sum(f("tx_px"));ty=sum(f("ty_px")); truth=a.truth_mm/1000
print("="*108);print("V44 — ONE-PASS CAMERA SCALE HYPOTHESIS SWEEP");print("="*108)
print(f"rows={len(rows)} summed affine tx/ty=({tx:.3f},{ty:.3f}) px")
for h in (.180,.185):
    net=math.hypot(tx*h/FX,ty*h/FY); k=net/truth
    print(f"h={h*1000:.0f} mm: affine net={net*1000:.2f} mm residual={(k-1)*100:+.2f}%  required focal k={k:.4f} -> fx={FX*k:.2f}, fy={FY*k:.2f}")
# Median raw-flow vs affine translation, weighted as a framewise diagnostic, not a trajectory replacement.
mfx=f("med_flow_x_px");mfy=f("med_flow_y_px")
rawx=sum(mfx);rawy=sum(mfy)
print(f"sum per-frame median inlier flow=({rawx:.3f},{rawy:.3f}) px norm={math.hypot(rawx,rawy):.3f}")
print(f"affine norm={math.hypot(tx,ty):.3f} px  median-flow/affine={math.hypot(rawx,rawy)/math.hypot(tx,ty):.4f}")
for x,y,label in [("flow_left_px","flow_right_px","left/right"),("flow_top_px","flow_bottom_px","top/bottom")]:
    A=f(x);B=f(y); ratios=[b/a for a,b in zip(A,B) if a>1e-9 and math.isfinite(a) and math.isfinite(b)]
    print(f"{label} flow median ratio: median={med(ratios):.4f} mean={mean(ratios):.4f}")
hs=f("height_m")
print(f"height logged: mean={mean(hs)*1000:.2f} median={med(hs)*1000:.2f} min={min(hs)*1000:.1f} max={max(hs)*1000:.1f} mm")
print("\nDECISION GUIDE")
print("- focal k ~1.05..1.08 with otherwise uniform flow -> runtime intrinsics/crop/effective-focal hypothesis rises.")
print("- median-flow/affine far from 1 -> estimator/model-fit bias rises.")
print("- left/right or top/bottom ratio materially differs from 1 -> projective/tilt/distortion/spatial-flow branch rises.")
print("- logged height inconsistent with independently measured optical-center height -> height geometry remains causal.")
print("- if all camera diagnostics are internally consistent, verify the physical 500-mm endpoint independently before changing calibration.")
print("="*108)
