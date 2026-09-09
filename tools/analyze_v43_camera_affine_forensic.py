#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path
p=argparse.ArgumentParser(); p.add_argument("--run",required=False); p.add_argument("--file",required=False); a=p.parse_args()\nif not a.run and not a.file: p.error("one of --run or --file is required")
f=Path(a.file) if a.file else Path(a.run)/"jtzero_v43_camera_forensic.csv"
rows=list(csv.DictReader(f.open()))
if not rows:
 print("="*100); print("V43 — CAMERA AFFINE FORENSIC"); print("="*100)
 print("accepted updates in forensic CSV: 0")
 print("VERDICT: FORENSIC LOG EMPTY — do not interpret affine statistics.")
 print("The physical V43 run itself remains valid; only the added forensic observability failed.")
 print("="*100)
 raise SystemExit(2)
def vals(k): return [float(r[k]) for r in rows]
def q(v,x): return sorted(v)[min(len(v)-1,max(0,int(round((len(v)-1)*x))))]
tx,ty,sc,rot,h,dx,dy,st=map(vals,["tx_px","ty_px","aff_scale","aff_rot_deg","height_m","dx_m","dy_m","step_m"])
c0x,c0y=vals("c0x"),vals("c0y")
nx=sum(dx); ny=sum(dy); net=math.hypot(nx,ny)
# Translation represented by affine at optical axis differs from translation at inlier centroid by (A-I)c.
# Compute centroid image displacement implied by the logged affine.
cent_dx=[]; cent_dy=[]; axis_vs_cent=[]
for r in rows:
 A=float(r["aff_a"]); B=float(r["aff_b"]); x=float(r["c0x"])-315.98271077441063; y=float(r["c0y"])-239.88148589100641
 txx=float(r["tx_px"]); tyy=float(r["ty_px"])
 # matrix [A -B; B A]
 ddx=txx+(A-1)*x-B*y; ddy=tyy+B*x+(A-1)*y
 cent_dx.append(ddx); cent_dy.append(ddy); axis_vs_cent.append(math.hypot(txx,tyy)/max(1e-12,math.hypot(ddx,ddy)))
print("="*100); print("V43 — CAMERA AFFINE FORENSIC"); print("="*100)
print("accepted updates:",len(rows))
print(f"logged camera net: {net*1000:.2f} mm  components=({nx*1000:.2f},{ny*1000:.2f})")
for name,v in [("height m",h),("affine scale",sc),("|rotation| deg",[abs(x) for x in rot]),("|tx,ty| px",[math.hypot(x,y) for x,y in zip(tx,ty)]),("step mm",[1000*x for x in st]),("centroid x px",c0x),("centroid y px",c0y),("axis/centroid affine-motion ratio",axis_vs_cent)]:
 print(f"{name:34s} mean={statistics.mean(v):.6f} median={statistics.median(v):.6f} p90={q(v,.9):.6f} max={max(v):.6f}")
print("\nSUMMED PIXEL MOTION")
print(f"affine translation at optical axis: tx={sum(tx):.3f} px ty={sum(ty):.3f} px norm={math.hypot(sum(tx),sum(ty)):.3f} px")
print(f"affine-predicted motion at inlier centroid: dx={sum(cent_dx):.3f} px dy={sum(cent_dy):.3f} px norm={math.hypot(sum(cent_dx),sum(cent_dy)):.3f} px")
print("\nINTERPRETATION")
print("The V41/V42/V43 metric estimator integrates affine tx/ty evaluated at the optical-axis-centered origin.")
print("If affine scale/rotation differs from identity and inlier centroids are off-axis, tx/ty is not the same as feature-centroid translation.")
print("A large systematic axis/centroid difference is direct evidence that similarity-transform parameterization contaminates metric translation.")
print("="*100)

print("\n500 MM RECONCILIATION")
print(f"net/truth scale={net/0.5:.6f} error={(net-0.5)*1000:+.2f} mm")
print(f"path={sum(st)*1000:.2f} mm path/net={sum(st)/net:.6f}")
print(f"summed axis pixel vector / net = {math.hypot(sum(tx),sum(ty))/net:.3f} px/m")
