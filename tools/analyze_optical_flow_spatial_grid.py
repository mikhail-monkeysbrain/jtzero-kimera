#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r[k])
    except:return d

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("--physical-mm",type=float,required=True)
    args=ap.parse_args()

    rows=list(csv.DictReader(args.csv.open(newline="")))
    move=[r for r in rows if int(f(r,"guide_stage",-1))==1]
    if not move: raise SystemExit("guide_stage=1 not found")

    cells=[]
    for ci in range(9):
        dx=dy=0.0
        used=0
        nsum=0
        for r in move:
            if int(f(r,"flow_sent",0))!=1: continue
            dt=f(r,"dt_s"); rng=f(r,"range_to_fc_m")
            n=int(f(r,f"c{ci}_n",0))
            bx=f(r,f"c{ci}_bx"); by=f(r,f"c{ci}_by")
            if n<3 or not all(map(math.isfinite,[dt,rng,bx,by])) or not (0<dt<0.2) or rng<=0:
                continue
            dx += bx*rng*dt
            dy += by*rng*dt
            used += 1
            nsum += n
        mm=1000*math.hypot(dx,dy)
        ratio=mm/args.physical_mm
        cells.append((ci,mm,ratio,used,nsum/max(1,used)))

    print("===== 3x3 SPATIAL FLOW FORENSIC =====")
    print("Cell indexing inside configured ROI:")
    print("  c0 c1 c2")
    print("  c3 c4 c5")
    print("  c6 c7 c8")
    print()
    for ci,mm,ratio,used,nmean in cells:
        print(f"c{ci}: displacement={mm:7.1f} mm  ratio={ratio:6.3f}  used={used:4d}  mean_inliers={nmean:5.1f}")
    print()
    vals=[x[2] for x in cells if x[3]>=20]
    if vals:
        print(f"cell ratio min/median/max = {min(vals):.3f}/{sorted(vals)[len(vals)//2]:.3f}/{max(vals):.3f}")
        spread=max(vals)-min(vals)
        print(f"spatial spread = {spread:.3f}")
        if spread>0.20:
            print("INTERPRETATION: strong spatial inconsistency remains; some ROI regions still observe different motion.")
        else:
            print("INTERPRETATION: cells are broadly consistent; remaining scale loss is global, not localized.")

if __name__=="__main__":main()
