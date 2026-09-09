#!/usr/bin/env python3
"""V44.12: locate when two already-recorded Kimera runs start to diverge.

No new physical run is required.  The comparison is progress-normalized because
the two recordings do not have identical keyframe timing/counts.
"""
import argparse,csv,math
from pathlib import Path

def loadcsv(p):
    if not p.exists(): return []
    with p.open(newline="") as f: return list(csv.DictReader(f))
def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d

def leg(run):
    rows=loadcsv(run/"jtzero_500mm_v25_legs.csv")
    if not rows: raise RuntimeError(f"missing legs CSV in {run}")
    r=rows[0]
    return I(r,"start_settled_kf"),I(r,"end_press_kf")

def series(run):
    k0,k1=leg(run)
    b=loadcsv(run/"jtzero_500mm_v25_backend.csv")
    f=loadcsv(run/"jtzero_500mm_v25_frontend.csv")
    byf={I(r,"keyframe"):r for r in f if I(r,"is_keyframe")==1}
    bb=[r for r in b if k0<=I(r,"keyframe")<=k1]
    if len(bb)<2: raise RuntimeError(f"insufficient backend rows in {run}")
    x0,y0,z0=F(bb[0],"px_m"),F(bb[0],"py_m"),F(bb[0],"pz_m")
    out=[]
    n=max(1,len(bb)-1)
    for j,r in enumerate(bb):
        k=I(r,"keyframe"); fr=byf.get(k,{})
        dx=(F(r,"px_m")-x0)*1000; dy=(F(r,"py_m")-y0)*1000; dz=(F(r,"pz_m")-z0)*1000
        out.append(dict(p=j/n,kf=k,t=I(r,"timestamp_ns"),dx=dx,dy=dy,dz=dz,
            horiz=math.hypot(dx,dy),status=fr.get("mono_status",""),
            inlier=F(fr,"mono_inlier_ratio"),tracked=F(fr,"tracked_features"),
            mtx=F(fr,"mono_body_tx"),mty=F(fr,"mono_body_ty"),mtz=F(fr,"mono_body_tz")))
    return out

def interp(s,p,key):
    if p<=s[0]["p"]: return s[0][key]
    if p>=s[-1]["p"]: return s[-1][key]
    x=p*(len(s)-1); i=int(math.floor(x)); a=x-i
    return s[i][key]*(1-a)+s[i+1][key]*a

def nearest(s,p): return s[min(range(len(s)),key=lambda i:abs(s[i]["p"]-p))]

def main():
    ap=argparse.ArgumentParser(description="V44.12 cumulative backend divergence on two existing runs")
    ap.add_argument("--reference",required=True); ap.add_argument("--current",required=True)
    ap.add_argument("--step",type=float,default=.05,help="normalized progress step")
    a=ap.parse_args(); A=series(Path(a.reference)); B=series(Path(a.current))
    ps=[]; p=0.0
    while p<1.0+1e-9: ps.append(min(p,1.0)); p+=a.step
    if ps[-1]<1: ps.append(1.0)
    print("="*150)
    print("V44.12 — PROGRESS-NORMALIZED CUMULATIVE KIMERA DIVERGENCE (NO NEW PHYSICAL RUN)")
    print("="*150)
    print(f"reference backend rows={len(A)} current={len(B)}")
    print("prog | ref_h mm cur_h mm delta_h | ref_dx cur_dx delta_x | ref_dy cur_dy | current frontend(status,inlier,tracked)")
    print("-"*150)
    vals=[]
    for p in ps:
        ah=interp(A,p,"horiz"); bh=interp(B,p,"horiz"); ax=interp(A,p,"dx"); bx=interp(B,p,"dx")
        ay=interp(A,p,"dy"); by=interp(B,p,"dy"); q=nearest(B,p); d=bh-ah; vals.append((p,d))
        print(f"{p:4.0%} | {ah:8.1f} {bh:8.1f} {d:+8.1f} | {ax:7.1f} {bx:7.1f} {bx-ax:+7.1f} | {ay:7.1f} {by:7.1f} | {q['status'] or '-':>8s} {q['inlier']:6.3f} {q['tracked']:6.0f}")
    final=vals[-1][1]
    target=.25*abs(final)
    onset=next(((p,d) for p,d in vals if abs(d)>=target),None)
    print("\nDIVERGENCE LANDMARK")
    print("-"*150)
    print(f"final horizontal delta={final:+.1f} mm; 25% threshold={target:.1f} mm")
    if onset:
        p,d=onset; ar=nearest(A,p); br=nearest(B,p)
        print(f"first sampled persistent-scale landmark: progress~{p:.0%}, delta={d:+.1f} mm, ref_kf={ar['kf']}, current_kf={br['kf']}")
        print(f"current frontend there: status={br['status'] or '-'} inlier={br['inlier']:.3f} tracked={br['tracked']:.0f}")
    print("\nREADING RULE")
    print("-"*150)
    print("Early monotonic divergence with comparable frontend quality -> inspect backend/IMU fusion first.")
    print("A sharp divergence coincident with VALID/inlier collapse -> inspect rejected/weak visual updates first.")
    print("This script localizes correlation; it does not claim causality.")
    print("="*150)
if __name__=="__main__": main()
