#!/usr/bin/env python3
"""V44.25: compare cumulative Kimera scale across four existing runs.

The goal is to separate a common clean-run residual from the transient 417 mm
failure. No new physical run is required.
"""
import argparse,csv,math
from pathlib import Path

def load(p):
    if not p.exists(): return []
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def leg(run):
    q=load(run/"jtzero_500mm_v25_legs.csv")
    if not q: raise RuntimeError(f"missing legs CSV in {run}")
    r=q[0]; return I(r,"start_settled_kf"),I(r,"end_press_kf"),F(r,"horizontal_m")*1000
def series(run):
    k0,k1,end=leg(run); b=load(run/"jtzero_500mm_v25_backend.csv")
    q=[r for r in b if k0<=I(r,"keyframe")<=k1]
    if len(q)<2: raise RuntimeError(f"insufficient backend rows in {run}")
    x0,y0=F(q[0],"px_m"),F(q[0],"py_m")
    dx=(F(q[-1],"px_m")-x0)*1000; dy=(F(q[-1],"py_m")-y0)*1000
    n=math.hypot(dx,dy)
    ux,uy=(dx/n,dy/n) if n>0 else (1,0)
    out=[]
    for j,r in enumerate(q):
        x=(F(r,"px_m")-x0)*1000; y=(F(r,"py_m")-y0)*1000
        out.append((j/(len(q)-1),x*ux+y*uy))
    return out,end
def interp(s,p):
    x=p*(len(s)-1); i=int(math.floor(x)); j=min(i+1,len(s)-1); a=x-i
    return s[i][1]*(1-a)+s[j][1]*a

ap=argparse.ArgumentParser()
for x in ("reference","bad","gated1","current"): ap.add_argument("--"+x,required=True)
a=ap.parse_args()
runs={k:Path(v).expanduser() for k,v in vars(a).items()}
S={}; ends={}
for k,p in runs.items(): S[k],ends[k]=series(p)

print("="*116)
print("V44.25 — FOUR-RUN CUMULATIVE SCALE SHAPE / CLEAN-RESIDUAL SCREEN")
print("="*116)
print("progress | reference    bad   gated1  current | bad-ref gated1-ref current-ref | ideal500")
print("-"*116)
for i in range(0,21):
    p=i/20
    v={k:interp(s,p) for k,s in S.items()}
    print(f"{p:7.0%} | {v['reference']:9.1f} {v['bad']:7.1f} {v['gated1']:8.1f} {v['current']:8.1f} |"
          f" {v['bad']-v['reference']:+7.1f} {v['gated1']-v['reference']:+10.1f} {v['current']-v['reference']:+11.1f} | {500*p:8.1f}")

print("\nENDPOINTS")
print("-"*116)
for k in ("reference","bad","gated1","current"):
    print(f"{k:9s}: {ends[k]:7.2f} mm  error={ends[k]-500:+7.2f} mm  scale={ends[k]/500:.5f}")

clean=[ends["reference"],ends["current"]]
mean=sum(clean)/len(clean); spread=max(clean)-min(clean)
print("\nCLEAN-RUN RESIDUAL")
print("-"*116)
print(f"reference/current mean={mean:.2f} mm; mean error={mean-500:+.2f} mm; pair spread={spread:.2f} mm")
print(f"bad deficit vs clean mean={ends['bad']-mean:+.2f} mm; gated1 deficit={ends['gated1']-mean:+.2f} mm")
print("\nDECISION")
print("-"*116)
print("If reference and current track closely over most progress, treat ~484 mm as the repeatable clean baseline.")
print("If bad/gated1 peel away at different progress while camera net remains similar, do not apply one global focal/scale correction to explain them.")
print("A common ~3% clean-run shortfall may be calibrated only after its cumulative shape is shown to be repeatable; this script tests that premise.")
print("="*116)
