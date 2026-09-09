#!/usr/bin/env python3
"""V44.26: compare clean-run backend displacement against camera-only progress.

Unlike keyframe-count normalization, camera-progress normalization asks whether
reference/current agree at the same observed image displacement. No physical
run is required.
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
    if not q: raise RuntimeError(f"missing legs CSV: {run}")
    r=q[0]; return I(r,"start_settled_kf"),I(r,"end_press_kf")
def backend(run):
    k0,k1=leg(run); q=[r for r in load(run/"jtzero_500mm_v25_backend.csv") if k0<=I(r,"keyframe")<=k1]
    if len(q)<2: raise RuntimeError(f"insufficient backend rows: {run}")
    x0,y0=F(q[0],"px_m"),F(q[0],"py_m"); xe,ye=F(q[-1],"px_m"),F(q[-1],"py_m")
    ux,uy=xe-x0,ye-y0; n=math.hypot(ux,uy); ux,uy=(ux/n,uy/n) if n else (1,0)
    return [(I(r,"timestamp_ns"),((F(r,"px_m")-x0)*ux+(F(r,"py_m")-y0)*uy)*1000) for r in q]
def camera(run):
    q=load(run/"jtzero_v43_camera_forensic.csv")
    if len(q)<2: raise RuntimeError(f"insufficient camera forensic rows: {run}")
    # Camera log already contains cumulative net_x/net_y. Use radial net as a
    # monotonic-ish observed progress coordinate; enforce nondecreasing envelope.
    raw=[(i/max(1,len(q)-1),math.hypot(F(r,"net_x_m"),F(r,"net_y_m"))*1000) for i,r in enumerate(q)]
    out=[]; m=0.0
    for t,v in raw:
        if math.isfinite(v): m=max(m,v)
        out.append((t,m))
    return out
def pairs(run):
    b=backend(run); c=camera(run); z=[]
    # backend() returns (timestamp_ns, displacement_mm), while camera() returns
    # (normalized_leg_progress, camera_net_mm). These clocks are intentionally
    # NOT compared. Map each backend sample to the same normalized leg progress.
    for i,(_,x) in enumerate(b):
        p=i/max(1,len(b)-1)
        j=min(range(len(c)), key=lambda k: abs(c[k][0]-p))
        cp,cm=c[j]
        z.append((cm,x,abs(cp-p)))
    # Camera net is the interpolation axis. Duplicate/plateau camera-progress
    # values are harmless; interp() handles them.
    return sorted(z,key=lambda a:a[0])

