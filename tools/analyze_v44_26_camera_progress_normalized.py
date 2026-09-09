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
def nearest_cam(c,t):
    return c[min(range(len(c)),key=lambda i:abs(c[i][0]-t))]
def pairs(run):
    b=backend(run); c=camera(run); z=[]
    for t,x in b:
        ct,cm=nearest_cam(c,t); z.append((cm,x,abs(ct-t)/1e6))
    # keep points with useful camera progress and sort for interpolation
    return sorted(z,key=lambda a:a[0])
def interp(z,target):
    if target<=z[0][0]: return z[0]
    if target>=z[-1][0]: return z[-1]
    for i in range(len(z)-1):
        a,b=z[i],z[i+1]
        if a[0]<=target<=b[0]:
            if b[0]==a[0]: return a
            w=(target-a[0])/(b[0]-a[0])
            return (target,a[1]*(1-w)+b[1]*w,max(a[2],b[2]))
    return z[-1]

ap=argparse.ArgumentParser()
ap.add_argument("--reference",required=True); ap.add_argument("--current",required=True)
a=ap.parse_args(); R=Path(a.reference).expanduser(); C=Path(a.current).expanduser()
A=pairs(R); B=pairs(C)
common=min(max(x[0] for x in A),max(x[0] for x in B))
print("="*116)
print("V44.26 — CLEAN RUNS NORMALIZED BY CAMERA-ONLY OBSERVED PROGRESS")
print("="*116)
print(f"common camera-progress range: 0..{common:.1f} mm")
print("cam progress | ref backend  cur backend  delta | ref backend/cam cur backend/cam | max timestamp dt")
print("-"*116)
for i in range(1,21):
    target=common*i/20
    ar=interp(A,target); br=interp(B,target)
    print(f"{target:11.1f} | {ar[1]:11.1f} {br[1]:11.1f} {br[1]-ar[1]:+7.1f} |"
          f" {ar[1]/target:15.4f} {br[1]/target:15.4f} | {max(ar[2],br[2]):8.2f} ms")
print("\nEND SCREEN")
print("-"*116)
for name,z in (("reference",A),("current",B)):
    cam=z[-1][0]; be=z[-1][1]
    print(f"{name:9s}: camera={cam:.2f}mm backend={be:.2f}mm backend/camera={be/cam:.5f}")
print("\nDECISION")
print("-"*116)
print("If ref/current backend values remain close at equal camera progress, the clean residual is downstream of raw image displacement and repeatable.")
print("If the large V44.25 mid-run separation collapses here, it was mainly a keyframe-count/time-normalization artifact, not a changing metric scale.")
print("Do not apply a production scale coefficient from this diagnostic alone.")
print("="*116)
