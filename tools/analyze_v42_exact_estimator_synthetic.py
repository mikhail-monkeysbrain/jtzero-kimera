#!/usr/bin/env python3
"""Synthetic sensitivity of the exact V42 similarity estimator.

No images/run are needed. Generates planar feature coordinates, projects the same
500-mm ground translation at h=180 mm in small increments, applies the calibrated
OV9281 distortion model, then fits OpenCV-equivalent similarity by least squares
(the noise-free counterpart of estimateAffinePartial2D) and integrates tx/f.
"""
import math, re
from pathlib import Path
import numpy as np

YAML=Path("params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003/LeftCameraParams.yaml")
txt=YAML.read_text()
def arr(k):
    m=re.search(r"^\s*"+re.escape(k)+r"\s*:\s*\[([^\]]+)\]",txt,re.M)
    return [float(x) for x in m.group(1).split(",")]
fx,fy,cx,cy=arr("intrinsics")
k1,k2,p1,p2,k3=arr("distortion_coefficients")
W,H=640,480
h=.180

def distort(uv):
    x=(uv[:,0]-cx)/fx; y=(uv[:,1]-cy)/fy
    r2=x*x+y*y
    rad=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    xd=x*rad+2*p1*x*y+p2*(r2+2*x*x)
    yd=y*rad+p1*(r2+2*y*y)+2*p2*x*y
    return np.c_[fx*xd+cx,fy*yd+cy]

def simfit(a,b):
    # b ~= [aa -bb; bb aa] a + t
    n=len(a); M=np.zeros((2*n,4)); y=b.reshape(-1)
    for i,(x,z) in enumerate(a):
        M[2*i]=[x,-z,1,0]; M[2*i+1]=[z,x,0,1]
    q=np.linalg.lstsq(M,y,rcond=None)[0]
    return q # alpha,beta,tx,ty

# Uniform feature grid with margin, representative of goodFeatures coverage.
u=np.linspace(45,595,12); v=np.linspace(40,440,9)
raw=np.array([(x,y) for y in v for x in u],float)

# Back-compute approximate undistorted rays by fixed-point iteration.
und=raw.copy()
for _ in range(8):
    d=distort(und); und += raw-d

# Simulate image displacement corresponding to horizontal ground motion.
# Camera image motion magnitude = dx*fx/h. Integrate 500 mm in 217 updates,
# matching V42 good_updates, while allowing small vertical component and tilt-like
# nonuniform warp via a projective denominator.
steps=217
dx_total=.500
base_px=(dx_total/steps)*fx/h

def run(proj_x, proj_y, dy_frac):
    pts=und.copy(); net=np.zeros(2)
    for _ in range(steps):
        # ideal next undistorted pixel coordinates: translation plus weak projective warp
        q=pts.copy()
        q[:,0]+=base_px
        q[:,1]+=base_px*dy_frac
        xn=(q[:,0]-cx)/fx; yn=(q[:,1]-cy)/fy
        den=1.0+proj_x*xn+proj_y*yn
        q[:,0]=cx+(q[:,0]-cx)/den
        q[:,1]=cy+(q[:,1]-cy)/den
        a=distort(pts)-np.array([cx,cy])
        b=distort(q)-np.array([cx,cy])
        al,be,tx,ty=simfit(a,b)
        net += np.array([-tx*h/fx,-ty*h/fy])
        pts=q
    return np.linalg.norm(net)*1000

cases=[]
for px in [0,.0005,.001,.002,.003,.005]:
  for py in [0,.0005,.001,.002,.003,.005]:
    z=run(px,py,0)
    cases.append((abs(z-500),px,py,z))
cases.sort()
print("="*100)
print("V42 — SYNTHETIC EXACT-ESTIMATOR SENSITIVITY")
print("="*100)
z0=run(0,0,0)
print(f"pure translation + calibrated distortion: {z0:.2f} mm  bias={z0-500:+.2f} mm ({(z0/500-1)*100:+.2f}%)")
print("\nPROJECTIVE SWEEP (dimensionless weak-warp coefficients per frame)")
for target in [0,.0005,.001,.002,.003,.005]:
    vals=[x[3] for x in cases if x[1]==target]
    print(f"proj_x={target:.4f}: range over proj_y sweep {min(vals):.2f} .. {max(vals):.2f} mm")
worst=max(cases,key=lambda x:x[3])
print(f"\nmax in sweep: {worst[3]:.2f} mm at proj_x={worst[1]:.4f}, proj_y={worst[2]:.4f}")
print(f"required V42 camera-only: 551.30 mm at direct h=180 mm")
print("\nNOTE: This is a model-sensitivity screen, not a reconstruction of V42.")
print("If plausible weak projective terms cannot approach 551 mm, the next run must log the affine matrix")
print("and feature centroids directly; no further offline inference can identify the camera-only bias.")
print("="*100)
