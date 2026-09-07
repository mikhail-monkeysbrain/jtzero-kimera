#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def F(r,k): return float(r[k])
def I(r,k): return int(float(r[k]))

def linreg(xs, ys):
    n=len(xs)
    if n<3: return float("nan"),float("nan"),float("nan")
    mx=sum(xs)/n; my=sum(ys)/n
    sxx=sum((x-mx)**2 for x in xs)
    if sxx<=0: return float("nan"),float("nan"),float("nan")
    sxy=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    k=sxy/sxx; b=my-k*mx
    ssr=sum((y-(k*x+b))**2 for x,y in zip(xs,ys))
    sst=sum((y-my)**2 for y in ys)
    r2=1.0-ssr/sst if sst>0 else float("nan")
    return k,b,r2

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_z_coupling_shape.py RUN_DIR")

root=Path(sys.argv[1])
legs=load(root/"jtzero_500mm_v25_legs.csv")
backend=load(root/"jtzero_500mm_v25_backend.csv")

# Fixed physical A->B axis. Do not assume LEG1 is A->B: B-first runs reverse the order.
ab=[r for r in legs if r["direction"]=="A->B"]
if not ab:
    raise SystemExit("no A->B leg found")
# Average unit direction across all A->B legs to reduce dependence on one endpoint.
uv=[]
for q in ab:
    dx,dy=F(q,"dx_m"),F(q,"dy_m")
    n=math.hypot(dx,dy)
    if n>1e-9:
        uv.append((dx/n,dy/n))
ux=sum(v[0] for v in uv)/len(uv); uy=sum(v[1] for v in uv)/len(uv)
nn=math.hypot(ux,uy); ux,uy=ux/nn,uy/nn

print("================ V25 Z-COUPLING SHAPE ================")
print("run:",root)
print(f"common A->B axis u=[{ux:+.6f},{uy:+.6f}]")
print("Test: a fixed geometric XY->Z rotation should produce a similar Z/common-X slope across reversals, with high linearity.")
print()

slopes=[]
for L in legs:
    leg=I(L,"leg")
    skf=I(L,"start_settled_kf")
    ekf=I(L,"end_press_kf")
    seg=[r for r in backend if I(r,"leg")==leg and r["phase"]=="MOVE" and skf<=I(r,"keyframe")<=ekf]
    if len(seg)<3:
        print(f"LEG {leg}: insufficient MOVE states")
        continue

    # Reference to first MOVE state.
    x0=F(seg[0],"px_m"); y0=F(seg[0],"py_m"); z0=F(seg[0],"pz_m")
    xs=[]; zs=[]
    for r in seg:
        dx=F(r,"px_m")-x0; dy=F(r,"py_m")-y0
        xs.append(dx*ux+dy*uy)
        zs.append(F(r,"pz_m")-z0)

    k,b,r2=linreg(xs,zs)
    endpoint_dx=F(L,"dx_m")*ux+F(L,"dy_m")*uy
    endpoint_dz=F(L,"dz_m")
    endpoint_k=endpoint_dz/endpoint_dx if abs(endpoint_dx)>1e-9 else float("nan")
    angle=math.degrees(math.atan(endpoint_k)) if math.isfinite(endpoint_k) else float("nan")

    # Compare early and late halves to detect dynamic/nonlinear coupling.
    mid=max(2,len(xs)//2)
    k1,_,r21=linreg(xs[:mid+1],zs[:mid+1])
    k2,_,r22=linreg(xs[mid:],zs[mid:]) if len(xs[mid:])>=3 else (float("nan"),)*3

    slopes.append(k)
    print(f"LEG {leg} {L['direction']}: n={len(seg)}")
    print(f"  endpoint common-X = {endpoint_dx*1000:+.2f} mm, dz={endpoint_dz*1000:+.2f} mm")
    print(f"  endpoint dz/dX    = {endpoint_k:+.4f}  effective angle={angle:+.2f} deg")
    print(f"  MOVE regression   = dz = {k:+.4f}*X + {b*1000:+.2f}mm, R2={r2:.4f}")
    print(f"  early/late slope  = {k1:+.4f} (R2={r21:.3f}) / {k2:+.4f} (R2={r22:.3f})")
    print()

print("INTERPRETATION:")
print("- Similar slope across all legs + high R2 supports a fixed geometric XY->Z coupling such as an extrinsic rotation error.")
print("- Large slope changes between legs, or poor/nonlinear early-vs-late behavior, argues that fixed T_BS alone is insufficient.")
print("- This test diagnoses shape only; it does not prove which estimator component is causal.")
