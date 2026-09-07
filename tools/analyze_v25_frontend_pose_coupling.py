#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def median(xs):
    if not xs: return float("nan")
    s=sorted(xs); n=len(s)
    return s[n//2] if n%2 else 0.5*(s[n//2-1]+s[n//2])

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_frontend_pose_coupling.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")
back=load(root/"jtzero_500mm_v25_backend.csv")
bykf={I(r,"keyframe"):r for r in back}

required={"mono_pose_valid","mono_body_tx","mono_body_ty","mono_body_tz","mono_body_t_norm"}
if not front or not required.issubset(front[0].keys()):
    raise SystemExit("frontend CSV does not contain mono pose diagnostics; run a new V25 dataset with the updated logger")

# Build one common horizontal mono direction from A->B valid samples.
ab_vecs=[]
for L in legs:
    if L["direction"]!="A->B": continue
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s,e=bykf.get(ks),bykf.get(ke)
    if not s or not e: continue
    t0,t1=I(s,"timestamp_ns"),I(e,"timestamp_ns")
    fs=[r for r in front if t0<=I(r,"timestamp_ns")<=t1 and I(r,"mono_pose_valid")==1]
    for r in fs:
        x,y=F(r,"mono_body_tx"),F(r,"mono_body_ty")
        n=math.hypot(x,y)
        if n>1e-9: ab_vecs.append((x/n,y/n))
if not ab_vecs:
    raise SystemExit("no valid A->B mono frontend poses")

ux,uy=mean([v[0] for v in ab_vecs]),mean([v[1] for v in ab_vecs])
un=math.hypot(ux,uy)
ux,uy=ux/un,uy/un

print("================ V25 MONO FRONTEND DIRECTION vs BACKEND Z ================")
print("run:",root)
print(f"common mono A->B horizontal axis = [{ux:+.4f},{uy:+.4f}]")
print("mono translation is direction-only (arbitrary monocular scale); angles/ratios are meaningful, millimetres are not.")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s,e=bykf.get(ks),bykf.get(ke)
    if not s or not e: continue
    t0,t1=I(s,"timestamp_ns"),I(e,"timestamp_ns")
    fs=[r for r in front if t0<=I(r,"timestamp_ns")<=t1 and I(r,"mono_pose_valid")==1]
    slopes=[]; angles=[]; alongs=[]; zn=[]
    for r in fs:
        x,y,z=F(r,"mono_body_tx"),F(r,"mono_body_ty"),F(r,"mono_body_tz")
        norm=F(r,"mono_body_t_norm")
        along=x*ux+y*uy
        if abs(along)>1e-6:
            slopes.append(z/along)
            angles.append(math.degrees(math.atan2(z,along)))
        if norm>1e-9: zn.append(z/norm)
        alongs.append(along)
    dz=F(L,"dz_m")*1000
    scale=F(L,"scale_horizontal")
    print(f"LEG {leg} {L['direction']}: backend scale={scale:.4f} backend dz={dz:+.1f}mm monoValid={len(fs)}")
    if fs:
        print(f"  mono along median={median(alongs):+.4f}  z/norm median={median(zn):+.4f}")
        print(f"  mono z/along median={median(slopes):+.4f}  mean={mean(slopes):+.4f}")
        print(f"  mono effective angle median={median(angles):+.2f}deg mean={mean(angles):+.2f}deg")
    print()

print("INTERPRETATION:")
print("- If mono effective angle already flips/signs like backend dz for A->B vs B->A, the wrong vertical direction is present before backend fusion.")
print("- If mono angle stays near zero/symmetric while backend dz is directional, the error is introduced mainly after frontend geometry, in fusion/backend.")
print("- Because monocular translation scale is arbitrary, do not compare mono translation magnitude with 500 mm or TF-Luna.")
