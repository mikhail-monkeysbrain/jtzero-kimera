#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def median(xs):
    if not xs: return float("nan")
    s=sorted(xs); n=len(s)
    return s[n//2] if n%2 else 0.5*(s[n//2-1]+s[n//2])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_frontend_pose_normalized.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")
back=load(root/"jtzero_500mm_v25_backend.csv")
bykf={I(r,"keyframe"):r for r in back}

need={"mono_pose_valid","mono_body_tx","mono_body_ty","mono_body_tz","mono_body_t_norm"}
if not front or not need.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks mono pose diagnostics")

# Estimate a common physical A->B horizontal axis from backend endpoint vectors,
# not from noisy monocular frame-by-frame translations.
ab=[]
for L in legs:
    if L["direction"]=="A->B":
        x,y=F(L,"dx_m"),F(L,"dy_m")
        n=math.hypot(x,y)
        if n>1e-9: ab.append((x/n,y/n))
ux,uy=mean([v[0] for v in ab]),mean([v[1] for v in ab])
n=math.hypot(ux,uy); ux,uy=ux/n,uy/n

print("================ V25 NORMALIZED MONO FRONTEND DIRECTION ================")
print("run:",root)
print(f"physical A->B axis=[{ux:+.4f},{uy:+.4f}]")
print("Each B->A mono vector is sign-flipped before comparison, so all legs are expressed as physical A->B.")
print("Only direction is interpreted; monocular magnitude is arbitrary.")
print()

for L in legs:
    leg=I(L,"leg"); sign=1.0 if L["direction"]=="A->B" else -1.0
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s,e=bykf.get(ks),bykf.get(ke)
    if not s or not e: continue
    t0,t1=I(s,"timestamp_ns"),I(e,"timestamp_ns")
    fs=[r for r in front if t0<=I(r,"timestamp_ns")<=t1 and I(r,"mono_pose_valid")==1]
    good=[]
    for r in fs:
        x=sign*F(r,"mono_body_tx"); y=sign*F(r,"mono_body_ty"); z=sign*F(r,"mono_body_tz")
        norm=F(r,"mono_body_t_norm")
        if norm<=1e-9: continue
        along=x*ux+y*uy
        lateral=-x*uy+y*ux
        # Require the visual translation to point broadly along the commanded axis.
        # This rejects unstable 5-point directions nearly sideways/backwards.
        if along/norm < 0.50: continue
        elev=math.degrees(math.atan2(z, math.hypot(along,lateral)))
        good.append((along/norm,lateral/norm,z/norm,elev))
    print(f"LEG {leg} {L['direction']}: monoValid={len(fs)} usable={len(good)} backend dz={F(L,'dz_m')*1000:+.1f}mm")
    if good:
        print(f"  forward/norm median = {median([g[0] for g in good]):+.3f}")
        print(f"  lateral/norm median = {median([g[1] for g in good]):+.3f}")
        print(f"  vertical/norm median= {median([g[2] for g in good]):+.3f}")
        print(f"  visual elevation median = {median([g[3] for g in good]):+.2f} deg")
    print()

print("INTERPRETATION:")
print("- Similar normalized visual elevation in A->B and B->A means the 5-point frontend does not show the backend's direction-dependent sign flip.")
print("- Opposite normalized visual elevation signs between A->B and B->A would support a visual-geometry directional asymmetry.")
print("- A low usable/monoValid count means the 5-point translation direction is unstable and the leg is not strong evidence.")
