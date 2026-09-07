#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(v): return math.sqrt(dot(v,v))
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r)
    cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return (
        (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
        (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
        (-sp,   cp*sr,          cp*cr),
    )

def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv) > 2:
    raise SystemExit("usage: analyze_v25_bias_common_axis.py [run_dir]")

root = Path(sys.argv[1]) if len(sys.argv)==2 else Path("/home/vio")
legs = load(root/"jtzero_500mm_v25_legs.csv")
back = load(root/"jtzero_500mm_v25_backend.csv")
bykf = {I(r,"keyframe"): r for r in back}

# Build one fixed world-horizontal axis from all A->B legs.
ab_dirs=[]
for L in legs:
    if L["direction"] != "A->B":
        continue
    ks=I(L,"start_settled_kf"); ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        continue
    d=(F(e,"px_m")-F(s,"px_m"), F(e,"py_m")-F(s,"py_m"), 0.0)
    n=norm(d)
    if n>1e-9:
        ab_dirs.append((d[0]/n,d[1]/n,0.0))

if not ab_dirs:
    raise SystemExit("ERROR: no A->B direction available")

u0=[mean([u[i] for u in ab_dirs]) for i in range(3)]
n=norm(u0)
u=(u0[0]/n,u0[1]/n,0.0)
uc=(-u[1],u[0],0.0)

print("================ V25 BA ON ONE FIXED A->B WORLD AXIS ================")
print(f"run: {root}")
print(f"fixed A->B axis = [{u[0]:+.6f},{u[1]:+.6f},0]")

rows=[]
for L in legs:
    leg=I(L,"leg"); ks=I(L,"start_settled_kf"); ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        continue

    d=(F(e,"px_m")-F(s,"px_m"), F(e,"py_m")-F(s,"py_m"), 0.0)
    dn=norm(d)
    du=(d[0]/dn,d[1]/dn,0.0)
    direction_dot=dot(du,u)

    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    common=[]; cross=[]; body_norm=[]
    for r in seg:
        ba_body=(F(r,"bax"),F(r,"bay"),F(r,"baz"))
        R=RzRyRx(math.radians(F(r,"roll_deg")),
                 math.radians(F(r,"pitch_deg")),
                 math.radians(F(r,"yaw_deg")))
        bw=mv(R,ba_body)
        common.append(dot(bw,u))
        cross.append(dot(bw,uc))
        body_norm.append(norm(ba_body))

    xy=F(L,"horizontal_m")*1000.0
    print(f"\nLEG {leg} {L['direction']}: scale={xy/500.0:.4f} "
          f"motion_axis_dot={direction_dot:+.4f}")
    print(f"  BA fixed-axis start/mean/end = "
          f"{common[0]:+.5f} / {mean(common):+.5f} / {common[-1]:+.5f} m/s^2")
    print(f"  BA fixed-cross mean = {mean(cross):+.5f} m/s^2  mean|BA|={mean(body_norm):.5f}")

    rows.append(dict(
        leg=leg,direction=L["direction"],scale=xy/500.0,
        motion_axis_dot=direction_dot,
        ba_common_start=common[0],ba_common_mean=mean(common),ba_common_end=common[-1],
        ba_cross_mean=mean(cross),ba_norm_mean=mean(body_norm),
    ))

print("\n================ DIRECTION MEANS ON THE SAME AXIS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if rr:
        print(f"{d}: scale={mean([r['scale'] for r in rr]):.4f} "
              f"BA_common={mean([r['ba_common_mean'] for r in rr]):+.5f} "
              f"BA_cross={mean([r['ba_cross_mean'] for r in rr]):+.5f}")

print("\nINTERPRETATION:")
print("- This analyzer deliberately uses ONE fixed A->B world axis for all legs.")
print("- A persistent world-frame BA must NOT be called direction-dependent merely because")
print("  its projection onto each leg's own direction changes sign when the vehicle reverses.")
print("- If BA_common keeps the same sign across A->B and B->A, the earlier along-leg sign")
print("  flip was a projection artifact; investigate persistent/drifting BA instead.")
print("- If BA_common itself reverses sign with motion direction, true direction-dependent")
print("  estimator behavior remains supported.")
