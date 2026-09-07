#!/usr/bin/env python3
import csv, math
from pathlib import Path

H=Path("/home/vio")
LEGS=H/"jtzero_500mm_v25_legs.csv"
BACK=H/"jtzero_500mm_v25_backend.csv"
OUT=H/"jtzero_v25_bias_projection.csv"

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def norm(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def RzRyRx(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return (
      (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
      (-sp,   cp*sr,          cp*cr),
    )

def matvec(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

legs=load(LEGS); be=load(BACK)
bykf={I(r,"keyframe"):r for r in be}
rows=[]

print("================ V25 ACCEL-BIAS PROJECTION BY LEG ================")
for L in legs:
    leg=I(L,"leg")
    ks=I(L,"start_settled_kf")
    ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        print(f"LEG{leg}: missing backend state")
        continue

    d=(F(e,"px_m")-F(s,"px_m"), F(e,"py_m")-F(s,"py_m"), 0.0)
    dn=norm(d)
    if dn < 1e-9:
        print(f"LEG{leg}: zero horizontal displacement")
        continue
    u=(d[0]/dn,d[1]/dn,0.0)
    ucross=(-u[1],u[0],0.0)

    seg=[r for r in be if ks <= I(r,"keyframe") <= ke]
    along=[]; cross=[]; vertical=[]; banorm=[]
    for r in seg:
        ba_body=(F(r,"bax"),F(r,"bay"),F(r,"baz"))
        rr=math.radians(F(r,"roll_deg"))
        pp=math.radians(F(r,"pitch_deg"))
        yy=math.radians(F(r,"yaw_deg"))
        ba_world=matvec(RzRyRx(rr,pp,yy),ba_body)
        along.append(dot(ba_world,u))
        cross.append(dot(ba_world,ucross))
        vertical.append(ba_world[2])
        banorm.append(norm(ba_body))

    dur=(I(e,"timestamp_ns")-I(s,"timestamp_ns"))*1e-9
    xy=F(L,"horizontal_m")*1000.0
    scale=xy/500.0
    ma=mean(along); mc=mean(cross); mz=mean(vertical); mb=mean(banorm)

    # If a constant acceleration bias along the leg were left uncompensated in
    # position integration, 0.5*a*t^2 gives its rough position-error scale.
    # This is only a scale indicator, not a causal prediction of the smoother.
    equiv=0.5*ma*dur*dur*1000.0

    row=dict(
      leg=leg,direction=L["direction"],duration_s=dur,xy_mm=xy,scale=scale,
      mean_ba_world_along_m_s2=ma,mean_ba_world_cross_m_s2=mc,
      mean_ba_world_vertical_m_s2=mz,mean_ba_norm_m_s2=mb,
      equiv_const_bias_position_mm=equiv,
      start_bax=F(s,"bax"),start_bay=F(s,"bay"),start_baz=F(s,"baz"),
      end_bax=F(e,"bax"),end_bay=F(e,"bay"),end_baz=F(e,"baz"),
    )
    rows.append(row)

    print(f"LEG {leg} {L['direction']}: XY={xy:.2f}mm scale={scale:.4f} dur={dur:.2f}s")
    print(f"  mean BA_world along/cross/z = [{ma:+.5f},{mc:+.5f},{mz:+.5f}] m/s^2  mean|BA|={mb:.5f}")
    print(f"  rough 0.5*a_along*t^2 scale = {equiv:+.1f} mm")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    print(
      f"{d}: scale={mean([r['scale'] for r in rr]):.4f} "
      f"BA_along={mean([r['mean_ba_world_along_m_s2'] for r in rr]):+.5f} "
      f"BA_cross={mean([r['mean_ba_world_cross_m_s2'] for r in rr]):+.5f} "
      f"BA_z={mean([r['mean_ba_world_vertical_m_s2'] for r in rr]):+.5f}"
    )

if rows:
    with OUT.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

print("\nINTERPRETATION:")
print("- Similar |BA| but opposite along-leg projection can produce direction-dependent inertial effects.")
print("- If B->A consistently has a larger along-leg BA projection than A->B, Hypothesis 1B becomes directional rather than merely 'large bias'.")
print("- If projection does not track the ~6% B->A scale excess, BA is probably secondary and LOW_DISPARITY/mono-scale/endpoint dynamics should move ahead.")
print("Saved:",OUT)
