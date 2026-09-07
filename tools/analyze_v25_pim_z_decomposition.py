#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

GZ=-9.81

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def median(xs):
    if not xs: return float("nan")
    s=sorted(xs); n=len(s)
    return s[n//2] if n%2 else 0.5*(s[n//2-1]+s[n//2])
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r)
    cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return (
      (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
      (-sp,   cp*sr,          cp*cr)
    )
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_pim_z_decomposition.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
back=sorted(load(root/"jtzero_500mm_v25_backend.csv"), key=lambda r:I(r,"timestamp_ns"))
legs=load(root/"jtzero_500mm_v25_legs.csv")

need={"pim_valid","pim_dt_s","pim_dpx","pim_dpy","pim_dpz","pim_dvx","pim_dvy","pim_dvz"}
if not front or not need.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks PIM diagnostics")

bts=[I(r,"timestamp_ns") for r in back]
def nearest_backend(ts):
    j=bisect.bisect_left(bts,ts)
    cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(back): cand.append(back[k])
    return min(cand,key=lambda r:abs(I(r,"timestamp_ns")-ts)) if cand else None
def prev_backend(ts):
    j=bisect.bisect_left(bts,ts)-1
    return back[j] if j>=0 else None

print("================ V25 PIM Z DECOMPOSITION ================")
print("run:",root)
print("predicted dz = previous_vz*dt + 0.5*g*dt^2 + rotated_PIM_deltaP_z")
print("predicted dVz = g*dt + rotated_PIM_deltaV_z")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    bseg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    if len(bseg)<2: continue
    t0,t1=I(bseg[0],"timestamp_ns"),I(bseg[-1],"timestamp_ns")
    fs=[r for r in front if t0<I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1 and I(r,"pim_valid")==1]

    rows=[]
    for f in fs:
        ts=I(f,"timestamp_ns")
        bp=prev_backend(ts); bc=nearest_backend(ts)
        if bp is None or bc is None: continue
        if abs(I(bc,"timestamp_ns")-ts)/1e6>80.0: continue
        dt=F(f,"pim_dt_s")
        if dt<=0 or dt>1.0: continue

        R=RzRyRx(math.radians(F(bp,"roll_deg")),
                 math.radians(F(bp,"pitch_deg")),
                 math.radians(F(bp,"yaw_deg")))
        dpw=mv(R,(F(f,"pim_dpx"),F(f,"pim_dpy"),F(f,"pim_dpz")))
        dvw=mv(R,(F(f,"pim_dvx"),F(f,"pim_dvy"),F(f,"pim_dvz")))

        term_v=F(bp,"vz_m_s")*dt
        term_g=0.5*GZ*dt*dt
        term_pim=dpw[2]
        pred=term_v+term_g+term_pim

        term_gv=GZ*dt
        term_pimv=dvw[2]
        pred_dv=term_gv+term_pimv

        actual=F(bc,"pz_m")-F(bp,"pz_m")
        rows.append((I(bc,"keyframe"),dt,term_v,term_g,term_pim,pred,actual,term_gv,term_pimv,pred_dv))

    print(f"LEG {leg} {L['direction']}: endpoint backend dz={F(L,'dz_m')*1000:+.1f} mm steps={len(rows)}")
    if rows:
        print(f"  prev-vz contribution mean/median = {mean([x[2] for x in rows])*1000:+.2f} / {median([x[2] for x in rows])*1000:+.2f} mm")
        print(f"  gravity contribution mean        = {mean([x[3] for x in rows])*1000:+.2f} mm")
        print(f"  rotated PIM deltaPz mean/median  = {mean([x[4] for x in rows])*1000:+.2f} / {median([x[4] for x in rows])*1000:+.2f} mm")
        print(f"  total inertial predicted dz mean = {mean([x[5] for x in rows])*1000:+.2f} mm")
        print(f"  backend actual step dz mean      = {mean([x[6] for x in rows])*1000:+.2f} mm")
        print(f"  gravity dVz mean                 = {mean([x[7] for x in rows]):+.4f} m/s")
        print(f"  rotated PIM deltaVz mean         = {mean([x[8] for x in rows]):+.4f} m/s")
        print(f"  total inertial predicted dVz     = {mean([x[9] for x in rows]):+.4f} m/s")
    print()

print("INTERPRETATION:")
print("- If rotated PIM deltaPz/deltaVz changes enough with direction to create the sign flip, the problem is already inside IMU preintegration / its frame or bias inputs.")
print("- If PIM terms are similar but previous-vz contribution flips, the directional error is being carried forward from the prior backend state.")
print("- Gravity contribution should be nearly identical for similar dt; a directional difference there would indicate a formula/frame mistake in this diagnostic.")
