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
    raise SystemExit("usage: analyze_v25_pim_vs_backend_z.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
back=load(root/"jtzero_500mm_v25_backend.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")

need={"pim_valid","pim_dt_s","pim_dpx","pim_dpy","pim_dpz","pim_dvx","pim_dvy","pim_dvz"}
if not front or not need.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks PIM diagnostics; collect a new V25 run with updated logger")

back=sorted(back,key=lambda r:I(r,"timestamp_ns"))
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

print("================ V25 PIM vs BACKEND Z ================")
print("run:",root)
print("PIM prediction uses previous backend pose/velocity + gravity + PIM deltaP/deltaV.")
print("This is the inertial-only one-step prediction before visual/backend correction.")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    bseg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    if len(bseg)<2:
        print(f"LEG {leg}: insufficient backend states"); continue
    t0,t1=I(bseg[0],"timestamp_ns"),I(bseg[-1],"timestamp_ns")
    fs=[r for r in front if t0<I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1 and I(r,"pim_valid")==1]

    rows=[]
    for f in fs:
        ts=I(f,"timestamp_ns")
        bp=prev_backend(ts)
        bc=nearest_backend(ts)
        if bp is None or bc is None: continue
        dt=F(f,"pim_dt_s")
        if dt<=0 or dt>1.0: continue
        # Require frontend timestamp to line up with current backend keyframe reasonably well.
        match_ms=abs(I(bc,"timestamp_ns")-ts)/1e6
        if match_ms>80.0: continue

        R=RzRyRx(math.radians(F(bp,"roll_deg")),
                 math.radians(F(bp,"pitch_deg")),
                 math.radians(F(bp,"yaw_deg")))
        dp_b=(F(f,"pim_dpx"),F(f,"pim_dpy"),F(f,"pim_dpz"))
        dv_b=(F(f,"pim_dvx"),F(f,"pim_dvy"),F(f,"pim_dvz"))
        dp_w=mv(R,dp_b)
        dv_w=mv(R,dv_b)

        pred_dz=F(bp,"vz_m_s")*dt + 0.5*GZ*dt*dt + dp_w[2]
        pred_dvz=GZ*dt + dv_w[2]
        actual_dz=F(bc,"pz_m")-F(bp,"pz_m")
        actual_dvz=F(bc,"vz_m_s")-F(bp,"vz_m_s")
        corr_dz=actual_dz-pred_dz
        corr_dvz=actual_dvz-pred_dvz
        rows.append((I(bc,"keyframe"),dt,pred_dz,actual_dz,corr_dz,pred_dvz,actual_dvz,corr_dvz,match_ms))

    print(f"LEG {leg} {L['direction']}: endpoint backend dz={F(L,'dz_m')*1000:+.1f} mm PIMsteps={len(rows)}")
    if rows:
        print(f"  inertial predicted step dz mean/median = {mean([x[2] for x in rows])*1000:+.2f} / {median([x[2] for x in rows])*1000:+.2f} mm")
        print(f"  backend actual step dz mean/median     = {mean([x[3] for x in rows])*1000:+.2f} / {median([x[3] for x in rows])*1000:+.2f} mm")
        print(f"  backend correction dz mean/median      = {mean([x[4] for x in rows])*1000:+.2f} / {median([x[4] for x in rows])*1000:+.2f} mm")
        print(f"  inertial predicted dVz mean            = {mean([x[5] for x in rows]):+.4f} m/s")
        print(f"  backend correction dVz mean            = {mean([x[7] for x in rows]):+.4f} m/s")
        print(f"  timestamp match mean                    = {mean([x[8] for x in rows]):.2f} ms")
        print("  largest |inertial predicted dz| steps:")
        for kf,dt,pdz,adz,cdz,pdv,adv,cdv,mm in sorted(rows,key=lambda x:abs(x[2]),reverse=True)[:5]:
            print(f"    KF={kf:3d} dt={dt:.3f}s PIMpred={pdz*1000:+.1f}mm backend={adz*1000:+.1f}mm correction={cdz*1000:+.1f}mm")
    print()

print("INTERPRETATION:")
print("- PIM prediction already negative on A->B and positive on B->A => directional Z is present in inertial prediction before visual correction.")
print("- PIM prediction has no directional sign, but backend correction creates it => fusion/visual correction is the main source.")
print("- If both contribute with the same sign, the defect is shared: inertial prediction starts it and backend fails to remove or amplifies it.")
print("- Do not use one-step PIM magnitudes to calibrate parameters until timestamp matching and frame conventions are validated.")
