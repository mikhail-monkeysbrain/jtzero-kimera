#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

GZ=-9.81
ONSET_XY_M=0.005

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r)
    cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return (
      (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-sy*sr + sy*sr - sy*sr),
      (-sp,   cp*sr,          cp*cr)
    )

# Correct explicit matrix; kept separate to avoid accidental algebra edits.
def R_body_to_world(r,p,y):
    cr,sr=math.cos(r),math.sin(r)
    cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return (
      (cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
      (sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
      (-sp,   cp*sr,            cp*cr)
    )

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_pim_vertical_axis_contributions.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
back=sorted(load(root/"jtzero_500mm_v25_backend.csv"), key=lambda r:I(r,"timestamp_ns"))
legs=load(root/"jtzero_500mm_v25_legs.csv")

need={"pim_valid","pim_dt_s","pim_dvx","pim_dvy","pim_dvz"}
if not front or not need.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks PIM diagnostics")

bts=[I(r,"timestamp_ns") for r in back]
bykf={I(r,"keyframe"):r for r in back}

def nearest_backend(ts):
    j=bisect.bisect_left(bts,ts)
    cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(back): cand.append(back[k])
    return min(cand,key=lambda r:abs(I(r,"timestamp_ns")-ts)) if cand else None

def prev_backend(ts):
    j=bisect.bisect_left(bts,ts)-1
    return back[j] if j>=0 else None

print("================ V25 PIM VERTICAL AXIS CONTRIBUTIONS ================")
print("run:",root)
print("World-Z PIM deltaV = R20*dVx + R21*dVy + R22*dVz.")
print("Then gravity*dt is added. We inspect the first motion keyframes only.")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s=bykf.get(ks)
    if not s: continue

    # Motion onset from backend XY displacement, for timeline alignment only.
    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    onset_kf=ks
    for r in seg:
        dx=F(r,"px_m")-F(s,"px_m")
        dy=F(r,"py_m")-F(s,"py_m")
        if math.hypot(dx,dy)>=ONSET_XY_M:
            onset_kf=I(r,"keyframe")
            break

    t0=I(bykf[onset_kf],"timestamp_ns")
    t1=I(bykf[ke],"timestamp_ns")
    fs=[r for r in front if t0<=I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1 and I(r,"pim_valid")==1]

    rows=[]
    for f in fs:
        ts=I(f,"timestamp_ns")
        bp=prev_backend(ts); bc=nearest_backend(ts)
        if bp is None or bc is None: continue
        if abs(I(bc,"timestamp_ns")-ts)/1e6>80.0: continue
        dt=F(f,"pim_dt_s")
        if dt<=0 or dt>1.0: continue

        r=math.radians(F(bp,"roll_deg"))
        p=math.radians(F(bp,"pitch_deg"))
        y=math.radians(F(bp,"yaw_deg"))
        R=R_body_to_world(r,p,y)
        dvx,dvy,dvz=F(f,"pim_dvx"),F(f,"pim_dvy"),F(f,"pim_dvz")

        cx=R[2][0]*dvx
        cy=R[2][1]*dvy
        cz=R[2][2]*dvz
        grav=GZ*dt
        total=grav+cx+cy+cz

        rows.append({
            "kf":I(bc,"keyframe"),"dt":dt,
            "r20":R[2][0],"r21":R[2][1],"r22":R[2][2],
            "dvx":dvx,"dvy":dvy,"dvz":dvz,
            "cx":cx,"cy":cy,"cz":cz,"grav":grav,"total":total,
            "actual_dvz":F(bc,"vz_m_s")-F(bp,"vz_m_s"),
        })

    first=rows[:min(8,len(rows))]
    print(f"LEG {leg} {L['direction']}: endpoint dz={F(L,'dz_m')*1000:+.1f}mm onsetKF={onset_kf} steps={len(rows)}")
    if not first:
        print("  no matched rows")
        print()
        continue

    print("  first motion steps:")
    for q in first:
        print(
          f"    KF={q['kf']:3d} R20={q['r20']:+.4f} R21={q['r21']:+.4f} "
          f"| X->{q['cx']*1000:+6.1f} Y->{q['cy']*1000:+6.1f} Z->{q['cz']*1000:+7.1f} "
          f"g->{q['grav']*1000:+7.1f} => PIMdVz={q['total']*1000:+6.1f} mm/s"
        )

    sx=sum(q["cx"] for q in first)
    sy=sum(q["cy"] for q in first)
    sz=sum(q["cz"]+q["grav"] for q in first)
    st=sum(q["total"] for q in first)
    print(f"  first-{len(first)} cumulative contributions:")
    print(f"    horizontal X leakage = {sx*1000:+.2f} mm/s")
    print(f"    horizontal Y leakage = {sy*1000:+.2f} mm/s")
    print(f"    native Z + gravity   = {sz*1000:+.2f} mm/s")
    print(f"    total PIM dVz        = {st*1000:+.2f} mm/s")

    horiz=sx+sy
    if abs(st)>1e-9:
        print(f"    horizontal leakage share of total = {100.0*horiz/st:+.1f}%")
    print()

print("INTERPRETATION:")
print("- If X/Y leakage changes sign with A->B/B->A and dominates total PIM dVz, the frame/orientation projection is the main mechanism.")
print("- If native Z+gravity changes sign while X/Y leakage stays small, the problem is mainly in accelerometer Z/bias/preintegration rather than horizontal leakage.")
print("- If both contribute, quantify which term dominates before changing T_BS or IMU noise parameters.")
