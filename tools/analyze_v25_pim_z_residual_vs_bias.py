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
    raise SystemExit("usage: analyze_v25_pim_z_residual_vs_bias.py RUN_DIR")

root=Path(sys.argv[1])
front=load(root/"jtzero_500mm_v25_frontend.csv")
back=sorted(load(root/"jtzero_500mm_v25_backend.csv"), key=lambda r:I(r,"timestamp_ns"))
legs=load(root/"jtzero_500mm_v25_legs.csv")

need_f={"pim_valid","pim_dt_s","pim_dvx","pim_dvy","pim_dvz"}
need_b={"bax","bay","baz","roll_deg","pitch_deg","yaw_deg","vx_m_s","vy_m_s","vz_m_s"}
if not front or not need_f.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks PIM diagnostics")
if not back or not need_b.issubset(back[0].keys()):
    raise SystemExit("backend CSV lacks bias/orientation diagnostics")

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

print("================ V25 PIM Z RESIDUAL vs ACCEL BIAS ================")
print("run:",root)
print("native-Z residual acceleration ~= (R22*dVz + g*dt)/dt")
print("We compare that residual with backend accelerometer bias bz during first motion keyframes.")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s=bykf.get(ks)
    if not s: continue

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

        R=R_body_to_world(math.radians(F(bp,"roll_deg")),
                          math.radians(F(bp,"pitch_deg")),
                          math.radians(F(bp,"yaw_deg")))
        dvz=F(f,"pim_dvz")
        native_z_plus_g = R[2][2]*dvz + GZ*dt
        native_acc_res = native_z_plus_g/dt
        rows.append({
            "kf":I(bc,"keyframe"),
            "native_acc_res":native_acc_res,
            "baz_prev":F(bp,"baz"),
            "baz_now":F(bc,"baz"),
            "dbaz":F(bc,"baz")-F(bp,"baz"),
            "vz_prev":F(bp,"vz_m_s"),
            "vz_now":F(bc,"vz_m_s"),
        })

    first=rows[:min(8,len(rows))]
    print(f"LEG {leg} {L['direction']}: endpoint dz={F(L,'dz_m')*1000:+.1f}mm onsetKF={onset_kf} samples={len(first)}")
    if not first:
        print("  no matched rows")
        print()
        continue
    print(f"  onset backend baz       = {first[0]['baz_prev']:+.5f} m/s^2")
    print(f"  first-{len(first)} mean baz       = {mean([r['baz_prev'] for r in first]):+.5f} m/s^2")
    print(f"  first-{len(first)} total baz change= {sum(r['dbaz'] for r in first):+.5f} m/s^2")
    print(f"  first-{len(first)} mean native-Z residual accel = {mean([r['native_acc_res'] for r in first]):+.5f} m/s^2")
    print("  timeline:")
    for r in first:
        print(f"    KF={r['kf']:3d} nativeZres={r['native_acc_res']:+.4f}  baz={r['baz_prev']:+.4f}->{r['baz_now']:+.4f}  Vz={r['vz_prev']*1000:+.1f}->{r['vz_now']*1000:+.1f}mm/s")
    print()

print("INTERPRETATION:")
print("- If baz is nearly unchanged while native-Z residual flips with direction, the directional error is not being created by rapid bias-state changes.")
print("- If baz itself shifts by comparable magnitude/sign before or during each leg, accelerometer-bias estimation becomes a strong suspect.")
print("- A stable baz does not prove the sensor is correct: a direction-dependent raw Z/body signal can still pass through preintegration.")
