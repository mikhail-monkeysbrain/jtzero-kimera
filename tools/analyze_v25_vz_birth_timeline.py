#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

GZ=-9.81
ONSET_XY_M=0.005
SIGNIFICANT_VZ=0.005  # 5 mm/s

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
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
      (-sp,   cp*sr,          cp*cr)
    )
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_vz_birth_timeline.py RUN_DIR")

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

print("================ V25 Vz BIRTH TIMELINE ================")
print("run:",root)
print("Goal: find the first keyframes where vertical velocity appears.")
print("PIM dVz = gravity*dt + rotated PIM deltaV_z.")
print("Backend correction dVz = actual backend dVz - PIM dVz.")
print()

for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s=bykf.get(ks)
    if not s:
        print(f"LEG {leg}: missing start backend state")
        continue

    # Find estimator-detected motion onset: first backend state >5 mm horizontally from settled start.
    candidates=[r for r in back if ks<=I(r,"keyframe")<=ke]
    onset_kf=ks
    for r in candidates:
        dx=F(r,"px_m")-F(s,"px_m"); dy=F(r,"py_m")-F(s,"py_m")
        if math.hypot(dx,dy)>=ONSET_XY_M:
            onset_kf=I(r,"keyframe")
            break

    t_start=I(bykf[onset_kf],"timestamp_ns")
    t_end=I(bykf[ke],"timestamp_ns")
    fs=[r for r in front
        if t_start<=I(r,"timestamp_ns")<=t_end
        and I(r,"is_keyframe")==1 and I(r,"pim_valid")==1]

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
        dvw=mv(R,(F(f,"pim_dvx"),F(f,"pim_dvy"),F(f,"pim_dvz")))
        pim_dvz=GZ*dt+dvw[2]
        actual_dvz=F(bc,"vz_m_s")-F(bp,"vz_m_s")
        corr_dvz=actual_dvz-pim_dvz

        rows.append({
            "kf":I(bc,"keyframe"),"dt":dt,
            "vz_prev":F(bp,"vz_m_s"),"vz_now":F(bc,"vz_m_s"),
            "pim_dvz":pim_dvz,"actual_dvz":actual_dvz,"corr_dvz":corr_dvz,
            "pz_now":F(bc,"pz_m"),"px_now":F(bc,"px_m"),"py_now":F(bc,"py_m"),
        })

    print(f"LEG {leg} {L['direction']}: endpoint dz={F(L,'dz_m')*1000:+.1f}mm onsetKF={onset_kf} startKF={ks}")
    if not rows:
        print("  no matched PIM/backend rows")
        print()
        continue

    print("  first 10 motion steps:")
    for r in rows[:10]:
        source="PIM" if abs(r["pim_dvz"])>=abs(r["corr_dvz"]) else "BACKEND"
        print(
            f"    KF={r['kf']:3d} Vz {r['vz_prev']*1000:+6.1f}->{r['vz_now']*1000:+6.1f} mm/s "
            f"| PIM dVz={r['pim_dvz']*1000:+6.1f} "
            f"corr={r['corr_dvz']*1000:+6.1f} mm/s dominant={source}"
        )

    first_sig=next((r for r in rows if abs(r["vz_now"])>=SIGNIFICANT_VZ),None)
    if first_sig:
        source="PIM" if abs(first_sig["pim_dvz"])>=abs(first_sig["corr_dvz"]) else "BACKEND"
        print(
            f"  FIRST |Vz|>=5mm/s: KF={first_sig['kf']} Vz={first_sig['vz_now']*1000:+.1f}mm/s "
            f"PIM dVz={first_sig['pim_dvz']*1000:+.1f} corr={first_sig['corr_dvz']*1000:+.1f} "
            f"dominant={source}"
        )
    else:
        print("  FIRST |Vz|>=5mm/s: not reached")

    first5=rows[:min(5,len(rows))]
    print(
        f"  first-5 cumulative dVz: PIM={sum(r['pim_dvz'] for r in first5)*1000:+.1f} "
        f"backend-correction={sum(r['corr_dvz'] for r in first5)*1000:+.1f} "
        f"actual={sum(r['actual_dvz'] for r in first5)*1000:+.1f} mm/s"
    )
    print()

print("INTERPRETATION:")
print("- If the first significant Vz is created mainly by PIM on every leg, inertial preintegration starts the error.")
print("- If the first significant Vz is created mainly by backend correction, the error starts in fusion/visual optimization.")
print("- If PIM starts a small error and backend correction repeatedly reinforces the same sign, both stages participate.")
print("- Motion onset is located from backend XY displacement only to align the timeline; it is not evidence about the physical input.")
