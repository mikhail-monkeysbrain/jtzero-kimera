#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path
from collections import defaultdict

if len(sys.argv) < 2:
    print("usage: analyze_v23_raw_backend_scale_coupling.py RUN [RUN ...]")
    sys.exit(2)

runs=[Path(x) for x in sys.argv[1:]]

def med(xs):
    return statistics.median(xs) if xs else float("nan")

def mean(xs):
    return statistics.mean(xs) if xs else float("nan")

def norm3(v):
    return math.sqrt(sum(x*x for x in v))

def vecmean(rows, keys):
    return tuple(mean([float(r[k]) for r in rows]) for k in keys)

def nearest_phase_window(backend, leg, phase):
    rr=[r for r in backend if int(r["leg"])==leg and r["phase"]==phase]
    if not rr:
        return None
    ts=[int(r["timestamp_ns"]) for r in rr]
    return min(ts),max(ts)

def imu_window(imu, t0, t1):
    return [r for r in imu if t0 <= int(r["mapped_ns"]) <= t1]

def backend_phase(backend, leg, phase):
    return [r for r in backend if int(r["leg"])==leg and r["phase"]==phase]

def endpoint(rr, first=True, n=3):
    if not rr:
        return None
    rr=sorted(rr,key=lambda r:int(r["timestamp_ns"]))
    use=rr[:n] if first else rr[-n:]
    keys=("px_m","py_m","pz_m","vx_m_s","vy_m_s","vz_m_s",
          "roll_deg","pitch_deg","yaw_deg","bax","bay","baz","bgx","bgy","bgz")
    return {k:mean([float(r[k]) for r in use]) for k in keys}

def load(run):
    main=run/"jtzero_500mm_v23.csv"
    back=run/"jtzero_500mm_v23_backend.csv"
    legs=run/"jtzero_500mm_v23_legs.csv"
    if not main.exists():
        # tolerate V21 naming only if explicitly passed
        cands=sorted(run.glob("jtzero_500mm_v*.csv"))
        cands=[p for p in cands if not any(s in p.name for s in ("_backend","_events","_frontend","_legs","_range","_camera","_attitude"))]
        if not cands: raise FileNotFoundError(f"no main IMU csv in {run}")
        main=cands[0]
    if not back.exists():
        cands=sorted(run.glob("*_backend.csv"))
        if not cands: raise FileNotFoundError(f"no backend csv in {run}")
        back=cands[0]
    if not legs.exists():
        cands=sorted(run.glob("*_legs.csv"))
        if not cands: raise FileNotFoundError(f"no legs csv in {run}")
        legs=cands[0]
    with main.open() as f: imu=list(csv.DictReader(f))
    with back.open() as f: backend=list(csv.DictReader(f))
    with legs.open() as f: legrows=list(csv.DictReader(f))
    return imu,backend,legrows,main,back

print("================ V23 RAW→BACKEND→SCALE COUPLING ================")
print("Coordinate-safe first pass: no body/world rotation assumptions are used.")
print("Raw IMU is aligned to backend phase timestamps via mapped_ns↔timestamp_ns.")
print("This does NOT reconstruct PIM and does NOT prove causality.")

alllegs=[]
for run in runs:
    imu,backend,legrows,main,back=load(run)
    print("\n============================================================")
    print("RUN:",run)
    print("IMU:",main.name,"rows=",len(imu)," BACKEND:",back.name,"rows=",len(backend))
    print("============================================================")

    byleg={int(r["leg"]):r for r in legrows}
    for leg in sorted(byleg):
        lr=byleg[leg]
        direction=lr["direction"]
        scale=float(lr["scale_horizontal"])
        dx=float(lr["dx_m"]); dy=float(lr["dy_m"]); dz=float(lr["dz_m"])

        ss=backend_phase(backend,leg,"SETTLE_START")
        mv=backend_phase(backend,leg,"MOVE")
        se=backend_phase(backend,leg,"SETTLE_END")
        e0=endpoint(ss,first=True)
        e1=endpoint(se,first=False)

        # Use complete settle phase windows. If too short, still report available data.
        w0=nearest_phase_window(backend,leg,"SETTLE_START")
        w1=nearest_phase_window(backend,leg,"SETTLE_END")
        i0=imu_window(imu,*w0) if w0 else []
        i1=imu_window(imu,*w1) if w1 else []

        a0=vecmean(i0,("ax","ay","az")) if i0 else (float("nan"),)*3
        a1=vecmean(i1,("ax","ay","az")) if i1 else (float("nan"),)*3
        g0=vecmean(i0,("gx","gy","gz")) if i0 else (float("nan"),)*3
        g1=vecmean(i1,("gx","gy","gz")) if i1 else (float("nan"),)*3

        da=tuple(a1[i]-a0[i] for i in range(3))
        dnorm=norm3(a1)-norm3(a0) if i0 and i1 else float("nan")

        if e0 and e1:
            datt=(e1["roll_deg"]-e0["roll_deg"],
                  e1["pitch_deg"]-e0["pitch_deg"],
                  e1["yaw_deg"]-e0["yaw_deg"])
            dba=(e1["bax"]-e0["bax"],e1["bay"]-e0["bay"],e1["baz"]-e0["baz"])
            dbg=(e1["bgx"]-e0["bgx"],e1["bgy"]-e0["bgy"],e1["bgz"]-e0["bgz"])
            dv=(e1["vx_m_s"]-e0["vx_m_s"],e1["vy_m_s"]-e0["vy_m_s"],e1["vz_m_s"]-e0["vz_m_s"])
        else:
            datt=dba=dbg=dv=(float("nan"),)*3

        print(f"\nLEG {leg} {direction}: scale={scale:.6f} dxyz=[{dx:+.4f},{dy:+.4f},{dz:+.4f}]")
        print(f"  samples settle-start/end IMU={len(i0)}/{len(i1)} backend={len(ss)}/{len(se)}")
        print(f"  raw a start=[{a0[0]:+.5f},{a0[1]:+.5f},{a0[2]:+.5f}] |a|={norm3(a0):.6f}")
        print(f"  raw a end  =[{a1[0]:+.5f},{a1[1]:+.5f},{a1[2]:+.5f}] |a|={norm3(a1):.6f}")
        print(f"  raw Δa     =[{da[0]:+.5f},{da[1]:+.5f},{da[2]:+.5f}] Δ|a|={dnorm:+.6f}")
        print(f"  backend ΔRPY=[{datt[0]:+.4f},{datt[1]:+.4f},{datt[2]:+.4f}] deg")
        print(f"  backend ΔBa =[{dba[0]:+.6f},{dba[1]:+.6f},{dba[2]:+.6f}] m/s²")
        print(f"  backend ΔBg =[{dbg[0]:+.6f},{dbg[1]:+.6f},{dbg[2]:+.6f}] rad/s")
        print(f"  backend ΔV  =[{dv[0]:+.5f},{dv[1]:+.5f},{dv[2]:+.5f}] m/s")

        alllegs.append(dict(run=run.name,leg=leg,direction=direction,scale=scale,
                            dx=dx,dy=dy,dz=dz,da=da,dnorm=dnorm,datt=datt,dba=dba,dbg=dbg,dv=dv,
                            a0=a0,a1=a1))

print("\n================ CROSS-RUN SAME-DIRECTION SUMMARY ================")
for direction in ("A->B","B->A"):
    rr=[x for x in alllegs if x["direction"]==direction]
    if not rr: continue
    print(f"\n{direction}: n_legs={len(rr)}")
    print("  scale:",", ".join(f'{x["scale"]:.4f}' for x in rr),
          f" median={med([x['scale'] for x in rr]):.4f}")
    print(f"  Δ|a| median={med([x['dnorm'] for x in rr]):+.6f} m/s²")
    for j,name in enumerate(("x","y","z")):
        print(f"  Δa{name} median={med([x['da'][j] for x in rr]):+.6f}  "
              f"ΔBa{name} median={med([x['dba'][j] for x in rr]):+.6f}")
    print(f"  Δroll/pitch median=[{med([x['datt'][0] for x in rr]):+.4f},"
          f"{med([x['datt'][1] for x in rr]):+.4f}] deg")
    print(f"  ΔVz median={med([x['dv'][2] for x in rr]):+.5f} m/s")

print("\n================ PAIRWISE SIGN CONSISTENCY ================")
def sign(v,eps=1e-12):
    return 1 if v>eps else (-1 if v<-eps else 0)

# This block intentionally does not call correlation significant for n<3 runs.
for metric,label in [
    (lambda x:x["dnorm"],"Δ|a|"),
    (lambda x:x["datt"][0],"Δroll"),
    (lambda x:x["datt"][1],"Δpitch"),
    (lambda x:x["dba"][0],"ΔBax"),
    (lambda x:x["dba"][1],"ΔBay"),
    (lambda x:x["dba"][2],"ΔBaz"),
    (lambda x:x["dv"][2],"ΔVz"),
]:
    vals=[(x["direction"],x["scale"],metric(x)) for x in alllegs]
    under=[v for d,s,v in vals if s<1.0]
    over=[v for d,s,v in vals if s>1.0]
    if under and over:
        print(f"{label}: median(scale<1)={med(under):+.6f}  median(scale>1)={med(over):+.6f}")

print("\n================ INTERPRETATION RULES ================")
print("- First trust only coordinate-safe observations above: raw Δa, backend ΔRPY/ΔBa/ΔV, and scale.")
print("- Do NOT project raw body acceleration onto VIO world-motion direction yet; frame convention must be verified from source.")
print("- A useful mechanism candidate should reproduce sign/order across same-direction legs and across A-FIRST/B-FIRST.")
print("- n_legs here are not independent sensor-position experiments; do not treat simple correlations as statistical proof.")
print("- If a repeatable coupling appears, next step is source-verified frame mapping and quantitative gravity/bias projection.")
