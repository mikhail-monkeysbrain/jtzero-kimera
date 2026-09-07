#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(Path("/home/vio/jtzero_p11_latest_imu_path_run.txt").read_text().strip())
rows = list(csv.DictReader((root / "p11_imu_path.csv").open()))
types = ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "HIGHRES_IMU")
stages = ("A1", "B1", "A2", "B2", "A3", "B3", "A4")

def mean_vec(rr):
    return tuple(statistics.mean(float(r[k]) for r in rr) for k in ("ax_si", "ay_si", "az_si"))

def norm(v):
    return math.sqrt(sum(x*x for x in v))

def sub(a,b):
    return tuple(x-y for x,y in zip(a,b))

def dot(a,b):
    return sum(x*y for x,y in zip(a,b))

def scale(v,s):
    return tuple(x*s for x in v)

def angle_deg(a,b):
    na,nb=norm(a),norm(b)
    if na == 0 or nb == 0: return float("nan")
    c=max(-1.0,min(1.0,dot(a,b)/(na*nb)))
    return math.degrees(math.acos(c))

print("================ P11 IMU VECTOR A/B ================")
print("run:", root)
print("Uses only recording=1 and the last 5 s of each plateau, matching analyze_p11_imu_path_ab.py.")

for typ in types:
    rr=[r for r in rows if r["type"]==typ and r["recording"]=="1"]
    if not rr:
        print(f"\n[{typ}] НЕТ ДАННЫХ")
        continue

    vecs=[]
    print(f"\n[{typ}]")
    for lab in stages:
        x=[r for r in rr if r["stage"]==lab]
        if not x: continue
        tm=max(int(r["recv_ns"]) for r in x)
        x=[r for r in x if int(r["recv_ns"])>=tm-5_000_000_000]
        v=mean_vec(x)
        vecs.append((lab,lab[0],v))
        print(f"{lab}: n={len(x):4d} a=[{v[0]:+.6f},{v[1]:+.6f},{v[2]:+.6f}] |mean a|={norm(v):.6f}")

    av=[v for _,p,v in vecs if p=="A"]
    bv=[v for _,p,v in vecs if p=="B"]
    if not av or not bv: continue
    A=tuple(statistics.mean(v[i] for v in av) for i in range(3))
    B=tuple(statistics.mean(v[i] for v in bv) for i in range(3))
    d=sub(B,A)
    ghat=scale(A,1.0/norm(A))
    dpar=scale(ghat,dot(d,ghat))
    dperp=sub(d,dpar)

    print(f"A mean vector = [{A[0]:+.6f},{A[1]:+.6f},{A[2]:+.6f}]  |A|={norm(A):.6f}")
    print(f"B mean vector = [{B[0]:+.6f},{B[1]:+.6f},{B[2]:+.6f}]  |B|={norm(B):.6f}")
    print(f"B-A vector    = [{d[0]:+.6f},{d[1]:+.6f},{d[2]:+.6f}]  |d|={norm(d):.6f} m/s^2")
    print(f"angle(A,B)    = {angle_deg(A,B):.5f} deg")
    print(f"parallel-to-A = {dot(d,ghat):+.6f} m/s^2  |parallel|={norm(dpar):.6f}")
    print(f"perpendicular = [{dperp[0]:+.6f},{dperp[1]:+.6f},{dperp[2]:+.6f}]  |perp|={norm(dperp):.6f}")
    if norm(d)>0:
        print(f"decomposition = parallel {100*norm(dpar)/norm(d):.1f}% / perpendicular {100*norm(dperp)/norm(d):.1f}% of |B-A vector|")

print("\nINTERPRETATION:")
print("- Mostly parallel B-A: measured acceleration magnitude/common-mode component changes; pure coordinate rotation is insufficient.")
print("- Mostly perpendicular B-A with nearly unchanged magnitude: orientation/projection change is a stronger explanation.")
print("- Similar decomposition in SCALED_IMU and SCALED_IMU2: common physical/FC factor is strengthened.")
print("- Different vector direction/decomposition between IMU instances: an IMU-instance-specific component remains plausible.")
print("- RAW_IMU here is still an FC/MAVLink measurement, not direct MEMS ADC data.")
