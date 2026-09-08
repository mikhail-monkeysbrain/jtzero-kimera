#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(x):return statistics.mean(x) if x else float("nan")
def norm(v):return math.sqrt(sum(x*x for x in v))
def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v):return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))
def nearest(rows,ts,key):
    return min(rows,key=lambda r:abs(I(r,key)-ts)) if rows else None
def central(rows):
    if len(rows)<8:return rows
    k=len(rows)//4
    return rows[k:len(rows)-k]
def solve_roll_delta(acc_flu, fc_roll_deg, fc_pitch_deg, fc_yaw_deg):
    target=norm(acc_flu)
    best=(1e9,0.0,0.0)
    # Search +/-3 deg, 0.001 deg resolution.
    for i in range(-3000,3001):
        d=i*0.001
        rr=math.radians(fc_roll_deg+d)
        pp=math.radians(-fc_pitch_deg)
        yy=math.radians(-fc_yaw_deg)
        wz=mv(RzRyRx(rr,pp,yy),acc_flu)[2]
        err=abs(wz-target)
        if err<best[0]:
            best=(err,d,wz)
    return best[1],best[2],best[0]

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_effective_roll_correction.py RUN [RUN ...]")

print("================ V25 EFFECTIVE ROLL CORRECTION ================")
print("For each settled endpoint, finds the small extra roll angle that makes the measured stationary acceleration vector vertical in world coordinates.")
print("Uses the measured acceleration magnitude as the target, so this tests orientation mismatch rather than absolute accelerometer scale.")
print("A single constant correction across A/B would support fixed frame/calibration misalignment.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    att=load(root/"jtzero_500mm_v25_attitude.csv")
    back=load(root/"jtzero_500mm_v25_backend.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    bykf={I(r,"keyframe"):r for r in back}

    print("\nRUN:",root)
    for L in legs:
        ks,ke=I(L,"start_settled_kf"),I(L,"end_settled_kf")
        s,e=bykf.get(ks),bykf.get(ke)
        if not s or not e:continue
        w=int(.6e9)
        states=[]
        for label,b in (("START",s),("END",e)):
            q=central([r for r in imu if abs(I(r,"mapped_ns")-I(b,"timestamp_ns"))<=w])
            if not q:continue
            acc=(mean([F(r,"ax") for r in q]),-mean([F(r,"ay") for r in q]),-mean([F(r,"az") for r in q]))
            a=nearest(att,I(b,"callback_wall_ns"),"recv_ns")
            if not a:continue
            fr,fp,fy=F(a,"roll_deg"),F(a,"pitch_deg"),F(a,"yaw_deg")
            d,wz,err=solve_roll_delta(acc,fr,fp,fy)
            states.append((label,d,fr,fp,fy,acc,wz,err))
            print(f"LEG {I(L,'leg')} {L['direction']} {label}:")
            print(f"  FC R/P/Y=[{fr:+.4f},{fp:+.4f},{fy:+.4f}] deg")
            print(f"  ACC FLU=[{acc[0]:+.5f},{acc[1]:+.5f},{acc[2]:+.5f}] |a|={norm(acc):.5f}")
            print(f"  effective roll correction={d:+.4f} deg  verticalization residual={err:.7f} m/s^2")
        if len(states)==2:
            d0,d1=states[0][1],states[1][1]
            print(f"  END-START correction change = {d1-d0:+.4f} deg")
            print(f"  backend dz={F(L,'dz_m')*1000:+.1f}mm")

print("\nINTERPRETATION:")
print("- Nearly identical correction at START and END supports one fixed roll/frame calibration offset.")
print("- Direction/position-dependent correction change means one constant extrinsic/roll correction cannot explain both states.")
print("- This is a diagnostic orientation consistency test, not a proposed estimator correction.")
