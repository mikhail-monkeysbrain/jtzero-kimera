#!/usr/bin/env python3
import csv, math
from pathlib import Path

HOME=Path("/home/vio")
IMU=HOME/"jtzero_500mm_v25.csv"
BACKEND=HOME/"jtzero_500mm_v25_backend.csv"
ATT=HOME/"jtzero_500mm_v25_attitude.csv"

def rows(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

imu=rows(IMU)
be=rows(BACKEND)
att=rows(ATT)

def f(r,k): return float(r[k])
def i(r,k): return int(float(r[k]))

# Use backend states before LEG1 START as the stationary analysis window.
pre=[r for r in be if r["leg"]=="0" and r["phase"]=="OUTSIDE"]
if not pre:
    raise SystemExit("No pre-LEG1 backend states found")
t0=min(i(r,"timestamp_ns") for r in pre)
t1=max(i(r,"timestamp_ns") for r in pre)

# Raw IMU CSV stores original FRD values in ax..gz columns.
imu_pre=[r for r in imu if r.get("type")=="IMU" and t0 <= i(r,"mapped_ns") <= t1]
if not imu_pre:
    raise SystemExit("No mapped IMU rows in pre-LEG1 window")

def mean(vals): return sum(vals)/len(vals)

ax=mean([ f(r,"ax") for r in imu_pre ])
ay=mean([ f(r,"ay") for r in imu_pre ])
az=mean([ f(r,"az") for r in imu_pre ])
# Convert raw MAVLink FRD -> FLU exactly as V25 does.
a_flu=(ax,-ay,-az)

# Average late stationary backend state (last 8 pre-leg states) to avoid startup transient.
late=pre[-8:] if len(pre)>=8 else pre
roll=math.radians(mean([f(r,"roll_deg") for r in late]))
pitch=math.radians(mean([f(r,"pitch_deg") for r in late]))
yaw=math.radians(mean([f(r,"yaw_deg") for r in late]))
ba=(mean([f(r,"bax") for r in late]), mean([f(r,"bay") for r in late]), mean([f(r,"baz") for r in late]))

# R = Rz(yaw)*Ry(pitch)*Rx(roll), body -> world.
cr,sr=math.cos(roll),math.sin(roll)
cp,sp=math.cos(pitch),math.sin(pitch)
cy,sy=math.cos(yaw),math.sin(yaw)
R=[
 [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
 [-sp,   cp*sr,          cp*cr]
]
gup=(0.0,0.0,9.81)
# expected specific force at rest without accel bias = R^T * (+g world)
pred=tuple(sum(R[j][k]*gup[j] for j in range(3)) for k in range(3))
res=tuple(a_flu[k]-pred[k] for k in range(3))
post=tuple(res[k]-ba[k] for k in range(3))

def norm(v): return math.sqrt(sum(x*x for x in v))
def tilt(v):
    # gravity-derived roll/pitch in FLU for +Z-up specific force
    r=math.degrees(math.atan2(v[1],v[2]))
    p=math.degrees(math.atan2(-v[0], math.sqrt(v[1]*v[1]+v[2]*v[2])))
    return r,p

raw_r,raw_p=tilt(a_flu)
pred_r,pred_p=tilt(pred)

print("================ V25 STATIC ACCEL/BIAS FORENSIC ================")
print(f"backend pre-LEG1 window: KF {pre[0]['keyframe']}..{pre[-1]['keyframe']}  states={len(pre)}")
print(f"raw IMU samples in mapped window: {len(imu_pre)}")
print()
print("RAW mean FLU accel [m/s^2]:")
print(f"  [{a_flu[0]:+.6f}, {a_flu[1]:+.6f}, {a_flu[2]:+.6f}]  |a|={norm(a_flu):.6f}")
print(f"  gravity-derived tilt: roll={raw_r:+.3f} deg pitch={raw_p:+.3f} deg")
print()
print("BACKEND late stationary mean:")
print(f"  RPY=[{math.degrees(roll):+.3f}, {math.degrees(pitch):+.3f}, {math.degrees(yaw):+.3f}] deg")
print(f"  BA =[{ba[0]:+.6f}, {ba[1]:+.6f}, {ba[2]:+.6f}] m/s^2  |BA|={norm(ba):.6f}")
print()
print("EXPECTED rest specific force from backend attitude, BA=0:")
print(f"  [{pred[0]:+.6f}, {pred[1]:+.6f}, {pred[2]:+.6f}]")
print(f"  implied tilt: roll={pred_r:+.3f} deg pitch={pred_p:+.3f} deg")
print()
print("RAW - EXPECTED (bias needed if attitude is correct):")
print(f"  [{res[0]:+.6f}, {res[1]:+.6f}, {res[2]:+.6f}]  |.|={norm(res):.6f}")
print()
print("(RAW - EXPECTED) - BACKEND_BA:")
print(f"  [{post[0]:+.6f}, {post[1]:+.6f}, {post[2]:+.6f}]  |.|={norm(post):.6f}")
print()
print("INTERPRETATION:")
print("  If RAW-EXPECTED approximately equals backend BA, the large BA is mainly")
print("  compensating a raw-accel vs backend-attitude mismatch, not random optimizer drift.")
print("  If they disagree strongly, investigate bias-state estimation / factor coupling.")
