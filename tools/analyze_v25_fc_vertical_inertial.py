#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def R_body_to_ned(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return (
      (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
      (-sp,   cp*sr,          cp*cr)
    )
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_fc_vertical_inertial.py RUN_DIR")

root=Path(sys.argv[1])
imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
att=load(root/"jtzero_500mm_v25_attitude.csv")
events=load(root/"jtzero_500mm_v25_events.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")
ev={(I(r,"leg"),r["event"]):r for r in events}
att.sort(key=lambda r:I(r,"recv_ns"))
ats=[I(r,"recv_ns") for r in att]

def nearest_att(t):
    j=bisect.bisect_left(ats,t)
    cand=[]
    for k in (j-1,j):
        if 0<=k<len(att): cand.append(att[k])
    return min(cand,key=lambda r:abs(I(r,"recv_ns")-t)) if cand else None

print("================ V25 FC RAW VERTICAL INERTIAL CHECK ================")
print("run:",root)
print("Independent check from raw FC HIGHRES_IMU + FC ATTITUDE.")
print("NED vertical is converted to VIO-style Z-up for easier sign comparison.")
print("Double integration is diagnostic only; do not treat its magnitude as ground truth.")
print()

for L in legs:
    leg=I(L,"leg")
    s=ev.get((leg,"START")); e=ev.get((leg,"END"))
    if not s or not e: continue
    t0,t1=I(s,"event_wall_ns"),I(e,"event_wall_ns")
    seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
    samples=[]
    for q in seg:
        t=I(q,"recv_ns"); a=nearest_att(t)
        if not a: continue
        dtmatch=abs(I(a,"recv_ns")-t)/1e6
        if dtmatch>20.0: continue
        # HIGHRES_IMU is FC FRD in this pipeline. At rest zacc is approximately -g.
        f_b=(F(q,"ax"),F(q,"ay"),F(q,"az"))
        rr=math.radians(F(a,"roll_deg")); pp=math.radians(F(a,"pitch_deg")); yy=math.radians(F(a,"yaw_deg"))
        f_n=mv(R_body_to_ned(rr,pp,yy),f_b)
        # specific force -> inertial acceleration in NED, where +Z is down
        az_ned=f_n[2]+G
        az_up=-az_ned
        samples.append((t,az_up,dtmatch))

    if len(samples)<3:
        print(f"LEG {leg}: insufficient samples")
        continue

    # Remove a small constant residual estimated from first/last 0.5 s combined.
    edge_ns=int(0.5e9)
    edge=[a for t,a,_ in samples if t<=t0+edge_ns or t>=t1-edge_ns]
    bias=mean(edge) if edge else 0.0

    v=0.0; z=0.0
    raw_v=0.0; raw_z=0.0
    prev_t=samples[0][0]
    for t,a,_ in samples[1:]:
        dt=(t-prev_t)/1e9
        if dt<=0 or dt>0.05:
            prev_t=t; continue
        # raw integration
        raw_z += raw_v*dt + 0.5*a*dt*dt
        raw_v += a*dt
        ac=a-bias
        z += v*dt + 0.5*ac*dt*dt
        v += ac*dt
        prev_t=t

    vals=[a for _,a,_ in samples]
    backend_dz=F(L,"dz_m")
    print(f"LEG {leg} {L['direction']}: backend dz={backend_dz*1000:+.1f} mm")
    print(f"  FC vertical accel mean={mean(vals):+.4f} m/s^2 edge_offset={bias:+.4f} m/s^2")
    print(f"  FC raw integrated      dv={raw_v:+.3f} m/s  dz={raw_z*1000:+.1f} mm")
    print(f"  FC edge-corrected      dv={v:+.3f} m/s  dz={z*1000:+.1f} mm")
    print(f"  ATT match mean={mean([m for _,_,m in samples]):.2f} ms")
    print()

print("INTERPRETATION:")
print("- If FC-derived vertical displacement sign follows backend Z on every leg, inertial input/gravity projection remains a strong suspect.")
print("- If FC-derived vertical signal is small or has unrelated signs while backend Z flips cleanly with direction, raw physical IMU input alone does not explain the defect.")
print("- Magnitudes from double integration are not trustworthy enough for calibration; use sign and repeatability only.")
