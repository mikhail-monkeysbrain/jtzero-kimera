#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def norm3(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))

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
    raise SystemExit("usage: analyze_v25_vertical_leakage_source.py RUN_DIR")

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

print("================ V25 VERTICAL LEAKAGE SOURCE CHECK ================")
print("run:",root)
print("Compares three signals:")
print("  1) accel magnitude residual: orientation-independent")
print("  2) accel along its own static gravity axis: mostly sensor/body-level")
print("  3) vertical after applying FC attitude: world-frame projection")
print()

for L in legs:
    leg=I(L,"leg")
    es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
    if not es or not ee: continue
    t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
    seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
    if len(seg)<20:
        print(f"LEG {leg}: insufficient IMU samples"); continue

    # Convert raw FC FRD specific force to FLU, same convention fed to Kimera.
    samples=[]
    for q in seg:
        t=I(q,"recv_ns")
        a_flu=(F(q,"ax"), -F(q,"ay"), -F(q,"az"))
        samples.append((t,a_flu))

    edge_ns=int(0.50e9)
    edge=[a for t,a in samples if t<=t0+edge_ns or t>=t1-edge_ns]
    if not edge: edge=[a for _,a in samples[:max(5,len(samples)//10)]]
    gvec=tuple(mean([a[j] for a in edge]) for j in range(3))
    gmag=norm3(gvec)
    gdir=tuple(x/gmag for x in gvec)

    mag_res=[]
    self_axis=[]
    world_up=[]
    proj_delta=[]
    horiz=[]
    att_match=[]

    for t,a in samples:
        amag=norm3(a)
        mres=amag-gmag
        sres=dot(a,gdir)-gmag

        ar=nearest_att(t)
        if ar is None: continue
        match=abs(I(ar,"recv_ns")-t)/1e6
        if match>20.0: continue

        # Convert FLU back to FRD for FC attitude transform.
        a_frd=(a[0],-a[1],-a[2])
        rr=math.radians(F(ar,"roll_deg"))
        pp=math.radians(F(ar,"pitch_deg"))
        yy=math.radians(F(ar,"yaw_deg"))
        f_n=mv(R_body_to_ned(rr,pp,yy),a_frd)
        az_up=-(f_n[2]+G)

        mag_res.append(mres)
        self_axis.append(sres)
        world_up.append(az_up)
        proj_delta.append(az_up-sres)
        horiz.append(math.hypot(a[0]-gvec[0],a[1]-gvec[1]))
        att_match.append(match)

    print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f} mm")
    print(f"  static gravity |g|={gmag:.5f} m/s^2 axis=[{gdir[0]:+.4f},{gdir[1]:+.4f},{gdir[2]:+.4f}]")
    print(f"  magnitude residual mean = {mean(mag_res):+.5f} m/s^2")
    print(f"  own-gravity-axis mean   = {mean(self_axis):+.5f} m/s^2")
    print(f"  FC-attitude world-Z mean= {mean(world_up):+.5f} m/s^2")
    print(f"  projection-only delta   = {mean(proj_delta):+.5f} m/s^2")
    print(f"  horizontal dynamic mean = {mean(horiz):.5f} m/s^2  ATT match={mean(att_match):.2f} ms")
    print()

print("INTERPRETATION:")
print("- If magnitude/own-axis residuals stay near zero or do not flip with direction, but FC-attitude world-Z does flip, the sign is being created mainly by attitude/frame projection.")
print("- If own-axis residual already flips with A->B/B->A like backend Z, the accelerometer/body-level signal itself remains a strong suspect.")
print("- A large projection-only delta with the backend sign points toward frame convention, attitude, or horizontal-to-vertical leakage rather than a true vertical acceleration.")
