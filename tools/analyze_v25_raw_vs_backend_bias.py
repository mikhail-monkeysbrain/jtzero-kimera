#!/usr/bin/env python3
import csv, math, bisect
from pathlib import Path

H=Path("/home/vio")
IMU=H/"jtzero_500mm_v25.csv"
ATT=H/"jtzero_500mm_v25_attitude.csv"
LEGS=H/"jtzero_500mm_v25_legs.csv"
BACK=H/"jtzero_500mm_v25_backend.csv"
OUT=H/"jtzero_v25_raw_vs_backend_bias.csv"
G=9.81

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(float(r[k]))
def mean(x): return sum(x)/len(x) if x else float("nan")
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(v): return math.sqrt(dot(v,v))

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v): return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

imu=[r for r in load(IMU) if r.get("type")=="IMU"]
att=load(ATT); legs=load(LEGS); be=load(BACK)
imu.sort(key=lambda r:I(r,"recv_ns")); att.sort(key=lambda r:I(r,"recv_ns"))
att_t=[I(r,"recv_ns") for r in att]
bykf={I(r,"keyframe"):r for r in be}

def nearest_att(t):
    j=bisect.bisect_left(att_t,t); cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(att): cand.append(att[k])
    return min(cand,key=lambda r:abs(I(r,"recv_ns")-t)) if cand else None

print("================ V25 RAW FC ACCEL vs BACKEND BA ================")
rows=[]
for L in legs:
    leg=I(L,"leg"); ks=I(L,"start_settled_kf"); ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e: continue
    t0=I(s,"callback_wall_ns"); t1=I(e,"callback_wall_ns")
    seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
    if not seg:
        print(f"LEG {leg}: no raw IMU in wall window"); continue

    d=(F(e,"px_m")-F(s,"px_m"),F(e,"py_m")-F(s,"py_m"),0.0); dn=norm(d)
    u=(d[0]/dn,d[1]/dn,0.0); uc=(-u[1],u[0],0.0)

    raw_al=[]; raw_cr=[]; raw_z=[]; dyn_al=[]; match_ms=[]
    for q in seg:
        a=nearest_att(I(q,"recv_ns"))
        if not a: continue
        # Raw CSV accelerometer is FRD. Convert specific force to FLU.
        af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
        # ATTITUDE raw is FRD/NED-style. Relative JT convention: roll same,
        # pitch/yaw sign-flipped for FLU comparison.
        rr=math.radians(F(a,"roll_deg")); pp=math.radians(-F(a,"pitch_deg")); yy=math.radians(-F(a,"yaw_deg"))
        aw=mv(RzRyRx(rr,pp,yy),af)
        raw_al.append(dot(aw,u)); raw_cr.append(dot(aw,uc)); raw_z.append(aw[2])
        # At rest world specific force should be approximately +G on Z.
        # Horizontal projection is therefore already a useful dynamic/residual observable.
        dyn_al.append(dot(aw,u))
        match_ms.append(abs(I(a,"recv_ns")-I(q,"recv_ns"))/1e6)

    bseg=[r for r in be if ks<=I(r,"keyframe")<=ke]
    ba_al=[]
    for r in bseg:
        ba=(F(r,"bax"),F(r,"bay"),F(r,"baz"))
        R=RzRyRx(math.radians(F(r,"roll_deg")),math.radians(F(r,"pitch_deg")),math.radians(F(r,"yaw_deg")))
        ba_al.append(dot(mv(R,ba),u))

    xy=F(L,"horizontal_m")*1000; scale=xy/500
    r=dict(leg=leg,direction=L["direction"],scale=scale,raw_samples=len(raw_al),
           raw_world_along_mean_m_s2=mean(raw_al),raw_world_cross_mean_m_s2=mean(raw_cr),
           raw_world_z_mean_m_s2=mean(raw_z),backend_ba_along_mean_m_s2=mean(ba_al),
           attitude_match_mean_ms=mean(match_ms))
    rows.append(r)
    print(f"LEG {leg} {L['direction']}: scale={scale:.4f} samples={len(raw_al)}")
    print(f"  RAW world specific-force along/cross/z = [{r['raw_world_along_mean_m_s2']:+.5f},{r['raw_world_cross_mean_m_s2']:+.5f},{r['raw_world_z_mean_m_s2']:+.5f}] m/s^2")
    print(f"  backend BA_along = {r['backend_ba_along_mean_m_s2']:+.5f} m/s^2  ATT match mean={r['attitude_match_mean_ms']:.2f}ms")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    z=[r for r in rows if r["direction"]==d]
    if z:
        print(f"{d}: scale={mean([r['scale'] for r in z]):.4f} RAW_along={mean([r['raw_world_along_mean_m_s2'] for r in z]):+.5f} BA_along={mean([r['backend_ba_along_mean_m_s2'] for r in z]):+.5f}")

if rows:
    with OUT.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

print("\nDECISION:")
print("- RAW_along repeats the A->B/B->A asymmetry with the same physical pattern -> sensor/motion excitation branch gains support.")
print("- RAW_along is near-symmetric/near-zero while backend BA_along flips strongly -> optimizer bias-state / visual-inertial coupling is primary.")
print("- Note: mean acceleration over a finite hand-motion leg can depend on endpoint timing; interpret together with both repeated legs, not one leg.")
print("Saved:",OUT)
