#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def rms(xs): return math.sqrt(mean([x*x for x in xs])) if xs else float("nan")
def pct(xs,p):
    if not xs: return float("nan")
    s=sorted(xs); x=(len(s)-1)*p
    i=int(math.floor(x)); j=int(math.ceil(x))
    return s[i] if i==j else s[i]*(j-x)+s[j]*(x-i)
def norm(v): return math.sqrt(sum(x*x for x in v))

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v): return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

if len(sys.argv)>2:
    raise SystemExit("usage: analyze_v25_raw_motion_profile.py [run_dir]")

root=Path(sys.argv[1]) if len(sys.argv)==2 else Path("/home/vio")
imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
att=load(root/"jtzero_500mm_v25_attitude.csv")
events=load(root/"jtzero_500mm_v25_events.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")

imu.sort(key=lambda r:I(r,"recv_ns"))
att.sort(key=lambda r:I(r,"recv_ns"))
atts=[I(r,"recv_ns") for r in att]

def nearest_att(t):
    j=bisect.bisect_left(atts,t); cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(att): cand.append(att[k])
    return min(cand,key=lambda r:abs(I(r,"recv_ns")-t)) if cand else None

ev={}
for r in events:
    ev[(I(r,"leg"),r["event"])]=r

print("================ V25 RAW-SENSOR MOTION PROFILE ================")
print("run:",root)
rows=[]
for L in legs:
    leg=I(L,"leg")
    es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
    if not es or not ee:
        print(f"LEG {leg}: missing START/END event"); continue
    t0=I(es,"event_wall_ns"); t1=I(ee,"event_wall_ns")
    seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
    if not seg:
        print(f"LEG {leg}: no IMU samples"); continue

    horiz=[]; zdev=[]; totaldev=[]; gx=[]; gy=[]; gz=[]; gnorm=[]; match=[]
    roll=[]; pitch=[]; yaw=[]
    for q in seg:
        a=nearest_att(I(q,"recv_ns"))
        if not a: continue
        # raw HIGHRES_IMU is FRD; convert specific force to FLU
        af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
        rr=math.radians(F(a,"roll_deg"))
        pp=math.radians(-F(a,"pitch_deg"))
        yy=math.radians(-F(a,"yaw_deg"))
        aw=mv(RzRyRx(rr,pp,yy),af)
        horiz.append(math.hypot(aw[0],aw[1]))
        zdev.append(aw[2]-9.81)
        totaldev.append(norm((aw[0],aw[1],aw[2]-9.81)))
        roll.append(F(a,"roll_deg")); pitch.append(F(a,"pitch_deg")); yaw.append(F(a,"yaw_deg"))
        match.append(abs(I(a,"recv_ns")-I(q,"recv_ns"))/1e6)

        # gyro columns may be named gx/gy/gz or xgyro/ygyro/zgyro depending on logger
        kg=None
        for keys in (("gx","gy","gz"),("xgyro","ygyro","zgyro")):
            if all(k in q and q[k] not in ("",None) for k in keys):
                kg=keys; break
        if kg:
            gv=(F(q,kg[0]),F(q,kg[1]),F(q,kg[2]))
            gx.append(gv[0]); gy.append(gv[1]); gz.append(gv[2]); gnorm.append(norm(gv))

    dur=(t1-t0)*1e-9
    xy=F(L,"horizontal_m")*1000.0
    r=dict(
      leg=leg,direction=L["direction"],scale=xy/500.0,duration_s=dur,samples=len(horiz),
      horiz_acc_rms=rms(horiz),horiz_acc_p90=pct(horiz,0.9),horiz_acc_max=max(horiz),
      z_acc_dev_rms=rms(zdev),total_dyn_acc_rms=rms(totaldev),
      gyro_rms=rms(gnorm) if gnorm else float("nan"),gyro_p90=pct(gnorm,0.9) if gnorm else float("nan"),
      roll_span=max(roll)-min(roll),pitch_span=max(pitch)-min(pitch),yaw_span=max(yaw)-min(yaw),
      att_match_mean_ms=mean(match)
    )
    rows.append(r)
    print(f"\nLEG {leg} {L['direction']}: scale={r['scale']:.4f} dur={dur:.2f}s samples={len(horiz)}")
    print(f"  RAW horiz accel RMS/p90/max = {r['horiz_acc_rms']:.4f}/{r['horiz_acc_p90']:.4f}/{r['horiz_acc_max']:.4f} m/s^2")
    print(f"  RAW z-dev RMS={r['z_acc_dev_rms']:.4f} total-dynamic RMS={r['total_dyn_acc_rms']:.4f} m/s^2")
    if gnorm:
        print(f"  RAW gyro norm RMS/p90 = {r['gyro_rms']:.5f}/{r['gyro_p90']:.5f} rad/s")
    print(f"  FC attitude span R/P/Y = {r['roll_span']:.3f}/{r['pitch_span']:.3f}/{r['yaw_span']:.3f} deg")
    print(f"  ATT match mean={r['att_match_mean_ms']:.2f} ms")

print("\n================ PAIRWISE RAW-INPUT COMPARISON ================")
by={r["leg"]:r for r in rows}
for a,b in ((1,2),(3,4)):
    if a not in by or b not in by: continue
    A=by[a]; B=by[b]
    print(f"PAIR {a}->{b}:")
    print(f"  scale delta B-A = {B['scale']-A['scale']:+.4f}")
    print(f"  duration delta = {B['duration_s']-A['duration_s']:+.2f}s")
    print(f"  horiz-acc RMS delta = {B['horiz_acc_rms']-A['horiz_acc_rms']:+.4f} m/s^2")
    print(f"  horiz-acc p90 delta = {B['horiz_acc_p90']-A['horiz_acc_p90']:+.4f} m/s^2")
    print(f"  total-dyn RMS delta = {B['total_dyn_acc_rms']-A['total_dyn_acc_rms']:+.4f} m/s^2")
    print(f"  pitch-span delta = {B['pitch_span']-A['pitch_span']:+.3f} deg")
    print(f"  yaw-span delta = {B['yaw_span']-A['yaw_span']:+.3f} deg")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if rr:
        m=lambda k:mean([r[k] for r in rr])
        print(f"{d}: scale={m('scale'):.4f} dur={m('duration_s'):.2f}s "
              f"hAccRMS={m('horiz_acc_rms'):.4f} hAccP90={m('horiz_acc_p90'):.4f} "
              f"dynRMS={m('total_dyn_acc_rms'):.4f} pitchSpan={m('pitch_span'):.3f} yawSpan={m('yaw_span'):.3f}")

print("\nINTERPRETATION:")
print("- This uses raw FC IMU + FC ATTITUDE + operator event times, not VIO velocity.")
print("- If B->A shows consistently larger raw acceleration/excitation, motion profile remains a real confounder.")
print("- If raw excitation is comparable while scale remains worse on B->A, the case for visual/estimator asymmetry strengthens.")
print("- Backend speed must not be used as proof of input-motion differences because it is itself a VIO output.")
