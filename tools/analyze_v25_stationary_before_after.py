#!/usr/bin/env python3
import csv, math, statistics, sys, bisect
from pathlib import Path

G=9.81

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(x):return statistics.mean(x) if x else float("nan")
def median(x):return statistics.median(x) if x else float("nan")
def rms(x):return math.sqrt(mean([v*v for v in x])) if x else float("nan")
def angle(a,b):
    na=math.sqrt(sum(v*v for v in a)); nb=math.sqrt(sum(v*v for v in b))
    if na<=0 or nb<=0:return float("nan")
    c=sum(x*y for x,y in zip(a,b))/(na*nb)
    return math.degrees(math.acos(max(-1.0,min(1.0,c))))
def nearest(rows,times,t):
    j=bisect.bisect_left(times,t); c=[]
    for k in (j-1,j):
        if 0<=k<len(rows):c.append(rows[k])
    return min(c,key=lambda r:abs(I(r,"recv_ns")-t)) if c else None
def R(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(M,v):return tuple(sum(M[i][j]*v[j] for j in range(3)) for i in range(3))
def acc_rp_flu(ax,ay,az):
    return (math.degrees(math.atan2(ay,az)),
            math.degrees(math.atan2(-ax,math.hypot(ay,az))))

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_stationary_before_after.py RUN [RUN ...]")

print("================ V25 STATIONARY BEFORE vs AFTER ================")
print("Compares settled stationary windows before physical motion and after the final detected horizontal-dynamics interval.")
print("Uses raw FC IMU + FC ATTITUDE only. No VIO/backend velocity defines stationarity.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=sorted([r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"],key=lambda r:I(r,"recv_ns"))
    att=sorted(load(root/"jtzero_500mm_v25_attitude.csv"),key=lambda r:I(r,"recv_ns"))
    ats=[I(r,"recv_ns") for r in att]
    evr=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    ev={(I(r,"leg"),r["event"]):r for r in evr}

    print("\nRUN:",root)
    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee:continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
        if len(seg)<30:continue

        # Detect active horizontal dynamics, same philosophy as physical_motion_phases analyzer.
        dyn=[]
        full=[]
        for q in seg:
            a=nearest(att,ats,I(q,"recv_ns"))
            if not a:continue
            af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
            rr=math.radians(F(a,"roll_deg")); pp=math.radians(-F(a,"pitch_deg")); yy=math.radians(-F(a,"yaw_deg"))
            M=R(rr,pp,yy); aw=mv(M,af)
            h=math.hypot(aw[0],aw[1])
            gx=F(q,"gx") if "gx" in q else F(q,"xgyro")
            gy=F(q,"gy") if "gy" in q else F(q,"ygyro")
            gz=F(q,"gz") if "gz" in q else F(q,"zgyro")
            zy=M[2][1]*af[1]; zz=M[2][2]*af[2]-G
            full.append((I(q,"recv_ns"),af,a,h,gx,gy,gz,zy,zz))
        if len(full)<30:continue

        base=[v for v in full if v[0]<=t0+int(.75e9)]
        bh=median([v[3] for v in base]); noise=rms([v[3]-bh for v in base]); thr=max(.04,4*noise)

        # Smooth absolute horizontal-dynamics excess.
        raw=[abs(v[3]-bh) for v in full]
        sm=[]
        half=10
        for i in range(len(raw)):
            a=max(0,i-half); b=min(len(raw),i+half+1)
            sm.append(median(raw[a:b]))
        active=[x>thr for x in sm]

        # Bridge short gaps / remove short bursts.
        dts=[(full[i+1][0]-full[i][0])*1e-9 for i in range(len(full)-1) if full[i+1][0]>full[i][0]]
        dt=median(dts) if dts else .005
        gap=max(1,round(.15/dt)); minrun=max(1,round(.12/dt))
        def getruns(mask):
            out=[]; s=None
            for i,v in enumerate(mask+[False]):
                if v and s is None:s=i
                elif not v and s is not None:out.append((s,i-1));s=None
            return out
        rruns=getruns(active)
        for (a,b),(c,d) in zip(rruns,rruns[1:]):
            if c-b-1<=gap:
                for k in range(b+1,c):active[k]=True
        for a,b in getruns(active):
            if b-a+1<minrun:
                for k in range(a,b+1):active[k]=False
        rruns=getruns(active)
        if not rruns:
            print(f"LEG {leg}: no active interval"); continue

        first,last=rruns[0][0],rruns[-1][1]

        # Use robust stationary windows away from motion edges.
        before=[v for v in full if t0+int(.25e9)<=v[0]<=min(t0+int(1.5e9), full[first][0]-int(.25e9))]
        after=[v for v in full if full[last][0]+int(.25e9)<=v[0]<=t1-int(.10e9)]

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm")
        if len(before)<10 or len(after)<10:
            print(f"  insufficient stationary windows: BEFORE={len(before)} AFTER={len(after)}")
            continue

        def stats(q):
            acc=[v[1] for v in q]
            ax=mean([a[0] for a in acc]); ay=mean([a[1] for a in acc]); az=mean([a[2] for a in acc])
            ar,ap=acc_rp_flu(ax,ay,az)
            fr=mean([F(v[2],"roll_deg") for v in q]); fp=mean([F(v[2],"pitch_deg") for v in q])
            gx=mean([v[4] for v in q]); gy=mean([v[5] for v in q]); gz=mean([v[6] for v in q])
            zy=mean([v[7] for v in q]); zz=mean([v[8] for v in q])
            return dict(acc=(ax,ay,az),ar=ar,ap=ap,fr=fr,fp=fp,g=(gx,gy,gz),zy=zy,zz=zz,res=zy+zz)
        b=stats(before); a=stats(after)

        print(f"  BEFORE n={len(before)} ACC=[{b['acc'][0]:+.5f},{b['acc'][1]:+.5f},{b['acc'][2]:+.5f}]")
        print(f"    accel R/P=[{b['ar']:+.4f},{b['ap']:+.4f}] FC R/P=[{b['fr']:+.4f},{b['fp']:+.4f}]")
        print(f"    gyro=[{b['g'][0]:+.6f},{b['g'][1]:+.6f},{b['g'][2]:+.6f}] residual={b['res']:+.5f} m/s^2")
        print(f"  AFTER  n={len(after)} ACC=[{a['acc'][0]:+.5f},{a['acc'][1]:+.5f},{a['acc'][2]:+.5f}]")
        print(f"    accel R/P=[{a['ar']:+.4f},{a['ap']:+.4f}] FC R/P=[{a['fr']:+.4f},{a['fp']:+.4f}]")
        print(f"    gyro=[{a['g'][0]:+.6f},{a['g'][1]:+.6f},{a['g'][2]:+.6f}] residual={a['res']:+.5f} m/s^2")
        print(f"  AFTER-BEFORE:")
        print(f"    gravity-vector angle={angle(b['acc'],a['acc']):.4f} deg")
        print(f"    dAccelR={a['ar']-b['ar']:+.4f} dAccelP={a['ap']-b['ap']:+.4f} deg")
        print(f"    dFCroll={a['fr']-b['fr']:+.4f} dFCpitch={a['fp']-b['fp']:+.4f} deg")
        print(f"    dResidual={a['res']-b['res']:+.5f} m/s^2")
        print(f"    gyro AFTER norm={math.sqrt(sum(x*x for x in a['g'])):.6f} rad/s")

print("\nINTERPRETATION:")
print("- AFTER gyro near zero + changed accel gravity vector + matching FC roll change => new stationary physical attitude/state, not continuing motion.")
print("- AFTER gyro near zero but accel/FC return to BEFORE while residual persists => filtering/calibration memory becomes more plausible.")
print("- AFTER still has appreciable gyro => window is not truly stationary; do not interpret persistence yet.")
print("- Direction-consistent dResidual with direction-consistent stationary gravity change links the residual to the new settled state.")
