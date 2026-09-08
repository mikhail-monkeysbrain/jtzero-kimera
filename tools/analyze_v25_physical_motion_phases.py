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
def nearest(rows, times, t):
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
def smooth(vals, half=10):
    out=[]
    for i in range(len(vals)):
        a=max(0,i-half); b=min(len(vals),i+half+1)
        out.append(median(vals[a:b]))
    return out
def runs(mask):
    out=[]; s=None
    for i,v in enumerate(mask+[False]):
        if v and s is None:s=i
        elif not v and s is not None:out.append((s,i-1)); s=None
    return out

if len(sys.argv)<2:raise SystemExit("usage: analyze_v25_physical_motion_phases.py RUN [RUN ...]")
print("================ V25 PHYSICAL MOTION PHASES ================")
print("Detects active horizontal dynamics from raw FC IMU transformed with FC attitude.")
print("No VIO/backend velocity is used to define physical motion phases.")
print("Threshold is derived from the settled-start horizontal acceleration noise in each leg.")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=sorted([r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"],key=lambda r:I(r,"recv_ns"))
    att=sorted(load(root/"jtzero_500mm_v25_attitude.csv"),key=lambda r:I(r,"recv_ns"))
    ats=[I(r,"recv_ns") for r in att]
    evr=load(root/"jtzero_500mm_v25_events.csv"); legs=load(root/"jtzero_500mm_v25_legs.csv")
    ev={(I(r,"leg"),r["event"]):r for r in evr}
    print("\nRUN:",root)
    for L in legs:
        leg=I(L,"leg"); es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee:continue
        t0,t1=I(es,"event_wall_ns"),I(ee,"event_wall_ns")
        vals=[]
        for q in imu:
            t=I(q,"recv_ns")
            if t<t0 or t>t1:continue
            a=nearest(att,ats,t)
            if not a:continue
            af=(F(q,"ax"),-F(q,"ay"),-F(q,"az"))
            rr=math.radians(F(a,"roll_deg")); pp=math.radians(-F(a,"pitch_deg")); yy=math.radians(-F(a,"yaw_deg"))
            M=R(rr,pp,yy); aw=mv(M,af)
            zy=M[2][1]*af[1]; zz=M[2][2]*af[2]-G
            gx=F(q,"gx") if "gx" in q else F(q,"xgyro")
            vals.append([t,math.hypot(aw[0],aw[1]),zy,zz,gx,F(a,"roll_deg")])
        if len(vals)<30:continue

        base=[v for v in vals if v[0]<=t0+int(.75e9)]
        bh=median([v[1] for v in base]); sy=rms([v[1]-bh for v in base])
        by=median([v[2] for v in base]); bz=median([v[3] for v in base])
        hs=smooth([abs(v[1]-bh) for v in vals],10)
        # Conservative per-run threshold: at least 0.04 m/s2 and >=4x settled RMS.
        thr=max(0.04,4.0*sy)
        active=[h>thr for h in hs]
        # bridge gaps shorter than ~0.15 s, remove bursts shorter than ~0.12 s
        dt=median([(vals[i+1][0]-vals[i][0])*1e-9 for i in range(len(vals)-1)])
        gap=max(1,round(.15/dt)); minrun=max(1,round(.12/dt))
        ar=runs(active)
        for (a,b),(c,d) in zip(ar,ar[1:]):
            if c-b-1<=gap:
                for k in range(b+1,c):active[k]=True
        for a,b in runs(active):
            if b-a+1<minrun:
                for k in range(a,b+1):active[k]=False
        rruns=runs(active)

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f}mm dur={(t1-t0)*1e-9:.2f}s")
        print(f"  settled hAcc baseline={bh:.4f} RMSnoise={sy:.4f} threshold={thr:.4f} m/s^2")
        if not rruns:
            print("  no active horizontal-dynamics intervals detected"); continue
        for n,(a,b) in enumerate(rruns,1):
            q=vals[a:b+1]
            res=[(x[2]-by)+(x[3]-bz) for x in q]
            y=[x[2]-by for x in q]; z=[x[3]-bz for x in q]
            gr=[x[4] for x in q]; roll=[x[5] for x in q]
            ta=(q[0][0]-t0)*1e-9; tb=(q[-1][0]-t0)*1e-9
            print(f"  ACTIVE#{n}: t={ta:.2f}..{tb:.2f}s dur={tb-ta:.2f}s "
                  f"hAccMean={mean([hs[k] for k in range(a,b+1)]):.4f}")
            print(f"    Y={mean(y):+.5f} Z={mean(z):+.5f} residual={mean(res):+.5f} m/s^2 "
                  f"gyroXmean={mean(gr):+.5f}rad/s rollDelta={roll[-1]-roll[0]:+.3f}deg")
        # Before / active envelope / after summary
        first,last=rruns[0][0],rruns[-1][1]
        zones=[("BEFORE",0,first-1),("ACTIVE-ENVELOPE",first,last),("AFTER",last+1,len(vals)-1)]
        for name,a,b in zones:
            if a>b:continue
            q=vals[a:b+1]; res=[(x[2]-by)+(x[3]-bz) for x in q]
            print(f"  {name}: t={(q[0][0]-t0)*1e-9:.2f}..{(q[-1][0]-t0)*1e-9:.2f}s "
                  f"residual={mean(res):+.5f} m/s^2")

print("\nINTERPRETATION:")
print("- ACTIVE intervals are raw-IMU horizontal-dynamics intervals, not VIO-derived motion.")
print("- Directional residual appearing only inside/after ACTIVE localizes the error to physical-motion response rather than a static START offset.")
print("- Residual that remains after the final ACTIVE interval suggests settling/filter memory or a changed physical attitude/state.")
print("- Multiple ACTIVE intervals can correspond to pushes, speed changes or braking; this analyzer does not name them without independent evidence.")
