#!/usr/bin/env python3
import csv, math, os, statistics, sys
from pathlib import Path

BINS=[i/10.0 for i in range(11)]

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def mean(xs): return statistics.mean(xs) if xs else float("nan")
def med(xs): return statistics.median(xs) if xs else float("nan")
def norm(v): return math.sqrt(sum(x*x for x in v))
def angle_deg(a,b):
    na,nb=norm(a),norm(b)
    c=max(-1.0,min(1.0,sum(x*y for x,y in zip(a,b))/(na*nb)))
    return math.degrees(math.acos(c))
def vmean(rr,ks):
    return tuple(mean([F(r[k]) for r in rr]) for k in ks)
def nearest(rows,key,target):
    return min(rows,key=lambda r:abs(I(r[key])-target))

def central_phase(back,imu,leg,phase):
    br=[r for r in back if I(r["leg"])==leg and r["phase"]==phase]
    if not br: return []
    lo=min(I(r["timestamp_ns"]) for r in br)
    hi=max(I(r["timestamp_ns"]) for r in br)
    rr=sorted([r for r in imu if lo<=I(r["mapped_ns"])<=hi],key=lambda r:I(r["mapped_ns"]))
    k=len(rr)//4
    return rr[k:len(rr)-k] if len(rr)-2*k>=3 else rr

def qmul(a,b):
    aw,ax,ay,az=a; bw,bx,by,bz=b
    return (
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    )
def qnorm(q):
    n=math.sqrt(sum(x*x for x in q))
    return tuple(x/n for x in q)
def qexp(wdt):
    a=norm(wdt)
    if a<1e-15: return (1.0,0.0,0.0,0.0)
    s=math.sin(a/2.0)/a
    return (math.cos(a/2.0),wdt[0]*s,wdt[1]*s,wdt[2]*s)
def rot_angle_deg(q):
    w=max(-1.0,min(1.0,abs(q[0])))
    return math.degrees(2.0*math.acos(w))
def rotvec_deg(q):
    q=qnorm(q)
    if q[0]<0: q=tuple(-x for x in q)
    a=2.0*math.acos(max(-1.0,min(1.0,q[0])))
    s=math.sqrt(max(0.0,1.0-q[0]*q[0]))
    if s<1e-12: return (0.0,0.0,0.0)
    return tuple(math.degrees(a*q[i]/s) for i in (1,2,3))

if len(sys.argv)<2:
    print("usage: analyze_v23_rotation_vio_timeline.py RUN [RUN ...]")
    sys.exit(2)

alllegs=[]
print("================ V23 ROTATION ↔ VIO TIMELINE ================")
print("Purpose: locate when real gyro rotation occurs relative to VIO attitude/Z/translation.")
print("No claim of time-resolved scale error is made: true physical translation vs time was not logged.")

for arg in sys.argv[1:]:
    run=Path(arg)
    imu=sorted(load(run/"jtzero_500mm_v23.csv"),key=lambda r:I(r["mapped_ns"]))
    back=sorted(load(run/"jtzero_500mm_v23_backend.csv"),key=lambda r:I(r["timestamp_ns"]))
    legs=load(run/"jtzero_500mm_v23_legs.csv")
    bykf={I(r["keyframe"]):r for r in back}
    print("\nRUN:",run)

    for lr in legs:
        leg=I(lr["leg"]); direction=lr["direction"]; scale=F(lr["scale_horizontal"])
        s=bykf.get(I(lr["start_settled_kf"]))
        e=bykf.get(I(lr["end_press_kf"]))
        if not s or not e: continue
        t0=I(s["timestamp_ns"]); t1=I(e["timestamp_ns"])
        if t1<=t0: continue
        bseg=[r for r in back if t0<=I(r["timestamp_ns"])<=t1]
        iseg=[r for r in imu if t0<=I(r["mapped_ns"])<=t1]
        if len(bseg)<2 or len(iseg)<2: continue

        st0=central_phase(back,imu,leg,"SETTLE_START")
        st1=central_phase(back,imu,leg,"SETTLE_END")
        if len(st0)<3 or len(st1)<3: continue
        g0=vmean(st0,("gx","gy","gz")); g1=vmean(st1,("gx","gy","gz"))
        gbias=tuple((g0[i]+g1[i])*0.5 for i in range(3))

        # Cumulative gyro orientation at every IMU sample.
        q=(1.0,0.0,0.0,0.0); qs=[]; prev=None
        for r in iseg:
            t=I(r["mapped_ns"])
            if prev is not None:
                dt=(t-prev[0])*1e-9
                if 0.0<dt<=0.03:
                    w=tuple(0.5*(F(prev[1][k])+F(r[k]))-gbias[j]
                            for j,k in enumerate(("gx","gy","gz")))
                    q=qnorm(qmul(q,qexp(tuple(x*dt for x in w))))
            qs.append((t,q))
            prev=(t,r)

        ex=F(e["px_m"])-F(s["px_m"]); ey=F(e["py_m"])-F(s["py_m"])
        en=math.hypot(ex,ey)
        if en<1e-9: continue
        ux,uy=ex/en,ey/en
        sx,sy,sz=F(s["px_m"]),F(s["py_m"]),F(s["pz_m"])
        sr,sp=F(s["roll_deg"]),F(s["pitch_deg"])

        samples=[]
        for frac in BINS:
            target=int(round(t0+frac*(t1-t0)))
            br=nearest(bseg,"timestamp_ns",target)
            qt=min(qs,key=lambda z:abs(z[0]-target))[1]
            rv=rotvec_deg(qt)
            grot=rot_angle_deg(qt)
            dx=(F(br["px_m"])-sx)*1000.0
            dy=(F(br["py_m"])-sy)*1000.0
            along=dx*ux+dy*uy
            cross=-dx*uy+dy*ux
            valong=(F(br["vx_m_s"])*ux+F(br["vy_m_s"])*uy)*1000.0
            droll=F(br["roll_deg"])-sr
            dpitch=F(br["pitch_deg"])-sp
            vtilt=math.hypot(droll,dpitch)
            z=(F(br["pz_m"])-sz)*1000.0
            vz=F(br["vz_m_s"])*1000.0
            samples.append(dict(frac=frac,grot=grot,rv=rv,vtilt=vtilt,along=along,cross=cross,
                                valong=valong,z=z,vz=vz))

        # Trapezoidal exposure in deg*s over normalized samples.
        exposure=0.0
        for a,b in zip(samples[:-1],samples[1:]):
            dt=(b["frac"]-a["frac"])*(t1-t0)*1e-9
            exposure+=0.5*(a["grot"]+b["grot"])*dt

        endpoint=max(samples[-1]["grot"],1e-9)
        def onset(level):
            th=level*endpoint
            for x in samples:
                if x["grot"]>=th: return x["frac"]
            return float("nan")

        print(f"\nLEG {leg} {direction}: scale={scale:.6f} duration={(t1-t0)*1e-9:.3f}s "
              f"gyro_end={samples[-1]['grot']:.3f}deg exposure={exposure:.3f}deg*s")
        print(f"  gyro onset: 20%={onset(0.2)*100:.0f}% 50%={onset(0.5)*100:.0f}% 80%={onset(0.8)*100:.0f}%")
        print("  time | gyroRot  rotvecFRD[x,y,z]       VIOtilt    along      V_along       Z        Vz")
        for x in samples:
            print(f"  {int(x['frac']*100):3d}% | {x['grot']:7.3f}  "
                  f"[{x['rv'][0]:+6.3f},{x['rv'][1]:+6.3f},{x['rv'][2]:+6.3f}]  "
                  f"{x['vtilt']:7.3f}deg  {x['along']:8.1f}mm {x['valong']:+10.1f}mm/s "
                  f"{x['z']:+8.1f}mm {x['vz']:+9.1f}mm/s")
        alllegs.append(dict(run=run.name,leg=leg,direction=direction,scale=scale,
                            gyro_end=samples[-1]["grot"],exposure=exposure,
                            onset20=onset(0.2),onset50=onset(0.5),onset80=onset(0.8)))

print("\n================ CROSS-LEG SUMMARY ================")
for d in ("A->B","B->A"):
    rr=[r for r in alllegs if r["direction"]==d]
    if not rr: continue
    print(f"{d}: n={len(rr)} scale median={med([r['scale'] for r in rr]):.4f} "
          f"gyro_end median={med([r['gyro_end'] for r in rr]):.3f}deg "
          f"exposure median={med([r['exposure'] for r in rr]):.3f}deg*s "
          f"onset50 median={med([r['onset50'] for r in rr])*100:.0f}%")
    for r in rr:
        print(f"  {r['run']} LEG{r['leg']}: scale={r['scale']:.4f} "
              f"gyro_end={r['gyro_end']:.3f} exposure={r['exposure']:.3f} "
              f"onset50={r['onset50']*100:.0f}%")

print("\n================ INTERPRETATION ================")
print("- If gyro rotation starts before/with VIO tilt and Z/velocity changes, a mechanical-rotation coupling is temporally plausible.")
print("- If VIO translation/velocity asymmetry is already established well before gyro rotation onset, rotation cannot be its sole cause.")
print("- Endpoint scale cannot be converted into time-resolved scale error here because true 500-mm translation versus time was not independently logged.")
print("- Exposure differences can explain why similar endpoint tilt may yield different scale, but with six legs this remains a mechanism clue, not statistical proof.")
