#!/usr/bin/env python3
import csv, math, sys, bisect, statistics
from pathlib import Path

WIN=0.75
STEP=0.25
SEARCH=4.0
GUARD=0.50

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def rms(xs): return math.sqrt(mean([x*x for x in xs])) if xs else float("nan")
def med(xs): return statistics.median(xs) if xs else float("nan")

def gyro_keys(r):
    for ks in (("gx","gy","gz"),("xgyro","ygyro","zgyro")):
        if all(k in r for k in ks): return ks
    return None

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_imu_stationary_plateaus.py RUN_DIR [RUN_DIR ...]")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    events=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    imu=sorted(imu,key=lambda r:I(r,"recv_ns"))
    ts=[I(r,"recv_ns") for r in imu]
    gk=gyro_keys(imu[0]) if imu else None
    ev={(I(r,"leg"),r["event"]):r for r in events}

    def window(t0,t1):
        i=bisect.bisect_left(ts,t0); j=bisect.bisect_right(ts,t1)
        return imu[i:j]

    def metrics(rr):
        if len(rr)<30: return None
        z=[-F(r,"az") for r in rr]
        n=[math.sqrt(F(r,"ax")**2+F(r,"ay")**2+F(r,"az")**2) for r in rr]
        if gk:
            gn=[math.sqrt(F(r,gk[0])**2+F(r,gk[1])**2+F(r,gk[2])**2) for r in rr]
            gr=rms(gn)
        else:
            gr=float("nan")
        nm=mean(n); ns=math.sqrt(mean([(x-nm)**2 for x in n]))
        return {"z":med(z),"norm":med(n),"norm_std":ns,"gyro_rms":gr,"n":len(rr)}

    def quietest(start_ns,end_ns):
        best=None
        t=start_ns
        while t+int(WIN*1e9)<=end_ns:
            rr=window(t,t+int(WIN*1e9))
            m=metrics(rr)
            if m:
                # Dimensionless quietness score. Used only to choose the least-disturbed window.
                gs=(m["gyro_rms"]/0.01) if math.isfinite(m["gyro_rms"]) else 0.0
                score=gs + m["norm_std"]/0.03
                cand=(score,t,m)
                if best is None or cand[0]<best[0]: best=cand
            t+=int(STEP*1e9)
        return best

    print("\n================ RUN ================")
    print(root)
    print("Stationary plateaus are selected as the quietest 0.75 s windows away from START/END handling.")
    print("No backend bias and no attitude transform are used.")
    print()

    rows=[]
    for L in legs:
        leg=I(L,"leg")
        es=ev.get((leg,"START")); ee=ev.get((leg,"END"))
        if not es or not ee: continue
        t0=I(es,"event_wall_ns"); t1=I(ee,"event_wall_ns")

        pre=quietest(t0-int(SEARCH*1e9), t0-int(GUARD*1e9))
        post=quietest(t1+int(GUARD*1e9), t1+int(SEARCH*1e9))
        motion=metrics(window(t0,t1))
        if not pre or not post or not motion:
            print(f"LEG {leg}: insufficient stationary data"); continue

        _,pt,pm=pre; _,qt,qm=post
        dz=qm["z"]-pm["z"]; dn=qm["norm"]-pm["norm"]
        mz=motion["z"]-pm["z"]; mn=motion["norm"]-pm["norm"]
        rows.append((L["direction"],dz,dn,mz,mn))

        print(f"LEG {leg} {L['direction']}: backend dz={F(L,'dz_m')*1000:+.1f} mm")
        print(f"  PRE  quiet plateau: Z={pm['z']:+.5f} |a|={pm['norm']:+.5f} gyroRMS={pm['gyro_rms']:.5f} normSTD={pm['norm_std']:.5f}")
        print(f"  MOVE relative PRE: Z={mz:+.5f} |a|={mn:+.5f}")
        print(f"  POST quiet plateau: Z={qm['z']:+.5f} |a|={qm['norm']:+.5f} gyroRMS={qm['gyro_rms']:.5f} normSTD={qm['norm_std']:.5f}")
        print(f"  PRE->POST shift:   Z={dz:+.5f} |a|={dn:+.5f} m/s^2")
        print()

    print("================ DIRECTION SUMMARY ================")
    for d in ("A->B","B->A"):
        rr=[r for r in rows if r[0]==d]
        if rr:
            print(f"{d}: PRE->POST Z={mean([r[1] for r in rr]):+.5f} |a|={mean([r[2] for r in rr]):+.5f} "
                  f"MOVE-vs-PRE Z={mean([r[3] for r in rr]):+.5f} |a|={mean([r[4] for r in rr]):+.5f} m/s^2")
    print()
    print("INTERPRETATION:")
    print("- PRE≈POST but MOVE differs: true motion-correlated sensor effect.")
    print("- PRE->POST remains large even on quiet plateaus: baseline/state shift persists after motion.")
    print("- If immediate-after analysis showed a shift but quiet-plateau analysis does not, the old result was handling/settling contamination.")
