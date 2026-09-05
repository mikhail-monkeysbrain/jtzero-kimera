#!/usr/bin/env python3
import csv, os, math, sys

CHAIN=os.path.expanduser("~/jtzero_kimera_chain.csv")
EVENTS=os.path.expanduser("~/jtzero_500mm_v25_events.csv")
LEGS=os.path.expanduser("~/jtzero_500mm_v25_legs.csv")

K_STABLE_N=4
K_MAX_SPEED=0.020
K_MAX_SPAN=0.008

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(r[k])
def P(r): return (F(r,"state_px"),F(r,"state_py"),F(r,"state_pz"))
def V(r): return (F(r,"state_vx"),F(r,"state_vy"),F(r,"state_vz"))
def norm(v): return math.sqrt(sum(x*x for x in v))
def dist(a,b): return norm(tuple(x-y for x,y in zip(a,b)))
def mm(v): return v*1000.0

for p in (CHAIN,EVENTS,LEGS):
    if not os.path.exists(p):
        print("MISSING",p); sys.exit(2)

chain=read(CHAIN); events=read(EVENTS); legs=read(LEGS)
bykf={I(r,"keyframe"):r for r in chain}
end_events=[e for e in events if e["event"]=="END"]

print("================ V25 SETTLE GATE DIAGNOSTICS ================")
print(f"rule: {K_STABLE_N} consecutive backend states, |V| <= {K_MAX_SPEED*1000:.1f} mm/s, span <= {K_MAX_SPAN*1000:.1f} mm")

for e in end_events:
    leg=int(e["leg"]); k0=I(e,"keyframe")
    settled=None
    for l in legs:
        if int(l["leg"])==leg:
            settled=int(l["end_settled_kf"]); break
    print(f"\nLEG {leg} END press KF={k0} recorded_settled={settled if settled is not None else 'NONE'}")

    samples=[]
    first_qual=None
    maxk=max(bykf)
    rows=[]
    for k in range(k0, maxk+1):
        r=bykf.get(k)
        if not r: continue
        sp=norm(V(r))
        cleared=False
        why=""
        if sp > K_MAX_SPEED:
            samples=[]
            cleared=True
            why="SPEED"
        else:
            if samples and dist(P(r),P(samples[0])) > K_MAX_SPAN:
                samples=[]
                cleared=True
                why="SPAN"
            samples.append(r)
        cnt=len(samples)
        span=0.0 if not samples else max(dist(P(q),P(samples[0])) for q in samples)
        if cnt >= K_STABLE_N and first_qual is None:
            first_qual=k
        rows.append((k,sp,span,cnt,why,P(r)))
        if settled is not None and k >= settled+3: break
        if settled is None and first_qual is not None and k>=first_qual+3: break

    print(f"  recomputed first stable KF={first_qual}")
    if settled is not None and first_qual is not None:
        print(f"  recorded - recomputed = {settled-first_qual} KF")
    print("  keyframes around END / gate resets / qualification:")
    for k,sp,span,cnt,why,p in rows:
        important=(k<=k0+5 or why or (first_qual is not None and abs(k-first_qual)<=4) or (settled is not None and abs(k-settled)<=4))
        if important:
            print(f"   KF={k:3d} |V|={mm(sp):7.2f}mm/s span={mm(span):6.2f}mm count={cnt} reset={why or '-':5s} P=[{mm(p[0]):+.1f},{mm(p[1]):+.1f},{mm(p[2]):+.1f}]mm")

    r0=bykf.get(k0)
    if r0 and first_qual is not None and bykf.get(first_qual):
        d=dist(P(r0),P(bykf[first_qual]))
        dt=(F(bykf[first_qual],"timestamp_ns")-F(r0,"timestamp_ns"))*1e-9
        print(f"  ENDpress -> first stable: shift={mm(d):.2f}mm source_dt={dt:.2f}s")

print("\n================ SUMMARY ================")
print("If recomputed first stable matches recorded settled, the long delay is caused by the configured stability gate, not by a state-machine bookkeeping bug.")
