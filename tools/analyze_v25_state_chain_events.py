#!/usr/bin/env python3
import csv, os, sys, math

CHAIN=os.path.expanduser("~/jtzero_kimera_chain.csv")
EVENTS=os.path.expanduser("~/jtzero_500mm_v25_events.csv")
LEGS=os.path.expanduser("~/jtzero_500mm_v25_legs.csv")

def read(path):
    with open(path,newline="") as f: return list(csv.DictReader(f))
def f(r,k): return float(r[k])
def i(r,k): return int(r[k])
def p(r,pfx): return (f(r,pfx+"_px"),f(r,pfx+"_py"),f(r,pfx+"_pz"))
def dist(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def fmt(v): return "["+",".join(f"{x*1000:+.1f}" for x in v)+"]mm"

for x in (CHAIN,EVENTS,LEGS):
    if not os.path.exists(x):
        print("MISSING:",x); sys.exit(2)
chain=read(CHAIN); events=read(EVENTS); legs=read(LEGS)
bykf={i(r,"keyframe"):r for r in chain}

print("================ V25 STATE / PRED / INCR EVENT ANALYSIS ================")
print(f"chain rows={len(chain)} KF={min(bykf)}..{max(bykf)} events={len(events)} completed_legs={len(legs)}")

for e in events:
    k=i(e,"keyframe")
    r=bykf.get(k)
    print(f"\nEVENT {e['event']} LEG{e['leg']} KF={k} age={f(e,'state_age_at_event_ms'):.1f}ms")
    if not r:
        print("  chain row: MISSING"); continue
    pp=p(r,"pred"); sp=p(r,"state"); ip=p(r,"incr")
    print(f"  PRED ={fmt(pp)}")
    print(f"  STATE={fmt(sp)}")
    print(f"  INCR ={fmt(ip)}")
    print(f"  PRED<->STATE={dist(pp,sp)*1000:.2f}mm STATE<->INCR={dist(sp,ip)*1000:.2f}mm")

print("\n================ COMPLETED LEG ENDPOINTS ================")
for l in legs:
    leg=l["leg"]; ks=i(l,"start_settled_kf"); ke=i(l,"end_settled_kf")
    rs=bykf.get(ks); re=bykf.get(ke)
    print(f"\nLEG {leg} {l['direction']} settled KF {ks}->{ke}")
    if not rs or not re:
        print("  chain settled row missing"); continue
    for name in ("pred","state","incr"):
        a=p(rs,name); b=p(re,name); d=tuple(y-x for x,y in zip(a,b))
        print(f"  {name.upper():5s} delta={fmt(d)} |d|={dist(a,b)*1000:.2f}mm")
    kp=i(l,"end_press_kf")
    rp=bykf.get(kp)
    if rp:
        print(f"  END settling KF {kp}->{ke}")
        for name in ("pred","state","incr"):
            a=p(rp,name); b=p(re,name)
            print(f"    {name.upper():5s} shift={dist(a,b)*1000:.2f}mm")

print("\n================ FIRST DIVERGENCE ================")
for threshold in (10,20,50,100,250,500):
    hit=next((r for r in chain if f(r,"state_incr_mm")>=threshold),None)
    if hit:
        print(f"STATE-INCR >= {threshold:3d}mm: KF={i(hit,'keyframe')} value={f(hit,'state_incr_mm'):.2f}mm")
for threshold in (5,10,20,50):
    hit=next((r for r in chain if f(r,"pred_state_mm")>=threshold),None)
    if hit:
        print(f"PRED-STATE >= {threshold:3d}mm: KF={i(hit,'keyframe')} value={f(hit,'pred_state_mm'):.2f}mm")

print("\n================ WINDOWS AROUND EVENTS ================")
for e in events:
    k=i(e,"keyframe")
    print(f"\n{e['event']} LEG{e['leg']} KF={k}")
    for q in range(max(min(bykf),k-3),min(max(bykf),k+3)+1):
        r=bykf.get(q)
        if not r: continue
        print(f" KF={q:3d} SI={f(r,'state_incr_mm'):7.2f}mm PS={f(r,'pred_state_mm'):6.2f}mm PSV={f(r,'pred_state_v_mm_s'):6.2f}mm/s STATE={fmt(p(r,'state'))}")
