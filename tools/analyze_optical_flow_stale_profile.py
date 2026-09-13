#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d
def i(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d
def pct(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    return xs[min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))]

def main():
    ap=argparse.ArgumentParser(description="Forensic of valid-but-stale optical-flow frames")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")
    ev=[(j,i(r,"return_event")) for j,r in enumerate(rows) if i(r,"return_event") in (1,2,3)]
    ia=next((j for j,e in ev if e==1),None)
    ih=next((j for j,e in ev if e==3 and ia is not None and j>ia),None)
    if ia is None or ih is None: raise SystemExit(f"Нужны A/H events: {ev}")
    seg=rows[ia:ih+1]
    stale=[r for r in seg if i(r,"valid")==1 and i(r,"flow_sent")!=1]
    sent=[r for r in seg if i(r,"valid")==1 and i(r,"flow_sent")==1]
    def stats(rs,name):
        if not rs:
            print(name+": нет кадров"); return
        lat=[f(r,"frame_pipeline_latency_ms") for r in rs]
        mag=[math.hypot(f(r,"flow_body_x"),f(r,"flow_body_y")) for r in rs]
        tr=[f(r,"tracked") for r in rs]; inl=[f(r,"inliers") for r in rs]
        dt=[f(r,"dt_s")*1000 for r in rs]
        print(name)
        print(f"  n={len(rs)}")
        print(f"  latency ms median/p95/max = {statistics.median(lat):.1f}/{pct(lat,.95):.1f}/{max(lat):.1f}")
        print(f"  |flow| rad/s median/p95/max = {statistics.median(mag):.3f}/{pct(mag,.95):.3f}/{max(mag):.3f}")
        print(f"  dt ms median/p95/max = {statistics.median(dt):.1f}/{pct(dt,.95):.1f}/{max(dt):.1f}")
        print(f"  tracked median/p05 = {statistics.median(tr):.0f}/{pct(tr,.05):.0f}")
        print(f"  inliers median/p05 = {statistics.median(inl):.0f}/{pct(inl,.05):.0f}")
    print("===== STALE FLOW FORENSIC =====")
    stats(sent,"SENT VALID")
    print()
    stats(stale,"STALE VALID")
    print()
    print("STALE FRAMES:")
    for r in stale:
        print(f"  frame={i(r,'frame'):4d} latency={f(r,'frame_pipeline_latency_ms'):6.1f}ms "
              f"dt={f(r,'dt_s')*1000:5.1f}ms flow=({f(r,'flow_body_x'):+.3f},{f(r,'flow_body_y'):+.3f}) "
              f"|f|={math.hypot(f(r,'flow_body_x'),f(r,'flow_body_y')):.3f} "
              f"tracked/inliers={i(r,'tracked')}/{i(r,'inliers')} "
              f"luna={f(r,'luna_m'):.3f}")
    print()
    if stale:
        big=sum(math.hypot(f(r,"flow_body_x"),f(r,"flow_body_y"))>=0.2 for r in stale)
        print(f"stale with |flow|>=0.2 rad/s = {big}/{len(stale)} ({100*big/len(stale):.1f}%)")
        print("ВЫВОД: если stale кадры сосредоточены на больших |flow| и высокой latency,")
        print("то current 80-ms receive-time gate систематически выбрасывает именно самые")
        print("информативные интервалы движения; лечить надо latency/throughput, а не просто")
        print("поднимать stale threshold.")
if __name__=="__main__":
    main()
