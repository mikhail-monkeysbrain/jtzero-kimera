#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path
from collections import Counter

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

def longest(flags):
    best=cur=0
    for x in flags:
        if x: cur+=1; best=max(best,cur)
        else: cur=0
    return best

def main():
    ap=argparse.ArgumentParser(description="Сводка причин invalid OpticalFlow и их влияния на loop closure")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")

    has_reason="invalid_reason" in rows[0]
    valid=[i(r,"valid")==1 for r in rows]
    sent=[i(r,"flow_sent")==1 for r in rows]
    dt=[f(r,"dt_s") for r in rows if 0<f(r,"dt_s")<1]
    inv=sum(not x for x in valid)
    print("===== INVALID FLOW FORENSIC =====")
    print(f"rows={len(rows)} valid={sum(valid)} invalid={inv} invalid_rate={100*inv/len(rows):.1f}%")
    print(f"flow_sent={sum(sent)} longest_invalid_run={longest([not x for x in valid])} frames")
    if dt:
        print(f"dt median/p95/max={statistics.median(dt)*1000:.1f}/{pct(dt,.95)*1000:.1f}/{max(dt)*1000:.1f} ms")

    if has_reason:
        c=Counter(i(r,"invalid_reason") for r in rows if i(r,"valid")!=1)
        names={1:"bad_dt",2:"few_features",3:"few_tracked",4:"homography_fail",5:"few_inliers",6:"flow_too_large"}
        print("invalid reasons:")
        for k,n in sorted(c.items()):
            print(f"  {k} {names.get(k,'unknown')}: {n}")
    else:
        print("invalid_reason: НЕТ В ЭТОМ CSV (нужен запуск после commit 03fc68b)")

    ev=[(j,i(r,"return_event")) for j,r in enumerate(rows) if i(r,"return_event") in (1,2,3)]
    print("events:",ev)
    if len(ev)>=3:
        ia=next((j for j,e in ev if e==1),None)
        ib=next((j for j,e in ev if e==2 and ia is not None and j>ia),None)
        ih=next((j for j,e in ev if e==3 and ib is not None and j>ib),None)
        if ia is not None and ib is not None and ih is not None:
            for name,lo,hi in [("A->B",ia,ib),("B->H",ib,ih)]:
                seg=rows[lo:hi+1]
                bad=sum(i(r,"valid")!=1 for r in seg)
                print(f"{name}: rows={len(seg)} invalid={bad} ({100*bad/len(seg):.1f}%) longest={longest([i(r,'valid')!=1 for r in seg])}")
                if has_reason:
                    cc=Counter(i(r,"invalid_reason") for r in seg if i(r,"valid")!=1)
                    print("  reasons:",dict(sorted(cc.items())))

    # Check whether EKF and RAW closure are tracking each other at H if available.
    hrows=[r for r in rows if i(r,"return_event")==3]
    if hrows:
        h=hrows[-1]
        # fields aren't explicit RAW state in CSV, so only announce that terminal closure must be used.
        print("H event found; compare terminal EKF closure vs RAW NED closure from launcher output.")

    print()
    if inv/len(rows)>0.10:
        print("ВЫВОД: invalid-rate >10% — frontend dropout ещё достаточно велик, чтобы терять displacement.")
    elif inv/len(rows)>0.03:
        print("ВЫВОД: invalid-rate заметен; проверьте причины и длину серий dropout.")
    else:
        print("ВЫВОД: массового dropout нет; искать систематическую модельную ошибку.")

if __name__=="__main__":
    main()
