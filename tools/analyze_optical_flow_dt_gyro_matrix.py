#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def i(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def bucket(v,bins):
    for idx,(lo,hi,label) in enumerate(bins):
        if v>=lo and (hi is None or v<hi):
            return idx
    return None

def summarize(cells):
    out=[]
    for row in cells:
        rr=[]
        for cell in row:
            n=len(cell)
            if n==0:
                rr.append((0,float("nan"),0,0))
                continue
            valid=sum(i(r,"valid")==1 for r in cell)
            stale=sum(i(r,"valid")==1 and i(r,"flow_sent")!=1 for r in cell)
            rr.append((n,100*valid/n,valid,stale))
        out.append(rr)
    return out

def print_matrix(title,rows,dt_bins,g_bins,gkey):
    cells=[[[] for _ in g_bins] for __ in dt_bins]
    for r in rows:
        dt=f(r,"dt_s")*1000.0
        gv=gkey(r)
        if not (math.isfinite(dt) and math.isfinite(gv)): continue
        a=bucket(dt,dt_bins); b=bucket(gv,g_bins)
        if a is not None and b is not None:
            cells[a][b].append(r)
    s=summarize(cells)
    print()
    print(title)
    print("valid% (n)")
    hdr="dt \\ gyro".ljust(16)+"".join(label.center(18) for _,_,label in g_bins)
    print(hdr)
    for ri,(_,_,dlabel) in enumerate(dt_bins):
        line=dlabel.ljust(16)
        for ci in range(len(g_bins)):
            n,p,_,_=s[ri][ci]
            cell="—" if n==0 else f"{p:5.1f}% ({n})"
            line+=cell.center(18)
        print(line)

    print()
    print("invalid counts by cell")
    print(hdr)
    for ri,(_,_,dlabel) in enumerate(dt_bins):
        line=dlabel.ljust(16)
        for ci in range(len(g_bins)):
            n,p,v,st=s[ri][ci]
            cell="—" if n==0 else f"{n-v:3d} inv / {st:2d} st"
            line+=cell.center(18)
        print(line)

def main():
    ap=argparse.ArgumentParser(description="2D dt×gyro failure matrix for JT-Zero OpticalFlow")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")

    usable=[r for r in rows if f(r,"dt_s",0)>0]
    dt_bins=[
        (0,65,"<65 ms"),
        (65,100,"65..100"),
        (100,140,"100..140"),
        (140,None,">=140"),
    ]
    gxy_bins=[
        (0,.15,"<0.15"),
        (.15,.50,"0.15..0.50"),
        (.50,None,">=0.50"),
    ]
    gz_bins=[
        (0,.15,"<0.15"),
        (.15,.30,"0.15..0.30"),
        (.30,None,">=0.30"),
    ]

    print("===== JT-ZERO — 2D FAILURE MATRIX =====")
    print(f"CSV: {a.csv}")
    print(f"usable rows={len(usable)}")

    print_matrix(
        "dt × |gyro XY| — roll/pitch angular load",
        usable,dt_bins,gxy_bins,
        lambda r: math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))
    )
    print_matrix(
        "dt × |gyro Z| — yaw angular load",
        usable,dt_bins,gz_bins,
        lambda r: abs(f(r,"fc_gyro_z"))
    )

    # Independent diagnostic: long dt alone at low angular rate.
    lowang=[r for r in usable if math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))<.15 and abs(f(r,"fc_gyro_z"))<.15]
    longdt=[r for r in lowang if f(r,"dt_s")*1000>=100]
    highang=[r for r in usable if math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))>=.5 or abs(f(r,"fc_gyro_z"))>=.3]
    shortdt=[r for r in highang if f(r,"dt_s")*1000<100]

    def vr(rs):
        if not rs:return (0,float("nan"))
        return len(rs),100*sum(i(r,"valid")==1 for r in rs)/len(rs)

    n1,p1=vr(longdt); n2,p2=vr(shortdt)
    print()
    print("КОНТРОЛЬНЫЕ СРЕЗЫ:")
    print(f"  low angular + dt>=100 ms: n={n1}, valid={p1:.1f}%" if n1 else "  low angular + dt>=100 ms: n=0")
    print(f"  high angular + dt<100 ms: n={n2}, valid={p2:.1f}%" if n2 else "  high angular + dt<100 ms: n=0")

    print()
    print("ИНТЕРПРЕТАЦИЯ:")
    print("  1) Если long-dt ячейки плохие даже при низком gyro — проблема прежде всего в межкадровом gap.")
    print("  2) Если high-gyro ячейки плохие даже при dt<100 ms — tracker ограничен угловым motion сам по себе.")
    print("  3) Если провал возникает в основном только при сочетании long dt + high gyro — главный лимит совместный:")
    print("     processing/capture cadence × motion robustness.")
    print("  4) Маршрут A/B здесь не используется.")

if __name__=="__main__":
    main()
