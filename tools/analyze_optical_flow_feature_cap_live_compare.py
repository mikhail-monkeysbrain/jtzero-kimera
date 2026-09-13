#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path
from collections import Counter

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d
def ii(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d
def pct(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    j=min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))
    return xs[j]
def med(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return statistics.median(xs) if xs else float("nan")
def load(label,path):
    p=Path(path)
    rows=list(csv.DictReader(p.open(newline="")))
    rows=[r for r in rows if f(r,"dt_s",0)>0]
    return label,p,rows

def pix(r):
    return math.hypot(f(r,"du_px"),f(r,"dv_px"))
def gxy(r):
    return math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))
def gz(r):
    return abs(f(r,"fc_gyro_z"))

def stats(rows):
    n=len(rows)
    if not n:return None
    valid=[r for r in rows if ii(r,"valid")==1]
    stale=[r for r in valid if ii(r,"flow_sent")!=1]
    inv=[r for r in rows if ii(r,"valid")!=1]
    rs=Counter(ii(r,"invalid_reason") for r in inv)
    return {
        "n":n,"valid":len(valid),"stale":len(stale),"invalid":len(inv),"reasons":rs,
        "lk_med":med([f(r,"t_lk_ms") for r in rows]),
        "lk_p95":pct([f(r,"t_lk_ms") for r in rows],.95),
        "lat_med":med([f(r,"frame_pipeline_latency_ms") for r in rows]),
        "lat_p95":pct([f(r,"frame_pipeline_latency_ms") for r in rows],.95),
        "trk_med":med([f(r,"tracked") for r in rows]),
        "inl_med":med([f(r,"inliers") for r in rows]),
        "pix_med":med([pix(r) for r in valid]),
    }

def print_row(name,s):
    if not s:
        print(f"{name:<24} n=0");return
    print(f"{name:<24} n={s['n']:4d} valid={100*s['valid']/s['n']:5.1f}% "
          f"stale={100*s['stale']/s['n']:5.1f}% "
          f"LK={s['lk_med']:5.1f}/{s['lk_p95']:5.1f}ms "
          f"lat={s['lat_med']:5.1f}/{s['lat_p95']:5.1f}ms "
          f"trk/inl={s['trk_med']:.0f}/{s['inl_med']:.0f}")

def by(rows,pred):
    return [r for r in rows if pred(r)]

def main():
    ap=argparse.ArgumentParser(description="Matched-load comparison of live feature-cap runs")
    ap.add_argument("--a",required=True,help="LABEL=/path/csv")
    ap.add_argument("--b",required=True,help="LABEL=/path/csv")
    a=ap.parse_args()
    def parse(spec):
        if "=" not in spec: raise SystemExit("Нужно LABEL=/path/csv")
        l,p=spec.split("=",1);return load(l,p)
    runs=[parse(a.a),parse(a.b)]

    print("===== JT-ZERO — LIVE FEATURE-CAP MATCHED LOAD COMPARE =====")
    print("Не сравнивает маршруты или общий valid% как доказательство.")
    print("Сравнение только в одинаковых диапазонах фактической нагрузки.\n")
    for label,path,rows in runs:
        s=stats(rows)
        print(label, path)
        print_row("ALL",s)
        print("  invalid reasons:",dict(sorted(s["reasons"].items())))
        print()

    bins_dt=[(0,65,"dt<65"),(65,100,"65<=dt<100"),(100,None,"dt>=100")]
    bins_g=[(0,.15,"gyro<0.15"),(.15,.5,"0.15<=gyro<0.5"),(.5,None,"gyro>=0.5")]
    bins_pix=[(0,5,"pix<5"),(5,20,"5<=pix<20"),(20,40,"20<=pix<40"),(40,None,"pix>=40")]

    for title,bins,fn in [
        ("ПО dt (ms)",bins_dt,lambda r:f(r,"dt_s")*1000),
        ("ПО |gyro XY| (rad/s)",bins_g,gxy),
        ("ПО |gyro Z| (rad/s)",bins_g,gz),
        ("ПО measured pixel displacement (только valid)",bins_pix,pix),
    ]:
        print("\n"+title)
        for lo,hi,name in bins:
            print("  "+name)
            for label,path,rows in runs:
                subset=[]
                for r in rows:
                    v=fn(r)
                    if not math.isfinite(v):continue
                    if title.startswith("ПО measured") and ii(r,"valid")!=1: continue
                    if v>=lo and (hi is None or v<hi):subset.append(r)
                print_row("    "+label,stats(subset))

    print("\n2D MATCHED: dt × |gyro XY|")
    for dlo,dhi,dname in bins_dt:
      for glo,ghi,gname in bins_g:
        print(f"  {dname}, {gname}")
        for label,path,rows in runs:
          sub=by(rows,lambda r,dlo=dlo,dhi=dhi,glo=glo,ghi=ghi:
                 (f(r,"dt_s")*1000>=dlo and (dhi is None or f(r,"dt_s")*1000<dhi) and
                  gxy(r)>=glo and (ghi is None or gxy(r)<ghi)))
          print_row("    "+label,stats(sub))

    print("\n2D MATCHED: dt × |gyro Z|")
    for dlo,dhi,dname in bins_dt:
      for glo,ghi,gname in bins_g:
        print(f"  {dname}, {gname}")
        for label,path,rows in runs:
          sub=by(rows,lambda r,dlo=dlo,dhi=dhi,glo=glo,ghi=ghi:
                 (f(r,"dt_s")*1000>=dlo and (dhi is None or f(r,"dt_s")*1000<dhi) and
                  gz(r)>=glo and (ghi is None or gz(r)<ghi)))
          print_row("    "+label,stats(sub))

    print("\nИНТЕРПРЕТАЦИЯ:")
    print("  - Общий invalid% между ручными прогонами не считать доказательством.")
    print("  - Смотрите только клетки, где у ОБОИХ вариантов достаточно кадров.")
    print("  - n<20 — только ориентир.")
    print("  - Если cap=200 стабильно снижает LK/latency в matched bins без ухудшения valid/stale,")
    print("    это основание для дальнейшего теста, но не окончательное доказательство flight-качества.")

if __name__=="__main__":
    main()
