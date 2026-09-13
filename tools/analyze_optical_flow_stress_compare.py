#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math
from pathlib import Path
from collections import Counter

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d
def ii(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def load(spec):
    if "=" not in spec:
        raise SystemExit(f"Ожидается LABEL=/path/file.csv, получено: {spec}")
    label,path=spec.split("=",1)
    rows=list(csv.DictReader(Path(path).open(newline="")))
    rows=[r for r in rows if f(r,"dt_s",0)>0]
    return label,Path(path),rows

def valid_pct(rows):
    if not rows:return float("nan")
    return 100.0*sum(ii(r,"valid")==1 for r in rows)/len(rows)

def stale_pct(rows):
    if not rows:return float("nan")
    return 100.0*sum(ii(r,"valid")==1 and ii(r,"flow_sent")!=1 for r in rows)/len(rows)

def filt(rows,dt_lo,dt_hi,gxy_lo=None,gxy_hi=None,gz_lo=None,gz_hi=None):
    out=[]
    for r in rows:
        dt=f(r,"dt_s")*1000.0
        gxy=math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))
        gz=abs(f(r,"fc_gyro_z"))
        if not (dt>=dt_lo and (dt_hi is None or dt<dt_hi)): continue
        if gxy_lo is not None and not (gxy>=gxy_lo and (gxy_hi is None or gxy<gxy_hi)): continue
        if gz_lo is not None and not (gz>=gz_lo and (gz_hi is None or gz<gz_hi)): continue
        out.append(r)
    return out

def fmt(x):
    return "  —  " if not math.isfinite(x) else f"{x:5.1f}%"

def main():
    ap=argparse.ArgumentParser(description="Сравнение ручных stress-run без требования повторяемого маршрута")
    ap.add_argument("--run",action="append",required=True,
                    help="LABEL=/path/to/optical_flow_mavlink.csv; повторить для каждого прогона")
    a=ap.parse_args()
    runs=[load(x) for x in a.run]

    print("===== JT-ZERO — СРАВНЕНИЕ STRESS-RUN ПО ФАКТИЧЕСКОЙ НАГРУЗКЕ =====")
    print("Маршрут и пройденное расстояние НЕ сравниваются.")
    print("Сравниваются только кадры, попавшие в одинаковые диапазоны dt/gyro.")
    print()

    for label,path,rows in runs:
        rs=Counter(ii(r,"invalid_reason") for r in rows if ii(r,"valid")!=1)
        print(f"{label}: rows={len(rows)} valid={valid_pct(rows):.1f}% stale={stale_pct(rows):.1f}% "
              f"bad_dt={rs.get(1,0)} few_features={rs.get(2,0)} few_inliers={rs.get(5,0)}")
        print(f"  {path}")

    dtbins=[(0,65,"dt<65"),(65,100,"65<=dt<100"),(100,None,"dt>=100")]
    print()
    print("1) MATCHED dt — без разделения по типу вращения")
    hdr="bin".ljust(18)+"".join(label.center(14) for label,_,__ in runs)
    print(hdr)
    for lo,hi,name in dtbins:
        line=name.ljust(18)
        for label,path,rows in runs:
            x=filt(rows,lo,hi)
            line+=(f"{fmt(valid_pct(x))} n={len(x):3d}").center(14)
        print(line)

    tests=[
        ("LOW ANGULAR", dict(gxy_lo=0,gxy_hi=.15,gz_lo=0,gz_hi=.15)),
        ("YAW LOAD", dict(gxy_lo=0,gxy_hi=None,gz_lo=.30,gz_hi=None)),
        ("TILT LOAD", dict(gxy_lo=.50,gxy_hi=None,gz_lo=0,gz_hi=None)),
    ]
    for title,kw in tests:
        print()
        print(f"2) {title} — valid% при одинаковом dt")
        print(hdr)
        for lo,hi,name in dtbins:
            line=name.ljust(18)
            for label,path,rows in runs:
                x=filt(rows,lo,hi,**kw)
                line+=(f"{fmt(valid_pct(x))} n={len(x):3d}").center(14)
            print(line)

    print()
    print("3) STALE — только валидные, но не отправленные измерения")
    for label,path,rows in runs:
        stale=[r for r in rows if ii(r,"valid")==1 and ii(r,"flow_sent")!=1]
        if stale:
            med_dt=sorted(f(r,"dt_s")*1000 for r in stale)[len(stale)//2]
            med_gz=sorted(abs(f(r,"fc_gyro_z")) for r in stale)[len(stale)//2]
            print(f"  {label}: n={len(stale)} median_dt={med_dt:.1f} ms median_|gz|={med_gz:.3f} rad/s")
        else:
            print(f"  {label}: n=0")

    print()
    print("ИНТЕРПРЕТАЦИЯ:")
    print("  Не делайте вывод по общему valid% между разными ручными прогонами.")
    print("  Смотрите только ячейки с сопоставимыми dt/gyro и достаточным n.")
    print("  Малые выборки (условно n<20) считать ориентиром, а не доказательством.")

if __name__=="__main__":
    main()
