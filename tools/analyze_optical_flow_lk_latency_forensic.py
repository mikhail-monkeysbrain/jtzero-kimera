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
    j=min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))
    return xs[j]
def corr(a,b):
    xs=[]; ys=[]
    for x,y in zip(a,b):
        if math.isfinite(x) and math.isfinite(y):
            xs.append(x); ys.append(y)
    if len(xs)<3:return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    sx=sum((x-mx)**2 for x in xs); sy=sum((y-my)**2 for y in ys)
    if sx<=0 or sy<=0:return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys))/math.sqrt(sx*sy)

def load(spec):
    if "=" not in spec:
        raise SystemExit(f"Ожидается LABEL=/path/file.csv, получено: {spec}")
    label,path=spec.split("=",1)
    p=Path(path)
    rows=list(csv.DictReader(p.open(newline="")))
    return label,p,[r for r in rows if f(r,"dt_s",0)>0]

def flow_mag(r):
    return math.hypot(f(r,"flow_body_x"),f(r,"flow_body_y"))
def pix_disp(r):
    du=f(r,"du_px"); dv=f(r,"dv_px")
    return math.hypot(du,dv)
def gyro_xy(r):
    return math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))
def gyro_z(r):
    return abs(f(r,"fc_gyro_z"))
def inlier_ratio(r):
    t=f(r,"tracked")
    if t>0:return f(r,"inliers")/t
    return float("nan")

def show_top(rows,title,key,n=20):
    rr=[r for r in rows if math.isfinite(key(r))]
    rr.sort(key=key,reverse=True)
    print()
    print(title)
    print(" frame  valid sent  dt_ms  lat_ms  LK_ms  pix_disp  |flow|  |gxy|  |gz|  trk/inl  inlier")
    for r in rr[:n]:
        tr=i(r,"tracked"); inn=i(r,"inliers")
        print(f"{i(r,'frame'):6d}  {i(r,'valid'):5d} {i(r,'flow_sent'):4d}"
              f" {f(r,'dt_s')*1000:6.1f} {f(r,'frame_pipeline_latency_ms'):7.1f}"
              f" {f(r,'t_lk_ms'):6.1f} {pix_disp(r):8.2f}"
              f" {flow_mag(r):7.3f} {gyro_xy(r):6.3f} {gyro_z(r):5.3f}"
              f" {tr:4d}/{inn:<4d} {inlier_ratio(r):6.3f}")

def binned(rows,field_name,func,bins):
    print()
    print(field_name)
    print("  диапазон                n   LK med/p95   latency med/p95   valid%  stale%")
    for lo,hi,label in bins:
        part=[]
        for r in rows:
            v=func(r)
            if not math.isfinite(v):continue
            if v>=lo and (hi is None or v<hi):part.append(r)
        if not part:
            print(f"  {label:<22} 0")
            continue
        lk=[f(r,"t_lk_ms") for r in part if math.isfinite(f(r,"t_lk_ms"))]
        lat=[f(r,"frame_pipeline_latency_ms") for r in part if math.isfinite(f(r,"frame_pipeline_latency_ms"))]
        valid=sum(i(r,"valid")==1 for r in part)
        stale=sum(i(r,"valid")==1 and i(r,"flow_sent")!=1 for r in part)
        print(f"  {label:<22} {len(part):4d}"
              f"  {statistics.median(lk):5.1f}/{pct(lk,.95):5.1f}"
              f"       {statistics.median(lat):5.1f}/{pct(lat,.95):5.1f}"
              f"      {100*valid/len(part):5.1f}   {100*stale/len(part):5.1f}")

def main():
    ap=argparse.ArgumentParser(description="JT-Zero LK/latency forensic across stress runs")
    ap.add_argument("--run",action="append",required=True,
                    help="LABEL=/path/to/optical_flow_mavlink.csv")
    ap.add_argument("--top",type=int,default=20)
    a=ap.parse_args()

    runs=[load(x) for x in a.run]
    print("===== JT-ZERO — LK / LATENCY FORENSIC =====")
    print("Не сравнивает маршруты. Ищет признаки, связанные со стоимостью LK и stale.")
    for label,path,rows in runs:
        print()
        print("="*72)
        print(label)
        print(path)
        print(f"rows={len(rows)}")

        lk=[f(r,"t_lk_ms") for r in rows]
        lat=[f(r,"frame_pipeline_latency_ms") for r in rows]
        disp=[pix_disp(r) for r in rows]
        fm=[flow_mag(r) for r in rows]
        dt=[f(r,"dt_s")*1000 for r in rows]
        tr=[f(r,"tracked") for r in rows]
        ir=[inlier_ratio(r) for r in rows]

        print("CORRELATIONS (Pearson; только ориентир, не причинность):")
        print(f"  corr(LK_ms, pixel_displacement) = {corr(lk,disp):+.3f}")
        print(f"  corr(LK_ms, |flow|)             = {corr(lk,fm):+.3f}")
        print(f"  corr(LK_ms, dt_ms)              = {corr(lk,dt):+.3f}")
        print(f"  corr(LK_ms, tracked)            = {corr(lk,tr):+.3f}")
        print(f"  corr(LK_ms, inlier_ratio)       = {corr(lk,ir):+.3f}")
        print(f"  corr(latency_ms, LK_ms)         = {corr(lat,lk):+.3f}")

        stale=[r for r in rows if i(r,"valid")==1 and i(r,"flow_sent")!=1]
        sent=[r for r in rows if i(r,"valid")==1 and i(r,"flow_sent")==1]
        if stale:
            print("STALE vs SENT (medians):")
            def med(rs,fn):
                xs=[fn(r) for r in rs if math.isfinite(fn(r))]
                return statistics.median(xs) if xs else float("nan")
            print(f"  LK_ms:      stale={med(stale,lambda r:f(r,'t_lk_ms')):.1f}  sent={med(sent,lambda r:f(r,'t_lk_ms')):.1f}")
            print(f"  latency_ms: stale={med(stale,lambda r:f(r,'frame_pipeline_latency_ms')):.1f}  sent={med(sent,lambda r:f(r,'frame_pipeline_latency_ms')):.1f}")
            print(f"  pix_disp:   stale={med(stale,pix_disp):.2f}  sent={med(sent,pix_disp):.2f}")
            print(f"  |flow|:     stale={med(stale,flow_mag):.3f}  sent={med(sent,flow_mag):.3f}")
            print(f"  dt_ms:      stale={med(stale,lambda r:f(r,'dt_s')*1000):.1f}  sent={med(sent,lambda r:f(r,'dt_s')*1000):.1f}")

        binned(rows,"ПО PIXEL DISPLACEMENT |du,dv|",pix_disp,[
            (0,5,"<5 px"),(5,10,"5..10 px"),(10,20,"10..20 px"),
            (20,40,"20..40 px"),(40,80,"40..80 px"),(80,None,">=80 px")
        ])
        binned(rows,"ПО TRACKED FEATURES",lambda r:f(r,"tracked"),[
            (0,80,"<80"),(80,160,"80..160"),(160,300,"160..300"),(300,450,"300..450"),(450,None,">=450")
        ])

        show_top(rows,"TOP ПО LK_ms",lambda r:f(r,"t_lk_ms"),a.top)
        show_top(rows,"TOP ПО PIPELINE LATENCY",lambda r:f(r,"frame_pipeline_latency_ms"),a.top)

    print()
    print("ИНТЕРПРЕТАЦИЯ:")
    print("  Сильная corr(LK_ms,pixel_displacement) + рост LK в больших pixel bins")
    print("  поддерживает гипотезу motion-dependent стоимости LK.")
    print("  Сильная corr(LK_ms,tracked) при слабой зависимости от displacement укажет скорее")
    print("  на стоимость большого числа треков.")
    print("  Сравнение stale/sent полезно только внутри одного прогона; оно не заменяет контролируемый A/B.")

if __name__=="__main__":
    main()
