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

def fmt(v,n=1):
    return "nan" if not math.isfinite(v) else f"{v:.{n}f}"

def summarize(rows):
    n=len(rows)
    if not n:
        return None
    valid=sum(ii(r,"valid")==1 for r in rows)
    sent=sum(ii(r,"flow_sent")==1 for r in rows)
    stale=sum(ii(r,"valid")==1 and ii(r,"flow_sent")!=1 for r in rows)
    invalid=n-valid
    reasons=Counter(ii(r,"invalid_reason") for r in rows if ii(r,"valid")!=1)
    inlier_ratio=[f(r,"inlier_ratio") for r in rows if ii(r,"valid")==1 and math.isfinite(f(r,"inlier_ratio"))]
    lat=[f(r,"frame_pipeline_latency_ms") for r in rows if math.isfinite(f(r,"frame_pipeline_latency_ms"))]
    tracked=[f(r,"tracked") for r in rows if math.isfinite(f(r,"tracked"))]
    inliers=[f(r,"inliers") for r in rows if math.isfinite(f(r,"inliers"))]
    return dict(n=n,valid=valid,sent=sent,stale=stale,invalid=invalid,reasons=reasons,
                ir_med=statistics.median(inlier_ratio) if inlier_ratio else float("nan"),
                lat_med=statistics.median(lat) if lat else float("nan"),
                lat_p95=pct(lat,.95) if lat else float("nan"),
                tr_med=statistics.median(tracked) if tracked else float("nan"),
                in_med=statistics.median(inliers) if inliers else float("nan"))

def print_bins(title,rows,key,bins,scale=1.0,unit=""):
    print()
    print(title)
    print("  диапазон              n    valid%   stale%  med_inlier  lat_med/p95")
    for lo,hi,label in bins:
        part=[]
        for r in rows:
            v=key(r)
            if not math.isfinite(v): continue
            v*=scale
            if v>=lo and (hi is None or v<hi):
                part.append(r)
        s=summarize(part)
        if not s:
            print(f"  {label:<20}  0")
            continue
        print(f"  {label:<20} {s['n']:4d}  {100*s['valid']/s['n']:7.1f}  {100*s['stale']/s['n']:7.1f}"
              f"     {fmt(s['ir_med'],3):>7}    {fmt(s['lat_med'])}/{fmt(s['lat_p95'])} ms")

def main():
    ap=argparse.ArgumentParser(description="Маршрут-независимый stress-profile frontend OpticalFlow")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")

    # Skip startup row with no previous image if present.
    usable=[r for r in rows if f(r,"dt_s",0)>0]
    s=summarize(usable)
    print("===== JT-ZERO — FRONTEND STRESS PROFILE =====")
    print(f"CSV: {a.csv}")
    print(f"rows={len(rows)} usable={len(usable)}")
    print(f"overall valid={s['valid']}/{s['n']} ({100*s['valid']/s['n']:.1f}%) "
          f"stale={s['stale']} ({100*s['stale']/s['n']:.1f}%)")
    names={0:"unknown/startup",1:"bad_dt",2:"few_features",3:"few_tracked",4:"homography_fail",5:"few_inliers",6:"flow_too_large"}
    if s["reasons"]:
        print("invalid reasons:")
        for k,n in sorted(s["reasons"].items()):
            print(f"  {k} {names.get(k,'unknown')}: {n}")

    print_bins(
        "ПО |FLOW| — фактическая сложность движения изображения:",
        usable,
        lambda r: math.hypot(f(r,"flow_body_x"),f(r,"flow_body_y")),
        [(0,.1,"0.00..0.10 rad/s"),(.1,.2,"0.10..0.20"),(.2,.4,"0.20..0.40"),
         (.4,.6,"0.40..0.60"),(.6,.8,"0.60..0.80"),(.8,None,">=0.80")]
    )

    print_bins(
        "ПО |GYRO XY| — нагрузка roll/pitch:",
        usable,
        lambda r: math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y")),
        [(0,.05,"0.00..0.05 rad/s"),(.05,.15,"0.05..0.15"),(.15,.30,"0.15..0.30"),
         (.30,.50,"0.30..0.50"),(.50,None,">=0.50")]
    )

    print_bins(
        "ПО |GYRO Z| — нагрузка yaw:",
        usable,
        lambda r: abs(f(r,"fc_gyro_z")),
        [(0,.05,"0.00..0.05 rad/s"),(.05,.15,"0.05..0.15"),(.15,.30,"0.15..0.30"),
         (.30,.50,"0.30..0.50"),(.50,None,">=0.50")]
    )

    print_bins(
        "ПО dt — реальный интервал между обработанными кадрами:",
        usable,
        lambda r: f(r,"dt_s"),
        [(0,50,"<50 ms"),(50,65,"50..65 ms"),(65,80,"65..80 ms"),
         (80,100,"80..100 ms"),(100,140,"100..140 ms"),(140,None,">=140 ms")],
        scale=1000.0
    )

    # Stage timing only exists in newer logs.
    if "t_lk_ms" in rows[0]:
        for field,title in [
            ("t_features_ms","goodFeaturesToTrack"),
            ("t_lk_ms","LK"),
            ("t_ransac_ms","RANSAC"),
            ("t_post_ms","post-processing"),
        ]:
            xs=[f(r,field) for r in usable if math.isfinite(f(r,field))]
            if xs:
                print(f"{title}: median/p95/max = {fmt(statistics.median(xs),2)}/{fmt(pct(xs,.95),2)}/{fmt(max(xs),2)} ms")

    # Bridge effectiveness, independent of route.
    bridge_rows=[r for r in usable if ii(r,"bridge_pending")==1] if "bridge_pending" in rows[0] else []
    if "bridge_pending" in rows[0]:
        print()
        print(f"bridge_pending rows={len(bridge_rows)}")
        print("Примечание: точное recovered/reset берётся из финальной строки терминала; CSV хранит состояние pending.")

    # Failure examples: top 12 hardest invalid/stale rows.
    hard=[r for r in usable if ii(r,"valid")!=1 or (ii(r,"valid")==1 and ii(r,"flow_sent")!=1)]
    hard.sort(key=lambda r:(f(r,"frame_pipeline_latency_ms",0), math.hypot(f(r,"flow_body_x",0),f(r,"flow_body_y",0))), reverse=True)
    print()
    print("ХУДШИЕ INVALID/STALE КАДРЫ:")
    for r in hard[:12]:
        print(f"  frame={ii(r,'frame'):4d} valid={ii(r,'valid')} reason={ii(r,'invalid_reason')} sent={ii(r,'flow_sent')} "
              f"dt={f(r,'dt_s')*1000:6.1f}ms lat={f(r,'frame_pipeline_latency_ms'):6.1f}ms "
              f"|flow|={math.hypot(f(r,'flow_body_x'),f(r,'flow_body_y')):.3f} "
              f"|gxy|={math.hypot(f(r,'fc_gyro_x'),f(r,'fc_gyro_y')):.3f} |gz|={abs(f(r,'fc_gyro_z')):.3f} "
              f"trk/inl={ii(r,'tracked')}/{ii(r,'inliers')}")

    print()
    print("ИНТЕРПРЕТАЦИЯ:")
    print("  Этот отчёт НЕ сравнивает A->B с B->H и НЕ предполагает повторяемость маршрута.")
    print("  Он оценивает вероятность отказа frontend как функцию фактической нагрузки каждого кадра.")
    print("  Поэтому результаты можно сравнивать между ручными прогонами даже при разных траекториях.")

if __name__=="__main__":
    main()
