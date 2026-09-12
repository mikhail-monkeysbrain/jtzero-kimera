#!/usr/bin/env python3
# JT-Zero — timing forensic: raw metric speed vs EKF speed and camera->send latency.
# Работает только по уже записанным CSV. Ничего не пишет в FC.

from __future__ import annotations
import argparse,csv,json,math,statistics,sys
from pathlib import Path

FLOW_THRESHOLD=0.03

def fv(r,k,d=float("nan")):
    try:
        v=r.get(k,"")
        return float(v) if v not in ("",None) else d
    except Exception:
        return d

def pct(vals,q):
    if not vals: return float("nan")
    s=sorted(vals); x=(len(s)-1)*q; i=int(math.floor(x)); j=min(i+1,len(s)-1); a=x-i
    return s[i]*(1-a)+s[j]*a

def pearson(a,b):
    if len(a)<3 or len(a)!=len(b): return float("nan")
    ma=statistics.mean(a); mb=statistics.mean(b)
    da=[x-ma for x in a]; db=[y-mb for y in b]
    va=sum(x*x for x in da); vb=sum(y*y for y in db)
    if va<=0 or vb<=0: return float("nan")
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)

def analyze(path:Path):
    rows=list(csv.DictReader(path.open(newline="")))
    if not rows: raise RuntimeError("CSV пуст")
    req=("flow_send_x","flow_send_y","range_to_fc_m","ekf_vx_ned","ekf_vy_ned",
         "ekf_local_valid","frame_pipeline_latency_ms","mono_ns","valid")
    for k in req:
        if k not in rows[0]: raise RuntimeError(f"нет колонки {k}")

    # movement envelope by raw angular flow
    mags=[math.hypot(fv(r,"flow_body_x",0),fv(r,"flow_body_y",0)) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=FLOW_THRESHOLD and int(fv(rows[i],"valid",0))==1]
    if not idx: raise RuntimeError("movement не найден")
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)

    t=[]; raw=[]; ekf=[]; lat=[]
    for i in range(i0,i1+1):
        r=rows[i]
        if int(fv(r,"valid",0))!=1 or int(fv(r,"ekf_local_valid",0))!=1: continue
        ts=fv(r,"mono_ns")
        h=fv(r,"range_to_fc_m")
        if not (math.isfinite(ts) and 0.05<h<20): continue
        raw_speed=h*math.hypot(fv(r,"flow_send_x",0),fv(r,"flow_send_y",0))
        ekf_speed=math.hypot(fv(r,"ekf_vx_ned",0),fv(r,"ekf_vy_ned",0))
        l=fv(r,"frame_pipeline_latency_ms")
        if not (math.isfinite(raw_speed) and math.isfinite(ekf_speed) and math.isfinite(l)): continue
        t.append(ts*1e-9); raw.append(raw_speed); ekf.append(ekf_speed); lat.append(l)

    if len(t)<20: raise RuntimeError("слишком мало совместных samples")

    # Approx sample interval.
    dts=[t[i]-t[i-1] for i in range(1,len(t)) if 0<t[i]-t[i-1]<0.2]
    dt=statistics.median(dts) if dts else 0.024

    # Cross-correlation by integer shifts: positive shift means EKF is delayed vs raw.
    corr=[]
    max_ms=400
    max_k=max(1,int(round((max_ms/1000)/dt)))
    for k in range(-max_k,max_k+1):
        if k>=0:
            a=raw[:len(raw)-k or None]
            b=ekf[k:]
        else:
            kk=-k
            a=raw[kk:]
            b=ekf[:len(ekf)-kk]
        if len(a)<20: continue
        corr.append((pearson(a,b),k))
    corr=[x for x in corr if math.isfinite(x[0])]
    best=max(corr,key=lambda x:x[0]) if corr else (float("nan"),0)
    best_ms=best[1]*dt*1000.0

    # Correlation between latency and instantaneous speed mismatch.
    ratio=[]; lat2=[]
    diff=[]; lat3=[]
    for r,e,l in zip(raw,ekf,lat):
        if r>0.02:
            ratio.append(e/r); lat2.append(l)
            diff.append(e-r); lat3.append(l)

    # Split latency into low/high halves to see if mismatch changes.
    medlat=statistics.median(lat)
    low=[(r,e) for r,e,l in zip(raw,ekf,lat) if l<=medlat and r>0.02]
    high=[(r,e) for r,e,l in zip(raw,ekf,lat) if l>medlat and r>0.02]
    def mean_ratio(pairs):
        vals=[e/r for r,e in pairs if r>0.02]
        return statistics.mean(vals) if vals else float("nan")

    return {
        "samples":len(t),
        "dt_ms":dt*1000.0,
        "lat_p50":pct(lat,.5),"lat_p95":pct(lat,.95),"lat_max":max(lat),
        "best_corr":best[0],"best_shift_ms":best_ms,
        "lat_ratio_corr":pearson(lat2,ratio),
        "lat_diff_corr":pearson(lat3,diff),
        "low_latency_ratio":mean_ratio(low),
        "high_latency_ratio":mean_ratio(high),
        "median_latency":medlat,
        "raw_peak":max(raw),"ekf_peak":max(ekf),
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("session_json",nargs="?")
    args=ap.parse_args()
    if args.session_json:
        sp=Path(args.session_json)
    else:
        cand=sorted(Path("/home/vio/jtzero_runs").glob("*_OPTICAL_FLOW_GUI_SERIES.json"),
                    key=lambda p:p.stat().st_mtime,reverse=True)
        if not cand:
            print("ОШИБКА: session JSON не найден",file=sys.stderr); return 2
        sp=cand[0]

    runs=json.loads(sp.read_text(encoding="utf-8"))
    print("="*100)
    print("JT-ZERO — RAW vs EKF SPEED TIMING FORENSIC")
    print("="*100)
    print(f"SESSION: {sp}")
    print()
    print(" # DIR  PHYS  EKF/RAW  LAT50  LAT95  BEST_SHIFT  CORR   LAT↔RATIO  LOW/HIGH")
    print("-"*100)

    for r in runs:
        d=analyze(Path(r["csv"]))
        er=float(r["ekf_mm"])/float(r["raw_mm"])
        print(f"{int(r['run']):2d} {(r.get('direction') or '?'):4s} {float(r['physical_measured_mm']):5.0f} "
              f"{er:7.3f} {d['lat_p50']:6.1f} {d['lat_p95']:6.1f} "
              f"{d['best_shift_ms']:9.1f}ms {d['best_corr']:6.3f} "
              f"{d['lat_ratio_corr']:9.3f} "
              f"{d['low_latency_ratio']:.3f}/{d['high_latency_ratio']:.3f}")
        print(f"    samples={d['samples']} dt≈{d['dt_ms']:.1f}ms peak raw/ekf={d['raw_peak']:.3f}/{d['ekf_peak']:.3f}m/s "
              f"lat max={d['lat_max']:.1f}ms diff-corr={d['lat_diff_corr']:.3f}")

    print("\nINTERPRETATION:")
    print("BEST_SHIFT > 0 означает: профиль EKF speed лучше всего совпадает с RAW, если EKF сдвинуть позже.")
    print("LAT↔RATIO > 0 означает: при большей camera->send latency отношение EKF_speed/RAW_speed имеет тенденцию расти.")
    print("LOW/HIGH — среднее EKF_speed/RAW_speed для нижней и верхней половины latency.")
    print("Это корреляционный анализ уже записанных данных; он не доказывает причинность сам по себе.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
