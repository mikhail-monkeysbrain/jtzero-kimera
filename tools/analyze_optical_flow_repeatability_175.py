#!/usr/bin/env python3
# Сводка повторяемости guided 175 мм по нескольким optical_flow_mavlink.csv.
# Ничего не пишет в FC.

from __future__ import annotations
import argparse, csv, math, statistics
from pathlib import Path

TARGET=0.175

def f(r,k,d=0.0):
    try: return float(r.get(k,d) or d)
    except Exception: return d

def one(path: Path, threshold: float, target_m: float):
    rows=list(csv.DictReader(path.open(newline="")))
    mags=[math.hypot(f(r,"flow_body_x"),f(r,"flow_body_y")) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=threshold and int(f(rows[i],"valid"))==1]
    if not idx: return {"path":str(path),"ok":False,"reason":"no movement"}
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)

    dx=dy=0.0
    fdx=fdy=0.0
    have_fc=("range_to_fc_m" in rows[0] and "flow_send_x" in rows[0] and "flow_send_y" in rows[0])
    for i in range(i0,i1+1):
        r=rows[i]; dt=f(r,"dt_s")
        if not (0<dt<0.2 and int(f(r,"valid"))==1): continue
        h=f(r,"luna_m")
        if 0.05<h<20:
            dx += h*f(r,"flow_body_x")*dt
            dy += h*f(r,"flow_body_y")*dt
        if have_fc:
            hf=f(r,"range_to_fc_m")
            if 0.05<hf<20:
                fdx += hf*f(r,"flow_send_x")*dt
                fdy += hf*f(r,"flow_send_y")*dt

    raw=math.hypot(dx,dy)
    fcp=math.hypot(fdx,fdy) if have_fc else float("nan")

    fresh=[i for i in range(i0,i1+1) if int(f(rows[i],"ekf_local_valid"))==1]
    if not fresh: return {"path":str(path),"ok":False,"reason":"no EKF local"}
    a=rows[fresh[0]]; b=rows[fresh[-1]]
    dn=f(b,"ekf_x_ned")-f(a,"ekf_x_ned")
    de=f(b,"ekf_y_ned")-f(a,"ekf_y_ned")
    ekf=math.hypot(dn,de)

    return {
        "path":str(path),"ok":True,"raw":raw,"fc":fcp,"ekf":ekf,
        "raw_scale":raw/target_m,"ekf_scale":ekf/target_m,
        "ekf_vs_raw": ekf/raw if raw>1e-9 else float("nan")
    }

def stats(vals):
    return statistics.mean(vals), statistics.pstdev(vals), min(vals), max(vals)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",nargs="+")
    ap.add_argument("--flow-threshold",type=float,default=0.03)
    ap.add_argument("--target-mm",type=float,default=175.0)
    args=ap.parse_args()
    if not (50.0 <= args.target_mm <= 1000.0):
        raise SystemExit("ОШИБКА: --target-mm должен быть 50..1000")
    target_m=args.target_mm*0.001
    rr=[one(Path(p),args.flow_threshold,target_m) for p in args.csv]

    print("="*78)
    print(f"JT-ZERO — {args.target_mm:g} мм OPTICAL FLOW REPEATABILITY")
    print("="*78)
    good=[]
    for n,r in enumerate(rr,1):
        if not r["ok"]:
            print(f"RUN {n}: FAIL {r['reason']}  {r['path']}")
            continue
        good.append(r)
        print(f"RUN {n}: RAW={r['raw']*1000:7.2f} mm  FC={r['fc']*1000:7.2f} mm  "
              f"EKF={r['ekf']*1000:7.2f} mm  EKF/RAW={r['ekf_vs_raw']:.4f}  "
              f"RAWerr={(r['raw']/target_m-1)*100:+6.2f}%  EKFerr={(r['ekf']/target_m-1)*100:+6.2f}%")
        print(f"       {r['path']}")

    if len(good)<2:
        print("\nНедостаточно успешных прогонов для статистики.")
        return 2

    print("\n===== SUMMARY =====")
    for label,key in [("RAW","raw"),("FC-PRESENTED","fc"),("EKF","ekf")]:
        vals=[r[key]*1000 for r in good if math.isfinite(r[key])]
        m,sd,mn,mx=stats(vals)
        cv=100*sd/m if m else float("nan")
        print(f"{label:13s}: mean={m:7.2f} mm  sd={sd:6.2f} mm  CV={cv:5.2f}%  min={mn:7.2f}  max={mx:7.2f}")
    vals=[r["ekf_vs_raw"] for r in good]
    m,sd,mn,mx=stats(vals)
    print(f"EKF/RAW      : mean={m:.4f}  sd={sd:.4f}  min={mn:.4f}  max={mx:.4f}")
    rawm=statistics.mean([r["raw"] for r in good])
    ekfm=statistics.mean([r["ekf"] for r in good])
    print(f"RAW mean error vs {args.target_mm:g} mm = {(rawm/target_m-1)*100:+.2f}%")
    print(f"EKF mean error vs {args.target_mm:g} mm = {(ekfm/target_m-1)*100:+.2f}%")

    print("\n===== VERDICT =====")
    raw_cv=statistics.pstdev([r["raw"] for r in good])/rawm*100
    ekf_cv=statistics.pstdev([r["ekf"] for r in good])/ekfm*100
    if raw_cv<=3.0:
        print("RAW_REPEATABILITY=GOOD")
    else:
        print("RAW_REPEATABILITY=VARIABLE")
    if ekf_cv<=5.0:
        print("EKF_REPEATABILITY=GOOD")
    else:
        print("EKF_REPEATABILITY=VARIABLE")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
