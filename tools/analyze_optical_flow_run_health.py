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

def fmt_pct(n,d):
    return 100.0*n/d if d else float("nan")

def longest_run(flags):
    best=cur=0
    for x in flags:
        if x:
            cur+=1; best=max(best,cur)
        else:
            cur=0
    return best

def main():
    ap=argparse.ArgumentParser(description="Общий аудит потерь OpticalFlow/EKF/Range/Gyro")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")
    n=len(rows)

    dt=[f(r,"dt_s") for r in rows if math.isfinite(f(r,"dt_s")) and f(r,"dt_s")>0]
    valid=[i(r,"valid")==1 for r in rows]
    sent=[i(r,"flow_sent")==1 for r in rows]
    rngsent=[i(r,"range_sent")==1 for r in rows]
    ekflocal=[i(r,"ekf_local_valid")==1 for r in rows]
    ekfstat=[i(r,"ekf_status_valid")==1 for r in rows]
    armed=[i(r,"fc_armed",-1) for r in rows]

    ekf_age=[f(r,"ekf_age_ms") for r in rows if math.isfinite(f(r,"ekf_age_ms")) and f(r,"ekf_age_ms")>=0]
    stat_age=[f(r,"ekf_status_age_ms") for r in rows if math.isfinite(f(r,"ekf_status_age_ms")) and f(r,"ekf_status_age_ms")>=0]
    luna_age=[f(r,"luna_age_ms") for r in rows if math.isfinite(f(r,"luna_age_ms")) and f(r,"luna_age_ms")>=-5]
    gyro_age=[f(r,"fc_gyro_age_ms") for r in rows if math.isfinite(f(r,"fc_gyro_age_ms")) and f(r,"fc_gyro_age_ms")>=-5]
    gyro_samples=[f(r,"fc_gyro_samples") for r in rows if math.isfinite(f(r,"fc_gyro_samples"))]
    inliers=[f(r,"inliers") for r in rows if i(r,"valid")==1 and math.isfinite(f(r,"inliers"))]
    tracked=[f(r,"tracked") for r in rows if i(r,"valid")==1 and math.isfinite(f(r,"tracked"))]
    ratios=[f(r,"inlier_ratio") for r in rows if i(r,"valid")==1 and math.isfinite(f(r,"inlier_ratio"))]

    # Camera gaps from dt. Typical period estimated by median.
    med_dt=statistics.median(dt) if dt else float("nan")
    gap2=[x for x in dt if math.isfinite(med_dt) and x>2.0*med_dt]
    gap3=[x for x in dt if math.isfinite(med_dt) and x>3.0*med_dt]
    gap50=[x for x in dt if x>0.050]
    gap100=[x for x in dt if x>0.100]

    # Pipeline latency.
    lat=[f(r,"frame_pipeline_latency_ms") for r in rows if math.isfinite(f(r,"frame_pipeline_latency_ms")) and f(r,"frame_pipeline_latency_ms")>=0]

    # EKF discontinuities / stale windows.
    stale_local=[x for x in ekf_age if x>100]
    stale_local_250=[x for x in ekf_age if x>250]
    stale_stat=[x for x in stat_age if x>500]
    stale_luna=[x for x in luna_age if x>100]
    stale_gyro=[x for x in gyro_age if x>20]

    print("===== ОБЩИЙ АУДИТ ПОТЕРЬ OPTICAL FLOW / EKF / RANGE / GYRO =====")
    print(f"строк/кадров CSV = {n}")
    print()
    print("КАМЕРА / FLOW:")
    print(f"  valid = {sum(valid)}/{n} ({fmt_pct(sum(valid),n):.2f}%)")
    print(f"  invalid = {n-sum(valid)}/{n} ({fmt_pct(n-sum(valid),n):.2f}%)")
    print(f"  flow_sent = {sum(sent)}/{n} ({fmt_pct(sum(sent),n):.2f}%)")
    print(f"  valid, но не отправлен = {sum(1 for v,s in zip(valid,sent) if v and not s)}")
    print(f"  dt median/p95/max = {med_dt*1000:.2f}/{pct(dt,.95)*1000:.2f}/{max(dt)*1000:.2f} ms" if dt else "  dt: нет данных")
    if dt:
        print(f"  gaps >2x median = {len(gap2)}; >3x median = {len(gap3)}; >50 ms = {len(gap50)}; >100 ms = {len(gap100)}")
    if lat:
        print(f"  pipeline latency median/p95/max = {statistics.median(lat):.2f}/{pct(lat,.95):.2f}/{max(lat):.2f} ms")
    if inliers:
        print(f"  inliers median/p05/min = {statistics.median(inliers):.1f}/{pct(inliers,.05):.1f}/{min(inliers):.1f}")
    if tracked:
        print(f"  tracked median/p05/min = {statistics.median(tracked):.1f}/{pct(tracked,.05):.1f}/{min(tracked):.1f}")
    if ratios:
        print(f"  inlier_ratio median/p05/min = {statistics.median(ratios):.3f}/{pct(ratios,.05):.3f}/{min(ratios):.3f}")
    print()

    print("EKF LOCAL_POSITION_NED:")
    print(f"  valid = {sum(ekflocal)}/{n} ({fmt_pct(sum(ekflocal),n):.2f}%)")
    print(f"  invalid rows = {n-sum(ekflocal)}")
    if ekf_age:
        print(f"  age median/p95/max = {statistics.median(ekf_age):.2f}/{pct(ekf_age,.95):.2f}/{max(ekf_age):.2f} ms")
        print(f"  age >100 ms = {len(stale_local)}; >250 ms = {len(stale_local_250)}")
    print(f"  longest invalid run = {longest_run([not x for x in ekflocal])} frames")
    print()

    print("EKF_STATUS_REPORT:")
    print(f"  valid = {sum(ekfstat)}/{n} ({fmt_pct(sum(ekfstat),n):.2f}%)")
    print(f"  invalid rows = {n-sum(ekfstat)}")
    if stat_age:
        print(f"  age median/p95/max = {statistics.median(stat_age):.2f}/{pct(stat_age,.95):.2f}/{max(stat_age):.2f} ms")
        print(f"  age >500 ms = {len(stale_stat)}")
    print()

    print("TF-LUNA / DISTANCE_SENSOR:")
    print(f"  range_sent rows = {sum(rngsent)}/{n} ({fmt_pct(sum(rngsent),n):.2f}%)")
    if luna_age:
        print(f"  Luna age median/p95/max = {statistics.median(luna_age):.2f}/{pct(luna_age,.95):.2f}/{max(luna_age):.2f} ms")
        print(f"  Luna age >100 ms = {len(stale_luna)}")
    print()

    print("FC GYRO / ATTITUDE:")
    if gyro_age:
        print(f"  gyro age median/p95/max = {statistics.median(gyro_age):.2f}/{pct(gyro_age,.95):.2f}/{max(gyro_age):.2f} ms")
        print(f"  gyro age >20 ms = {len(stale_gyro)}")
    if gyro_samples:
        print(f"  gyro samples/frame median/p05/min = {statistics.median(gyro_samples):.1f}/{pct(gyro_samples,.05):.1f}/{min(gyro_samples):.1f}")
        print(f"  frames with 0 gyro samples = {sum(1 for x in gyro_samples if x<=0)}")
    print()

    print("ARM:")
    known=[x for x in armed if x in (0,1)]
    if known:
        print(f"  ARMED rows = {sum(1 for x in known if x==1)}/{len(known)} ({fmt_pct(sum(1 for x in known if x==1),len(known)):.2f}%)")
    else:
        print("  ARM state: нет данных")

    # Conservative verdict.
    severe=[]
    if n-sum(valid) > max(5,0.01*n): severe.append("много invalid optical-flow кадров")
    if len(gap3) > max(3,0.01*len(dt)): severe.append("есть заметные camera/dt gaps")
    if n-sum(ekflocal) > max(5,0.01*n): severe.append("есть заметные провалы LOCAL_POSITION_NED")
    if ekf_age and len(stale_local_250)>0: severe.append("есть stale LOCAL_POSITION_NED >250 ms")
    if gyro_age and len(stale_gyro)>max(5,0.01*n): severe.append("есть заметные задержки gyro")
    print()
    if severe:
        print("ИТОГ: ОБНАРУЖЕНЫ ПОТЕРИ/ЗАДЕРЖКИ:")
        for x in severe: print("  - "+x)
    else:
        print("ИТОГ: крупных потерь по transport/validity не видно; если ошибка огромная, вероятнее проблема модели/синхронизации/компенсации, а не массовый dropout.")

if __name__=="__main__":
    main()
