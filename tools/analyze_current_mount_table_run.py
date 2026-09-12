#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r[k])
    except:return d

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("--physical-mm",type=float,default=None)
    args=ap.parse_args()

    with args.csv.open(newline="") as fh:
        rows=list(csv.DictReader(fh))

    move=[r for r in rows if int(f(r,"guide_stage",-1))==1]
    if len(move)<2:
        raise SystemExit("guide_stage=1 не найден")

    # Integrate actual transmitted optical-flow angular rates against the actual
    # range sent to FC. This is a diagnostic RAW metric only; ArduPilot's EKF
    # applies its own attitude/range/lever-arm model.
    dx=dy=0.0
    prev_t=None
    valid=0
    ranges=[]
    mags=[]
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        t=f(r,"mono_ns")*1e-9
        rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[t,rng,fx,fy])) or rng<=0:
            continue
        if prev_t is not None:
            dt=t-prev_t
            if 0<dt<0.2:
                dx += fx*rng*dt
                dy += fy*rng*dt
        prev_t=t
        valid+=1
        ranges.append(rng)
        mags.append(math.hypot(fx,fy))

    raw_mm=1000*math.hypot(dx,dy)

    # EKF LOCAL displacement over the same stage.
    loc=[r for r in move if int(f(r,"ekf_local_valid",0))==1]
    ekf_mm=float("nan")
    dn=de=float("nan")
    if len(loc)>=2:
        dn=f(loc[-1],"ekf_x_ned")-f(loc[0],"ekf_x_ned")
        de=f(loc[-1],"ekf_y_ned")-f(loc[0],"ekf_y_ned")
        ekf_mm=1000*math.hypot(dn,de)

    print("===== CURRENT-MOUNT TABLE RUN FORENSIC =====")
    print(f"rows move={len(move)} valid_tx={valid}")
    print(f"range median/min/max = {statistics.median(ranges):.3f}/{min(ranges):.3f}/{max(ranges):.3f} m")
    print(f"RAW transmitted-flow integral = {raw_mm:.1f} mm")
    print(f"  components (body-rate proxy) = X {dx*1000:+.1f} mm, Y {dy*1000:+.1f} mm")
    print(f"EKF LOCAL displacement = {ekf_mm:.1f} mm")
    print(f"  N/E = {dn*1000:+.1f}/{de*1000:+.1f} mm")
    if math.isfinite(ekf_mm) and raw_mm>1e-9:
        print(f"EKF/RAW = {ekf_mm/raw_mm:.4f}")
    if args.physical_mm is not None and args.physical_mm>0:
        print()
        print(f"PHYSICAL = {args.physical_mm:.1f} mm")
        print(f"RAW/PHYS = {raw_mm/args.physical_mm:.4f}")
        print(f"EKF/PHYS = {ekf_mm/args.physical_mm:.4f}")
    else:
        print()
        print("PHYSICAL не задан. Не используем nominal 300 мм как ground truth.")
        print("Повтори команду с --physical-mm <реально измеренное расстояние>.")

if __name__=="__main__":main()
