#!/usr/bin/env python3
# Анализ guided OpticalFlow bench по CSV.
# Ничего не пишет в FC. Ищет физическое движение по raw flow magnitude,
# интегрирует raw flow*range и сравнивает с EKF LOCAL_POSITION_NED / velocity.

from __future__ import annotations
import argparse, csv, math
from pathlib import Path

def f(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--flow-threshold", type=float, default=0.03,
                    help="rad/s; порог выделения движения")
    args=ap.parse_args()
    p=Path(args.csv)
    rows=list(csv.DictReader(p.open(newline="")))
    if len(rows)<10:
        raise SystemExit("Слишком мало строк")

    mags=[math.hypot(f(r,"flow_body_x"), f(r,"flow_body_y")) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=args.flow_threshold and int(f(rows[i],"valid"))==1]
    print("===== OPTICAL FLOW GUIDED CSV ANALYSIS =====")
    print(f"rows={len(rows)} threshold={args.flow_threshold:.3f} rad/s active_rows={len(idx)}")
    if not idx:
        print("VERDICT: движение по raw flow не найдено")
        return 2

    i0=max(1,min(idx)-5)
    i1=min(len(rows)-1,max(idx)+5)
    print(f"movement envelope rows={i0}..{i1} frames={rows[i0].get('frame')}..{rows[i1].get('frame')}")

    # Интеграл raw flow. Для горизонтального сдвига и постоянной высоты
    # |v| ~= h * |flow|. Вектор оставляем в body-flow axes; для масштаба важен модуль.
    dx=dy=0.0
    path=0.0
    valid_n=0
    ranges=[]
    for i in range(i0,i1+1):
        r=rows[i]
        dt=f(r,"dt_s")
        h=f(r,"luna_m")
        if not (0 < dt < 0.2 and 0.05 < h < 20 and int(f(r,"valid"))==1):
            continue
        vx=h*f(r,"flow_body_x")
        vy=h*f(r,"flow_body_y")
        ddx=vx*dt; ddy=vy*dt
        dx+=ddx; dy+=ddy; path+=math.hypot(ddx,ddy)
        ranges.append(h); valid_n+=1

    raw_net=math.hypot(dx,dy)

    # EKF position delta over the same envelope
    fresh=[i for i in range(i0,i1+1) if int(f(rows[i],"ekf_local_valid"))==1]
    if fresh:
        a=rows[fresh[0]]; b=rows[fresh[-1]]
        dN=f(b,"ekf_x_ned")-f(a,"ekf_x_ned")
        dE=f(b,"ekf_y_ned")-f(a,"ekf_y_ned")
        ekf_pos=math.hypot(dN,dE)
    else:
        dN=dE=ekf_pos=float("nan")

    # Интеграл EKF velocity over same time envelope
    evN=evE=0.0
    last_ns=None
    ekf_v_peak=0.0
    for i in range(i0,i1+1):
        r=rows[i]
        if int(f(r,"ekf_local_valid"))!=1:
            continue
        ns=int(f(r,"mono_ns"))
        if last_ns is not None:
            dt=(ns-last_ns)*1e-9
            if 0 < dt < 0.2:
                evN += f(r,"ekf_vx_ned")*dt
                evE += f(r,"ekf_vy_ned")*dt
        last_ns=ns
        ekf_v_peak=max(ekf_v_peak, math.hypot(f(r,"ekf_vx_ned"),f(r,"ekf_vy_ned")))
    ekf_vel_int=math.hypot(evN,evE)

    peak_flow=max(mags[i0:i1+1])
    med_h=sorted(ranges)[len(ranges)//2] if ranges else float("nan")
    flags=sorted({int(f(rows[i],"ekf_flags")) for i in range(i0,i1+1) if int(f(rows[i],"ekf_status_valid"))==1})

    print(f"raw peak flow={peak_flow:.4f} rad/s median_range={med_h:.3f} m")
    print(f"RAW integral vector=({dx*1000:+.1f},{dy*1000:+.1f}) mm net={raw_net*1000:.1f} mm path={path*1000:.1f} mm")
    print(f"EKF position delta=({dN*1000:+.1f},{dE*1000:+.1f}) mm net={ekf_pos*1000:.1f} mm")
    print(f"EKF velocity integral=({evN*1000:+.1f},{evE*1000:+.1f}) mm net={ekf_vel_int*1000:.1f} mm")
    print(f"EKF peak horizontal speed={ekf_v_peak:.4f} m/s flags={flags}")
    print(f"RAW scale vs 175mm = {raw_net/0.175:.3f}x")
    print(f"EKF pos vs RAW = {(ekf_pos/raw_net if raw_net>1e-9 else float('nan')):.4f}x")

    print("\n===== INTERPRETATION =====")
    if 0.5 <= raw_net/0.175 <= 1.5 and ekf_pos < 0.2*raw_net:
        print("RAW_FLOW_SEES_MOVE=YES")
        print("EKF_LOCAL_FOLLOWS_RAW=NO")
        print("Масштаб raw OpticalFlow по порядку величины соответствует реальному движению,")
        print("но LOCAL_POSITION_NED почти не интегрирует это движение.")
    elif raw_net < 0.5*0.175:
        print("RAW_FLOW_SEES_MOVE=TOO_SMALL")
        print("Сначала разбираться с raw flow scale/axis/timing.")
    else:
        print("RESULT=REVIEW_NUMBERS")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
