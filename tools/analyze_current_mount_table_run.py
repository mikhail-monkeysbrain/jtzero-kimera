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

    # Robust table-only counterfactual. In this experiment TF-Luna can see the
    # floor beyond the table edge. The dominant low-range cluster is the table;
    # values far above its lower quantiles are classified as off-table and are
    # NOT valid camera-height measurements for a table translation test.
    rsorted=sorted(ranges)
    q20=rsorted[max(0,int(0.20*(len(rsorted)-1)))]
    q50=statistics.median(ranges)
    table_ref=q20
    table_hi=table_ref+0.06
    off_table=sum(1 for x in ranges if x>table_hi)
    table_frac=1.0-off_table/max(1,len(ranges))

    dx_table=dy_table=0.0
    prev_t=None
    table_used=0
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        t=f(r,"mono_ns")*1e-9
        rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[t,rng,fx,fy])) or rng<=0:
            continue
        # Use only samples where Luna itself is consistent with the table.
        if rng<=table_hi:
            if prev_t is not None:
                dt=t-prev_t
                if 0<dt<0.2:
                    dx_table += fx*rng*dt
                    dy_table += fy*rng*dt
            prev_t=t
            table_used+=1
        else:
            # Break integration continuity across floor observations.
            prev_t=None
    raw_table_mm=1000*math.hypot(dx_table,dy_table)

    # A second counterfactual keeps all flow intervals but replaces the corrupt
    # floor range with the robust table reference. This estimates how much the
    # range switching alone can bias scale.
    dx_ref=dy_ref=0.0
    prev_t=None
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        t=f(r,"mono_ns")*1e-9
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[t,fx,fy])): continue
        if prev_t is not None:
            dt=t-prev_t
            if 0<dt<0.2:
                dx_ref += fx*table_ref*dt
                dy_ref += fy*table_ref*dt
        prev_t=t
    raw_ref_mm=1000*math.hypot(dx_ref,dy_ref)

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
    print(f"table range reference (q20) = {table_ref:.3f} m; table gate <= {table_hi:.3f} m")
    print(f"off-table/floor observations = {off_table}/{len(ranges)} ({100*off_table/max(1,len(ranges)):.1f}%)")
    print(f"RAW transmitted-flow integral = {raw_mm:.1f} mm")
    print(f"  components (body-rate proxy) = X {dx*1000:+.1f} mm, Y {dy*1000:+.1f} mm")
    print(f"RAW with fixed table-range counterfactual = {raw_ref_mm:.1f} mm")
    print(f"RAW table-only observed intervals = {raw_table_mm:.1f} mm (used {table_used} samples)")
    print(f"EKF LOCAL displacement = {ekf_mm:.1f} mm")
    print(f"  N/E = {dn*1000:+.1f}/{de*1000:+.1f} mm")
    if math.isfinite(ekf_mm) and raw_mm>1e-9:
        print(f"EKF/RAW = {ekf_mm/raw_mm:.4f}")
    if args.physical_mm is not None and args.physical_mm>0:
        print()
        print(f"PHYSICAL = {args.physical_mm:.1f} mm")
        print(f"RAW/PHYS = {raw_mm/args.physical_mm:.4f}")
        print(f"RAW_FIXED_TABLE/PHYS = {raw_ref_mm/args.physical_mm:.4f}")
        print(f"RAW_TABLE_ONLY/PHYS = {raw_table_mm/args.physical_mm:.4f}")
        print(f"EKF/PHYS = {ekf_mm/args.physical_mm:.4f}")
    else:
        print()
        print("PHYSICAL не задан. Не используем nominal 300 мм как ground truth.")
        print("Повтори команду с --physical-mm <реально измеренное расстояние>.")

if __name__=="__main__":main()
