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
    ap.add_argument("--camera-z-m",type=float,default=0.0500)
    ap.add_argument("--luna-z-m",type=float,default=0.0710)
    args=ap.parse_args()

    with args.csv.open(newline="") as fh:
        rows=list(csv.DictReader(fh))

    move=[r for r in rows if int(f(r,"guide_stage",-1))==1]
    if len(move)<2:
        raise SystemExit("guide_stage=1 не найден")

    # Reconstruct visual displacement from the SAME camera interval used to
    # compute each transmitted flow rate. flow_send_x/y = angular displacement / dt_s,
    # therefore displacement contribution is rate * range * dt_s.
    #
    # IMPORTANT: using time between consecutive MAVLink sends here is wrong when
    # an intermediate visual step is invalid: the next valid rate still describes
    # only its own camera pair, not the whole send-to-send gap.
    dx=dy=0.0
    valid=0
    ranges=[]
    mags=[]
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        dt=f(r,"dt_s")
        rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[dt,rng,fx,fy])) or not (0<dt<0.2) or rng<=0:
            continue
        dx += fx*rng*dt
        dy += fy*rng*dt
        valid+=1
        ranges.append(rng)
        mags.append(math.hypot(fx,fy))

    raw_mm=1000*math.hypot(dx,dy)

    dz_cam_luna=args.camera_z_m-args.luna_z_m
    dx_cam=dy_cam=0.0
    cam_heights=[]
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        dt=f(r,"dt_s")
        rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        hcam=rng-dz_cam_luna
        if not all(map(math.isfinite,[dt,hcam,fx,fy])) or not (0<dt<0.2) or hcam<=0:
            continue
        dx_cam += fx*hcam*dt
        dy_cam += fy*hcam*dt
        cam_heights.append(hcam)
    raw_cam_mm=1000*math.hypot(dx_cam,dy_cam)

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
    table_used=0
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        dt=f(r,"dt_s")
        rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[dt,rng,fx,fy])) or not (0<dt<0.2) or rng<=0:
            continue
        if rng<=table_hi:
            dx_table += fx*rng*dt
            dy_table += fy*rng*dt
            table_used+=1
    raw_table_mm=1000*math.hypot(dx_table,dy_table)

    # A second counterfactual keeps all flow intervals but replaces the corrupt
    # floor range with the robust table reference. This estimates how much the
    # range switching alone can bias scale.
    dx_ref=dy_ref=0.0
    for r in move:
        if int(f(r,"valid",0))!=1 or int(f(r,"flow_sent",0))!=1:
            continue
        dt=f(r,"dt_s")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        if not all(map(math.isfinite,[dt,fx,fy])) or not (0<dt<0.2):
            continue
        dx_ref += fx*table_ref*dt
        dy_ref += fy*table_ref*dt
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
    print(f"RAW using TF-Luna range directly = {raw_mm:.1f} mm")
    print(f"  components (body-rate proxy) = X {dx*1000:+.1f} mm, Y {dy*1000:+.1f} mm")
    relation = "below" if dz_cam_luna>0 else "above"
    print(f"sensor Z geometry: camera={args.camera_z_m:.3f} m, luna={args.luna_z_m:.3f} m, camera is {abs(dz_cam_luna)*1000:.1f} mm {relation} TF-Luna")
    print(f"camera height median/min/max = {statistics.median(cam_heights):.3f}/{min(cam_heights):.3f}/{max(cam_heights):.3f} m")
    print(f"RAW geometry-corrected camera-height integral = {raw_cam_mm:.1f} mm")
    print(f"RAW with fixed table-range counterfactual = {raw_ref_mm:.1f} mm")
    print(f"RAW table-only observed intervals = {raw_table_mm:.1f} mm (used {table_used} samples)")
    print(f"EKF LOCAL displacement = {ekf_mm:.1f} mm")
    print(f"  N/E = {dn*1000:+.1f}/{de*1000:+.1f} mm")
    if math.isfinite(ekf_mm) and raw_mm>1e-9:
        print(f"EKF/RAW_LUNA_RANGE = {ekf_mm/raw_mm:.4f}")
    if math.isfinite(ekf_mm) and raw_cam_mm>1e-9:
        print(f"EKF/RAW_CAMERA_HEIGHT = {ekf_mm/raw_cam_mm:.4f}")
    if args.physical_mm is not None and args.physical_mm>0:
        print()
        print(f"PHYSICAL = {args.physical_mm:.1f} mm")
        print(f"RAW_LUNA_RANGE/PHYS = {raw_mm/args.physical_mm:.4f}")
        print(f"RAW_CAMERA_HEIGHT/PHYS = {raw_cam_mm/args.physical_mm:.4f}")
        print(f"RAW_FIXED_TABLE/PHYS = {raw_ref_mm/args.physical_mm:.4f}")
        print(f"RAW_TABLE_ONLY/PHYS = {raw_table_mm/args.physical_mm:.4f}")
        print(f"EKF/PHYS = {ekf_mm/args.physical_mm:.4f}")
    else:
        print()
        print("PHYSICAL не задан. Не используем nominal 300 мм как ground truth.")
        print("Повтори команду с --physical-mm <реально измеренное расстояние>.")

if __name__=="__main__":main()
