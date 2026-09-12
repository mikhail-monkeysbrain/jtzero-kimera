#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r[k])
    except:return d

def i(r,k,d=0):
    try:return int(float(r[k]))
    except:return d

def pct(xs,p):
    if not xs:return float("nan")
    ys=sorted(xs)
    j=min(len(ys)-1,max(0,int(round((len(ys)-1)*p))))
    return ys[j]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    args=ap.parse_args()
    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows:raise SystemExit("empty CSV")

    valid=[r for r in rows if i(r,"valid",0)==1 and i(r,"flow_sent",0)==1]
    invalid=sum(1 for r in rows if i(r,"valid",0)==0)
    ranges=[f(r,"range_to_fc_m") for r in valid if math.isfinite(f(r,"range_to_fc_m"))]
    flows=[math.hypot(f(r,"flow_send_x"),f(r,"flow_send_y")) for r in valid
           if math.isfinite(f(r,"flow_send_x")) and math.isfinite(f(r,"flow_send_y"))]
    inliers=[f(r,"inliers") for r in valid if math.isfinite(f(r,"inliers"))]

    gyros=[]
    for r in rows:
        gx,gy,gz=f(r,"fc_gyro_x"),f(r,"fc_gyro_y"),f(r,"fc_gyro_z")
        if all(map(math.isfinite,[gx,gy,gz])):
            gyros.append(math.sqrt(gx*gx+gy*gy+gz*gz))

    loc=[r for r in rows if i(r,"ekf_local_valid",0)==1]
    speeds=[]
    for r in loc:
        vn,ve=f(r,"ekf_vx_ned"),f(r,"ekf_vy_ned")
        if math.isfinite(vn) and math.isfinite(ve):
            speeds.append(math.hypot(vn,ve))

    drift=float("nan"); dn=de=float("nan")
    if len(loc)>=2:
        dn=f(loc[-1],"ekf_x_ned")-f(loc[0],"ekf_x_ned")
        de=f(loc[-1],"ekf_y_ned")-f(loc[0],"ekf_y_ned")
        drift=math.hypot(dn,de)

    dt=float("nan")
    if len(rows)>=2:
        t0=f(rows[0],"mono_ns"); t1=f(rows[-1],"mono_ns")
        if math.isfinite(t0) and math.isfinite(t1):
            dt=(t1-t0)*1e-9

    print("===== OPTICAL FLOW STATIC HEALTH =====")
    print(f"duration = {dt:.2f} s")
    print(f"frames = {len(rows)} valid_tx = {len(valid)} invalid = {invalid} ({100*invalid/max(1,len(rows)):.3f}%)")
    if ranges:
        print(f"range median/min/max = {statistics.median(ranges):.3f}/{min(ranges):.3f}/{max(ranges):.3f} m")
    if flows:
        print(f"|flow| rad/s median/p95/max = {statistics.median(flows):.5f}/{pct(flows,.95):.5f}/{max(flows):.5f}")
    if inliers:
        print(f"inliers median/p05/min = {statistics.median(inliers):.1f}/{pct(inliers,.05):.1f}/{min(inliers):.1f}")
    if gyros:
        print(f"|gyro| rad/s median/p95/max = {statistics.median(gyros):.5f}/{pct(gyros,.95):.5f}/{max(gyros):.5f}")
    if speeds:
        print(f"EKF |vH| m/s median/p95/max = {statistics.median(speeds):.5f}/{pct(speeds,.95):.5f}/{max(speeds):.5f}")
    if math.isfinite(drift):
        print(f"EKF position drift = {drift*1000:.1f} mm  dN/dE={dn*1000:+.1f}/{de*1000:+.1f} mm")
        if dt and dt>0:
            print(f"drift rate = {drift/dt*1000:.3f} mm/s")

    fail=[]
    if invalid/max(1,len(rows))>0.01: fail.append("invalid >1%")
    if speeds and pct(speeds,.95)>0.03: fail.append("EKF speed p95 >0.03 m/s")
    if ranges and max(ranges)-min(ranges)>0.05: fail.append("range span >0.05 m")
    if math.isfinite(drift) and drift>0.05: fail.append("position drift >50 mm")

    print()
    if fail:
        print("RESULT: FAIL")
        for x in fail: print("  - "+x)
    else:
        print("RESULT: PASS")
        print("Static optical-flow/EKF health is within the current bench gate.")

if __name__=="__main__":main()
