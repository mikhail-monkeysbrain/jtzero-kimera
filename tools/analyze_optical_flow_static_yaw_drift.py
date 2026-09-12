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

def wrap(x):
    return (x+math.pi)%(2*math.pi)-math.pi

def pct(xs,p):
    if not xs:return float("nan")
    ys=sorted(xs)
    return ys[min(len(ys)-1,max(0,int(round((len(ys)-1)*p))))]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    args=ap.parse_args()
    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows: raise SystemExit("empty CSV")
    if "fc_yaw" not in rows[0]:
        raise SystemExit("fc_yaw column missing: run was recorded before yaw logging was added")

    good=[r for r in rows if i(r,"ekf_local_valid",0)==1 and math.isfinite(f(r,"fc_yaw"))]
    if len(good)<20: raise SystemExit("not enough valid rows")

    yaw0=f(good[0],"fc_yaw")
    yaws=[wrap(f(r,"fc_yaw")-yaw0) for r in good]
    ydeg=[math.degrees(x) for x in yaws]
    gyros=[math.sqrt(f(r,"fc_gyro_x",0)**2+f(r,"fc_gyro_y",0)**2+f(r,"fc_gyro_z",0)**2) for r in good]

    n0,e0=f(good[0],"ekf_x_ned"),f(good[0],"ekf_y_ned")
    n1,e1=f(good[-1],"ekf_x_ned"),f(good[-1],"ekf_y_ned")
    drift=math.hypot(n1-n0,e1-e0)

    speeds=[math.hypot(f(r,"ekf_vx_ned",0),f(r,"ekf_vy_ned",0)) for r in good]
    flows=[math.hypot(f(r,"flow_send_x",0),f(r,"flow_send_y",0)) for r in good if i(r,"flow_sent",0)==1]

    t0=f(rows[0],"mono_ns"); t1=f(rows[-1],"mono_ns")
    dur=(t1-t0)*1e-9 if math.isfinite(t0) and math.isfinite(t1) else float("nan")

    print("===== STATIC YAW / DRIFT FORENSIC =====")
    print(f"duration = {dur:.2f} s")
    print(f"EKF drift = {drift*1000:.1f} mm  dN/dE={(n1-n0)*1000:+.1f}/{(e1-e0)*1000:+.1f} mm")
    print(f"yaw delta end = {ydeg[-1]:+.2f} deg")
    print(f"yaw excursion min/max = {min(ydeg):+.2f}/{max(ydeg):+.2f} deg")
    print(f"|gyro| median/p95/max = {statistics.median(gyros):.5f}/{pct(gyros,.95):.5f}/{max(gyros):.5f} rad/s")
    print(f"EKF |vH| median/p95/max = {statistics.median(speeds):.5f}/{pct(speeds,.95):.5f}/{max(speeds):.5f} m/s")
    if flows:
        print(f"|flow| median/p95/max = {statistics.median(flows):.5f}/{pct(flows,.95):.5f}/{max(flows):.5f} rad/s")

    print()
    if abs(ydeg[-1])>3 or (max(ydeg)-min(ydeg))>5:
        print("VERDICT: YAW UNSTABLE — heading drift/jumps are large enough to contaminate N/E position.")
    elif drift*1000>50 and (statistics.median(flows) if flows else 0)<0.02:
        print("VERDICT: LARGE EKF DRIFT WITH SMALL RAW FLOW — suspect yaw/fusion state, not frontend scale.")
    else:
        print("VERDICT: no large yaw instability detected in this static run.")

if __name__=="__main__": main()
