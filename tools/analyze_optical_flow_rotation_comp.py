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

def integrate(rows, physical_mm):
    rawx=rawy=compx=compy=0.0
    gx=[];gy=[];gz=[];ages=[];samples=[]
    used=0
    for r in rows:
        if i(r,"guide_stage",-1)!=1 or i(r,"valid",0)!=1 or i(r,"flow_sent",0)!=1:
            continue
        dt=f(r,"dt_s"); rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        gxx=f(r,"fc_gyro_x"); gyy=f(r,"fc_gyro_y"); gzz=f(r,"fc_gyro_z")
        age=f(r,"fc_gyro_age_ms"); ns=f(r,"fc_gyro_samples")
        vals=[dt,rng,fx,fy,gxx,gyy,gzz]
        if not all(map(math.isfinite,vals)) or not (0<dt<0.2) or rng<=0:
            continue
        rawx += fx*rng*dt
        rawy += fy*rng*dt
        # Mirror ArduPilot EKF convention:
        # ofDataNew.flowRadXY = -rawFlowRates
        # flowRadXYcomp = flowRadXY + bodyRadXYZ
        cx = -fx + gxx
        cy = -fy + gyy
        compx += cx*rng*dt
        compy += cy*rng*dt
        gx.append(gxx);gy.append(gyy);gz.append(gzz);ages.append(age);samples.append(ns)
        used+=1
    raw=1000*math.hypot(rawx,rawy)
    comp=1000*math.hypot(compx,compy)
    return raw,comp,used,gx,gy,gz,ages,samples,(rawx,rawy),(compx,compy)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("--physical-mm",type=float,required=True)
    args=ap.parse_args()
    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows:raise SystemExit("empty CSV")
    if "fc_gyro_x" not in rows[0]:
        raise SystemExit("Этот run записан до добавления FC gyro columns. Нужен один новый прогон.")

    raw,comp,used,gx,gy,gz,ages,samples,rv,cv=integrate(rows,args.physical_mm)

    print("===== ROTATION-COMPENSATED FLOW FORENSIC =====")
    print(f"move samples used = {used}")
    print(f"RAW uncorrected displacement = {raw:.1f} mm  ratio={raw/args.physical_mm:.4f}")
    print(f"AP-style gyro-comp displacement = {comp:.1f} mm  ratio={comp/args.physical_mm:.4f}")
    print(f"RAW components = {rv[0]*1000:+.1f}/{rv[1]*1000:+.1f} mm")
    print(f"COMP components = {cv[0]*1000:+.1f}/{cv[1]*1000:+.1f} mm")
    if gx:
        gnorm=[math.sqrt(a*a+b*b+c*c) for a,b,c in zip(gx,gy,gz)]
        print(f"FC gyro norm median/p95/max = {statistics.median(gnorm):.4f}/{sorted(gnorm)[int(.95*(len(gnorm)-1))]:.4f}/{max(gnorm):.4f} rad/s")
        print(f"gyro age median/max = {statistics.median(ages):.2f}/{max(ages):.2f} ms")
        print(f"gyro samples/frame median = {statistics.median(samples):.1f}")
    print()
    if abs(comp/args.physical_mm-1.0) < abs(raw/args.physical_mm-1.0):
        print("INTERPRETATION: body rotation explains part of the apparent RAW scale error.")
    else:
        print("INTERPRETATION: AP-style body-rate compensation does not improve metric agreement; investigate frontend/height next.")

if __name__=="__main__":main()
