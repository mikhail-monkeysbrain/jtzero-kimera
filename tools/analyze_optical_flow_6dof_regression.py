#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path
from collections import Counter

CAMERA_Z=0.050
RANGE_Z=0.071
CAMERA_RANGE_DZ=CAMERA_Z-RANGE_Z  # -0.021 m

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def ii(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def pct(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    j=min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))
    return xs[j]

def longest(flags):
    best=cur=0
    for x in flags:
        if x: cur+=1; best=max(best,cur)
        else: cur=0
    return best

def body_to_ned(dx,dy,roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    r00=cy*cp
    r01=cy*sp*sr-sy*cr
    r10=sy*cp
    r11=sy*sp*sr+cy*cr
    return r00*dx+r01*dy, r10*dx+r11*dy

def integrate_segment(rows,lo,hi):
    n=e=bx=by=losx=losy=0.0
    used=0
    heights=[]
    rolls=[];pitches=[];yaws=[]
    for r in rows[lo:hi+1]:
        if ii(r,"valid")!=1 or ii(r,"flow_sent")!=1: continue
        dt=f(r,"dt_s")
        if not (0<dt<0.2): continue
        lm=f(r,"luna_m")
        fx=f(r,"flow_body_x"); fy=f(r,"flow_body_y")
        gx=f(r,"fc_gyro_x"); gy=f(r,"fc_gyro_y")
        roll=f(r,"fc_roll"); pitch=f(r,"fc_pitch"); yaw=f(r,"fc_yaw")
        vals=[lm,fx,fy,gx,gy,roll,pitch,yaw]
        if not all(map(math.isfinite,vals)): continue
        h=lm-CAMERA_RANGE_DZ
        if h<=0.02: continue

        # Mirror publisher/ArduPilot optical-flow convention used by GUI RAW NED:
        # comp_x=-flow_x+gyro_x; comp_y=-flow_y+gyro_y
        # body vx=-comp_y*h; body vy=comp_x*h
        comp_x=-fx+gx
        comp_y=-fy+gy
        dbx=(-comp_y)*h*dt
        dby=( comp_x)*h*dt
        dn,de=body_to_ned(dbx,dby,roll,pitch,yaw)
        bx+=dbx; by+=dby
        n+=dn; e+=de
        losx+=fx*h*dt; losy+=fy*h*dt
        heights.append(h); rolls.append(roll); pitches.append(pitch); yaws.append(yaw)
        used+=1
    return dict(n=n,e=e,bx=bx,by=by,losx=losx,losy=losy,used=used,
                heights=heights,rolls=rolls,pitches=pitches,yaws=yaws)

def angle_span(xs):
    if not xs:return float("nan")
    # adequate for short hand tests that do not cross +/-pi multiple times
    un=[xs[0]]
    for x in xs[1:]:
        prev=un[-1]
        d=math.remainder(x-prev,2*math.pi)
        un.append(prev+d)
    return max(un)-min(un)

def main():
    ap=argparse.ArgumentParser(description="JT-Zero 6-DoF hand regression forensic")
    ap.add_argument("csv",type=Path)
    ap.add_argument("--raw-gate-mm",type=float,default=20.0)
    ap.add_argument("--ekf-gate-mm",type=float,default=30.0)
    ap.add_argument("--invalid-gate-pct",type=float,default=3.0)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")

    events=[(j,ii(r,"return_event")) for j,r in enumerate(rows) if ii(r,"return_event") in (1,2,3)]
    ia=next((j for j,e in events if e==1),None)
    ib=next((j for j,e in events if e==2 and ia is not None and j>ia),None)
    ih=next((j for j,e in events if e==3 and ib is not None and j>ib),None)

    print("===== JT-ZERO — 6-DoF РУЧНОЙ REGRESSION FORENSIC =====")
    print(f"CSV: {a.csv}")
    print(f"rows={len(rows)} events={events}")
    if ia is None or ib is None or ih is None:
        print("РЕЗУЛЬТАТ: НЕДОСТАТОЧНО ДАННЫХ — нужны события SPACE(A), B и H.")
        raise SystemExit(2)

    seg=rows[ia:ih+1]
    invalid=[ii(r,"valid")!=1 for r in seg]
    invalid_n=sum(invalid)
    invalid_pct=100*invalid_n/len(seg)
    stale=sum(ii(r,"valid")==1 and ii(r,"flow_sent")!=1 for r in seg)
    reasons=Counter(ii(r,"invalid_reason") for r in seg if ii(r,"valid")!=1)

    ab=integrate_segment(rows,ia,ib)
    bh=integrate_segment(rows,ib,ih)
    ah=integrate_segment(rows,ia,ih)

    # EKF is directly observable at marked rows.
    an,ae=f(rows[ia],"ekf_x_ned"),f(rows[ia],"ekf_y_ned")
    bn,be=f(rows[ib],"ekf_x_ned"),f(rows[ib],"ekf_y_ned")
    hn,he=f(rows[ih],"ekf_x_ned"),f(rows[ih],"ekf_y_ned")
    ekf_ab=math.hypot(bn-an,be-ae)*1000
    ekf_bh=math.hypot(hn-bn,he-be)*1000
    ekf_cl=math.hypot(hn-an,he-ae)*1000
    raw_ab=math.hypot(ab["n"],ab["e"])*1000
    raw_bh=math.hypot(bh["n"],bh["e"])*1000
    raw_cl=math.hypot(ah["n"],ah["e"])*1000

    heights=ah["heights"]; rolls=ah["rolls"]; pitches=ah["pitches"]; yaws=ah["yaws"]
    hmin=min(heights) if heights else float("nan")
    hmax=max(heights) if heights else float("nan")

    print()
    print("ЗАМЫКАНИЕ:")
    print(f"  RAW NED A->B = {raw_ab:.1f} мм")
    print(f"  RAW NED B->H = {raw_bh:.1f} мм")
    print(f"  RAW NED closure A->H = {raw_cl:.1f} мм  dN/E={ah['n']*1000:+.1f}/{ah['e']*1000:+.1f} мм")
    print(f"  EKF A->B = {ekf_ab:.1f} мм")
    print(f"  EKF B->H = {ekf_bh:.1f} мм")
    print(f"  EKF closure A->H = {ekf_cl:.1f} мм")
    print(f"  |EKF-RAW closure| = {abs(ekf_cl-raw_cl):.1f} мм")

    print()
    print("6-DoF НАГРУЗКА:")
    print(f"  camera height min/max/span = {hmin:.3f}/{hmax:.3f}/{(hmax-hmin):.3f} м")
    print(f"  roll span  = {math.degrees(angle_span(rolls)):.1f}°")
    print(f"  pitch span = {math.degrees(angle_span(pitches)):.1f}°")
    print(f"  yaw span   = {math.degrees(angle_span(yaws)):.1f}°")

    print()
    print("FRONTEND:")
    print(f"  interval rows = {len(seg)}")
    print(f"  invalid = {invalid_n} ({invalid_pct:.2f}%)")
    print(f"  longest invalid run = {longest(invalid)} frames")
    print(f"  stale/not-sent valid flow = {stale}")
    names={0:"unknown/startup",1:"bad_dt",2:"few_features",3:"few_tracked",4:"homography_fail",5:"few_inliers",6:"flow_too_large"}
    for k,n in sorted(reasons.items()):
        print(f"  reason {k} {names.get(k,'unknown')} = {n}")

    lat=[f(r,"frame_pipeline_latency_ms") for r in seg if math.isfinite(f(r,"frame_pipeline_latency_ms")) and f(r,"frame_pipeline_latency_ms")>=0]
    if lat:
        print(f"  latency median/p95/max = {statistics.median(lat):.1f}/{pct(lat,.95):.1f}/{max(lat):.1f} ms")

    raw_ok=raw_cl<a.raw_gate_mm
    ekf_ok=ekf_cl<a.ekf_gate_mm
    inv_ok=invalid_pct<a.invalid_gate_pct
    run_ok=longest(invalid)<=2
    print()
    print("GATES:")
    print(f"  RAW closure < {a.raw_gate_mm:.0f} мм: {'PASS' if raw_ok else 'FAIL'}")
    print(f"  EKF closure < {a.ekf_gate_mm:.0f} мм: {'PASS' if ekf_ok else 'FAIL'}")
    print(f"  invalid < {a.invalid_gate_pct:.1f}%: {'PASS' if inv_ok else 'FAIL'}")
    print(f"  longest invalid run <= 2: {'PASS' if run_ok else 'FAIL'}")
    print()
    if raw_ok and ekf_ok and inv_ok and run_ok:
        print("РЕЗУЛЬТАТ: PASS — 6-DoF ручной regression-test прошёл текущий диагностический gate.")
    else:
        print("РЕЗУЛЬТАТ: FAIL — следующий шаг определять по провалившемуся gate; параметры вслепую не менять.")

if __name__=="__main__":
    main()
