#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math
from pathlib import Path

CAMERA_Z=0.050
RANGE_Z=0.071
DZ=CAMERA_Z-RANGE_Z

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def ii(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def body_to_ned(dx,dy,roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    r00=cy*cp
    r01=cy*sp*sr-sy*cr
    r10=sy*cp
    r11=sy*sp*sr+cy*cr
    return r00*dx+r01*dy, r10*dx+r11*dy

def step_disp(r):
    dt=f(r,"dt_s")
    vals=[f(r,k) for k in ("luna_m","flow_body_x","flow_body_y",
                            "fc_gyro_x","fc_gyro_y",
                            "fc_roll","fc_pitch","fc_yaw")]
    if not (0<dt<0.2) or not all(map(math.isfinite,vals)):
        return None
    lm,fx,fy,gx,gy,roll,pitch,yaw=vals
    h=lm-DZ
    if h<=0.02:return None
    comp_x=-fx+gx
    comp_y=-fy+gy
    dbx=(-comp_y)*h*dt
    dby=( comp_x)*h*dt
    dn,de=body_to_ned(dbx,dby,roll,pitch,yaw)
    return dn,de

def acc(rows, pred):
    n=e=0.0; used=0
    for r in rows:
        if not pred(r): continue
        d=step_disp(r)
        if d is None: continue
        n+=d[0]; e+=d[1]; used+=1
    return n,e,used

def magmm(v): return math.hypot(v[0],v[1])*1000

def main():
    ap=argparse.ArgumentParser(description="Разложение RAW displacement на sent/stale/all-valid")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")

    ev=[(j,ii(r,"return_event")) for j,r in enumerate(rows) if ii(r,"return_event") in (1,2,3)]
    ia=next((j for j,e in ev if e==1),None)
    ib=next((j for j,e in ev if e==2 and ia is not None and j>ia),None)
    ih=next((j for j,e in ev if e==3 and ia is not None and j>ia),None)
    if ia is None or ih is None:
        raise SystemExit(f"Нужны как минимум A/H events, найдено: {ev}")

    print("===== RAW DISPLACEMENT — SENT / STALE / ALL-VALID =====")
    print(f"CSV: {a.csv}")
    print(f"events: A={ia} B={ib if ib is not None else 'нет'} H={ih}")

    segments=[]
    if ib is not None and ia < ib < ih:
        segments += [("A->B",ia,ib),("B->H",ib,ih)]
    segments += [("A->H",ia,ih)]

    for name,lo,hi in segments:
        seg=rows[lo:hi+1]
        sent=acc(seg,lambda r: ii(r,"valid")==1 and ii(r,"flow_sent")==1)
        stale=acc(seg,lambda r: ii(r,"valid")==1 and ii(r,"flow_sent")!=1)
        allv=acc(seg,lambda r: ii(r,"valid")==1)
        inv=sum(ii(r,"valid")!=1 for r in seg)

        print()
        print(name)
        print(f"  sent-only: N/E={sent[0]*1000:+.1f}/{sent[1]*1000:+.1f} mm |.|={magmm(sent):.1f} mm samples={sent[2]}")
        print(f"  stale-only:N/E={stale[0]*1000:+.1f}/{stale[1]*1000:+.1f} mm |.|={magmm(stale):.1f} mm samples={stale[2]}")
        print(f"  all-valid: N/E={allv[0]*1000:+.1f}/{allv[1]*1000:+.1f} mm |.|={magmm(allv):.1f} mm samples={allv[2]}")
        print(f"  invalid rows={inv}")

        # Contribution of stale rejected intervals to the all-valid displacement.
        dn=(allv[0]-sent[0])*1000
        de=(allv[1]-sent[1])*1000
        print(f"  all-valid - sent-only = {dn:+.1f}/{de:+.1f} mm  |.|={math.hypot(dn,de):.1f} mm")

    # EKF marked-point displacement for comparison
    an,ae=f(rows[ia],"ekf_x_ned"),f(rows[ia],"ekf_y_ned")
    hn,he=f(rows[ih],"ekf_x_ned"),f(rows[ih],"ekf_y_ned")
    print()
    print("EKF MARKED POINTS")
    if ib is not None and ia < ib < ih:
        bn,be=f(rows[ib],"ekf_x_ned"),f(rows[ib],"ekf_y_ned")
        print(f"  A->B = {math.hypot(bn-an,be-ae)*1000:.1f} mm")
        print(f"  B->H = {math.hypot(hn-bn,he-be)*1000:.1f} mm")
    print(f"  A->H = {math.hypot(hn-an,he-ae)*1000:.1f} mm")

    print()
    print("ИНТЕРПРЕТАЦИЯ:")
    print("  Если all-valid A->B резко ближе к физическому/EKF displacement, чем sent-only,")
    print("  то stale_reject действительно выбрасывает существенную часть движения.")
    print("  Если all-valid почти такой же, причина не в stale policy и нужно разбирать")
    print("  gyro/attitude/range coupling на самих valid кадрах.")

if __name__=="__main__":
    main()
