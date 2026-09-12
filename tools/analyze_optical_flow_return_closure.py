#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

CAM_Z_DEFAULT=0.050
RANGE_Z_DEFAULT=0.071

def f(r,k,d=float("nan")):
    try:return float(r[k])
    except:return d

def i(r,k,d=0):
    try:return int(float(r[k]))
    except:return d

def wrap_deg(x):
    return (x+180.0)%360.0-180.0

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("--camera-z-m",type=float,default=CAM_Z_DEFAULT)
    ap.add_argument("--range-z-m",type=float,default=RANGE_Z_DEFAULT)
    args=ap.parse_args()

    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows:
        raise SystemExit("empty CSV")
    required={"return_event","fc_yaw","ekf_local_valid","ekf_x_ned","ekf_y_ned",
              "flow_sent","valid","dt_s","range_to_fc_m","flow_send_x","flow_send_y"}
    missing=required-set(rows[0].keys())
    if missing:
        raise SystemExit("run was recorded before closure forensic fields were added: "+", ".join(sorted(missing)))

    events=[]
    for idx,r in enumerate(rows):
        ev=i(r,"return_event",0)
        if ev in (1,2,3):
            events.append((idx,ev,r))
    # Event is logged on the next frame after the key, which is adequate at camera rate.
    a=next(((idx,r) for idx,ev,r in events if ev==1),None)
    b=next(((idx,r) for idx,ev,r in events if ev==2 and (a is None or idx>a[0])),None)
    h=next(((idx,r) for idx,ev,r in events if ev==3 and (b is None or idx>b[0])),None)

    print("===== RETURN LOOP CLOSURE FORENSIC =====")
    print(f"rows={len(rows)} events={[ev for _,ev,_ in events]}")
    if not a:
        raise SystemExit("A/target event not found. Start GUI and wait for auto target or press SPACE.")
    if not b:
        raise SystemExit("B/turn event not found. Press B at the far point in the next run.")
    if not h:
        raise SystemExit("H/physical-home event not found. Press H after physically returning to A.")

    ia,ra=a; ib,rb=b; ih,rh=h

    def ekf_xy(r):
        return f(r,"ekf_x_ned"),f(r,"ekf_y_ned")
    an,ae=ekf_xy(ra); bn,be=ekf_xy(rb); hn,he=ekf_xy(rh)
    ab_ekf=(bn-an,be-ae)
    bh_ekf=(hn-bn,he-be)
    ah_ekf=(hn-an,he-ae)

    dz=args.camera_z_m-args.range_z_m
    rawx=rawy=0.0
    bodyx=bodyy=0.0
    nedn=nede=0.0
    raw_at={ia:(0.0,0.0)}
    body_at={ia:(0.0,0.0)}
    ned_at={ia:(0.0,0.0)}
    valid_used=0
    invalid=0

    def accum_row(r):
        nonlocal rawx,rawy,bodyx,bodyy,nedn,nede,valid_used
        if i(r,"valid",0)!=1 or i(r,"flow_sent",0)!=1:
            return False
        dt=f(r,"dt_s"); rng=f(r,"range_to_fc_m")
        fx=f(r,"flow_send_x"); fy=f(r,"flow_send_y")
        gx=f(r,"fc_gyro_x"); gy=f(r,"fc_gyro_y")
        roll=f(r,"fc_roll"); pitch=f(r,"fc_pitch"); yaw=f(r,"fc_yaw")
        hcam=rng-dz
        vals=[dt,hcam,fx,fy,gx,gy,roll,pitch,yaw]
        if not all(map(math.isfinite,vals)) or not (0<dt<0.2) or hcam<=0:
            return False

        # Legacy LOS integral.
        rawx += fx*hcam*dt
        rawy += fy*hcam*dt

        # Mirror ArduPilot EKF3 conventions:
        # internal flow = -rawFlowRates; compensated = internal + body rates.
        comp_x=-fx+gx
        comp_y=-fy+gy
        dbx=(-comp_y)*hcam*dt
        dby=( comp_x)*hcam*dt
        bodyx += dbx
        bodyy += dby

        # Full body-FRD -> NED 3-2-1 rotation for planar body displacement.
        cr,sr=math.cos(roll),math.sin(roll)
        cp,sp=math.cos(pitch),math.sin(pitch)
        cy,sy=math.cos(yaw),math.sin(yaw)
        r00=cy*cp
        r01=cy*sp*sr-sy*cr
        r10=sy*cp
        r11=sy*sp*sr+cy*cr
        nedn += r00*dbx+r01*dby
        nede += r10*dbx+r11*dby
        valid_used+=1
        return True

    for idx in range(ia,ih+1):
        r=rows[idx]
        ok=accum_row(r)
        if not ok:
            invalid+=1
        if idx in (ib,ih):
            raw_at[idx]=(rawx,rawy)
            body_at[idx]=(bodyx,bodyy)
            ned_at[idx]=(nedn,nede)

    raw_at.setdefault(ib,(rawx,rawy))
    body_at.setdefault(ib,(bodyx,bodyy))
    ned_at.setdefault(ib,(nedn,nede))
    raw_at[ih]=(rawx,rawy)
    body_at[ih]=(bodyx,bodyy)
    ned_at[ih]=(nedn,nede)

    ab_raw=raw_at[ib]
    ah_raw=raw_at[ih]
    bh_raw=(ah_raw[0]-ab_raw[0],ah_raw[1]-ab_raw[1])

    ab_body=body_at[ib]
    ah_body=body_at[ih]
    bh_body=(ah_body[0]-ab_body[0],ah_body[1]-ab_body[1])

    ab_ned=ned_at[ib]
    ah_ned=ned_at[ih]
    bh_ned=(ah_ned[0]-ab_ned[0],ah_ned[1]-ab_ned[1])

    yaw_a=math.degrees(f(ra,"fc_yaw",0.0))
    yaw_b=math.degrees(f(rb,"fc_yaw",0.0))
    yaw_h=math.degrees(f(rh,"fc_yaw",0.0))

    def mm(v): return 1000.0*math.hypot(v[0],v[1])
    print(f"A row={ia}  EKF=({an:.4f},{ae:.4f}) yaw={yaw_a:.2f} deg")
    print(f"B row={ib}  EKF=({bn:.4f},{be:.4f}) yaw={yaw_b:.2f} deg")
    print(f"H row={ih}  EKF=({hn:.4f},{he:.4f}) yaw={yaw_h:.2f} deg")
    print()
    print("EKF:")
    print(f"  A->B vector N/E = {ab_ekf[0]*1000:+.1f}/{ab_ekf[1]*1000:+.1f} mm  |.|={mm(ab_ekf):.1f} mm")
    print(f"  B->H vector N/E = {bh_ekf[0]*1000:+.1f}/{bh_ekf[1]*1000:+.1f} mm  |.|={mm(bh_ekf):.1f} mm")
    print(f"  closure A->H     = {ah_ekf[0]*1000:+.1f}/{ah_ekf[1]*1000:+.1f} mm  |.|={mm(ah_ekf):.1f} mm")
    print()
    print("RAW AP-model NED metric (gyro compensated + ATTITUDE body->NED):")
    print(f"  A->B N/E = {ab_ned[0]*1000:+.1f}/{ab_ned[1]*1000:+.1f} mm  |.|={mm(ab_ned):.1f} mm")
    print(f"  B->H N/E = {bh_ned[0]*1000:+.1f}/{bh_ned[1]*1000:+.1f} mm  |.|={mm(bh_ned):.1f} mm")
    print(f"  closure  = {ah_ned[0]*1000:+.1f}/{ah_ned[1]*1000:+.1f} mm  |.|={mm(ah_ned):.1f} mm")
    print()
    print("RAW AP-model BODY metric:")
    print(f"  A->B X/Y = {ab_body[0]*1000:+.1f}/{ab_body[1]*1000:+.1f} mm  |.|={mm(ab_body):.1f} mm")
    print(f"  B->H X/Y = {bh_body[0]*1000:+.1f}/{bh_body[1]*1000:+.1f} mm  |.|={mm(bh_body):.1f} mm")
    print(f"  closure  = {ah_body[0]*1000:+.1f}/{ah_body[1]*1000:+.1f} mm  |.|={mm(ah_body):.1f} mm")
    print()
    print("RAW legacy LOS integral (for comparison only; rotating axes):")
    print(f"  A->B = {ab_raw[0]*1000:+.1f}/{ab_raw[1]*1000:+.1f} mm  |.|={mm(ab_raw):.1f} mm")
    print(f"  B->H = {bh_raw[0]*1000:+.1f}/{bh_raw[1]*1000:+.1f} mm  |.|={mm(bh_raw):.1f} mm")
    print(f"  closure = {ah_raw[0]*1000:+.1f}/{ah_raw[1]*1000:+.1f} mm  |.|={mm(ah_raw):.1f} mm")
    print(f"  valid samples={valid_used}, invalid/non-sent rows={invalid}")
    print()
    print("Yaw:")
    print(f"  A={yaw_a:.2f} deg  B={yaw_b:.2f} deg  H={yaw_h:.2f} deg")
    print(f"  dYaw A->B={wrap_deg(yaw_b-yaw_a):+.2f} deg")
    print(f"  dYaw A->H={wrap_deg(yaw_h-yaw_a):+.2f} deg")
    print()

    raw_close=mm(ah_ned)
    ekf_close=mm(ah_ekf)
    print(f"EKF vs RAW-NED closure delta = {ekf_close-raw_close:+.1f} mm")
    if raw_close<=25 and ekf_close>50:
        print("INTERPRETATION: attitude-corrected RAW NED approximately closes but EKF does not; investigate EKF fusion after the frontend.")
    elif raw_close>50 and abs(raw_close-ekf_close)<40:
        print("INTERPRETATION: EKF follows the attitude-corrected RAW NED non-closure; investigate flow/gyro/height modelling rather than yaw-frame accumulation.")
    elif raw_close>50 and ekf_close>50:
        print("INTERPRETATION: both RAW NED and EKF fail to close, but by different amounts; compare A->B/B->H vectors and attitude/height.")
    else:
        print("INTERPRETATION: both attitude-corrected RAW NED and EKF closure are small in this run.")

if __name__=="__main__":
    main()
