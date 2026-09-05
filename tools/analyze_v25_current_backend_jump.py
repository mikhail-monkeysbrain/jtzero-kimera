#!/usr/bin/env python3
import csv, os, math, sys

BACK=os.path.expanduser("~/jtzero_500mm_v25_backend.csv")
FRONT=os.path.expanduser("~/jtzero_500mm_v25_frontend.csv")
IMU=os.path.expanduser("~/jtzero_500mm_v25.csv")

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k,d=0.0):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def norm(v): return math.sqrt(sum(x*x for x in v))

if not os.path.exists(BACK):
    raise SystemExit("missing backend CSV")
back=read(BACK)
front=read(FRONT) if os.path.exists(FRONT) else []

best=None
for a,b in zip(back,back[1:]):
    pa=[F(a,"px_m"),F(a,"py_m"),F(a,"pz_m")]
    pb=[F(b,"px_m"),F(b,"py_m"),F(b,"pz_m")]
    dp=[(pb[j]-pa[j])*1000 for j in range(3)]
    mag=norm(dp)
    dt=(I(b,"timestamp_ns")-I(a,"timestamp_ns"))*1e-9
    if best is None or mag>best[0]:
        best=(mag,I(b,"keyframe"),dt,dp,a,b)

mag,kf,dt,dp,a0,b0=best
print("================ CURRENT V25 BACKEND JUMP ================")
print(f"largest backend jump KF={kf} dP={mag:.2f}mm dt={dt*1000:.2f}ms delta=[{dp[0]:+.1f},{dp[1]:+.1f},{dp[2]:+.1f}]mm equiv={mag/max(dt,1e-9):.1f}mm/s")

bykf={I(r,"keyframe"):r for r in back}
print("\n================ BACKEND WINDOW ================")
print(" KF   dPmm   dtms   speed(mm/s)        P[mm]                         V[mm/s]                  RPY[deg]")
for k in range(max(min(bykf),kf-8),min(max(bykf),kf+8)+1):
    r=bykf.get(k); rp=bykf.get(k-1)
    if not r or not rp: continue
    p=[F(r,"px_m"),F(r,"py_m"),F(r,"pz_m")]
    pp=[F(rp,"px_m"),F(rp,"py_m"),F(rp,"pz_m")]
    dd=norm([(p[j]-pp[j])*1000 for j in range(3)])
    dtt=(I(r,"timestamp_ns")-I(rp,"timestamp_ns"))*1e-6
    v=[F(r,"vx_m_s")*1000,F(r,"vy_m_s")*1000,F(r,"vz_m_s")*1000]
    rpy=[F(r,"roll_deg"),F(r,"pitch_deg"),F(r,"yaw_deg")]
    mark=" <<<" if k==kf else ""
    print(f"{k:3d} {dd:7.2f} {dtt:7.1f} {F(r,'speed_m_s')*1000:11.2f}  [{p[0]*1000:+8.1f},{p[1]*1000:+8.1f},{p[2]*1000:+8.1f}]  [{v[0]:+8.1f},{v[1]:+8.1f},{v[2]:+8.1f}]  [{rpy[0]:+6.2f},{rpy[1]:+6.2f},{rpy[2]:+6.2f}]{mark}")

if front:
    keys=[r for r in front if I(r,"is_keyframe")==1]
    print("\n================ FRONTEND MATCHED BY TIMESTAMP ================")
    print(" BKF  FFID  dt(ms) status          tracked inliers/putatives ratio")
    for k in range(max(min(bykf),kf-8),min(max(bykf),kf+8)+1):
        br=bykf.get(k)
        if not br: continue
        ts=I(br,"timestamp_ns")
        fr=min(keys,key=lambda r:abs(I(r,"timestamp_ns")-ts))
        dms=(I(fr,"timestamp_ns")-ts)*1e-6
        print(f"{k:4d} {I(fr,'frame_id'):5d} {dms:+7.2f} {fr.get('mono_status',''):14s} {I(fr,'tracked_features'):7d} {I(fr,'mono_inliers'):4d}/{I(fr,'mono_putatives'):4d} {F(fr,'mono_inlier_ratio'):5.3f}")

if os.path.exists(IMU):
    imu=read(IMU)
    # Find likely timestamp column.
    fields=list(imu[0].keys()) if imu else []
    tcol=next((x for x in fields if x in ("timestamp_ns","imu_timestamp_ns","mapped_timestamp_ns","fc_timestamp_ns")),None)
    if tcol:
        t0=I(bykf[kf-1],"timestamp_ns"); t1=I(bykf[kf],"timestamp_ns")
        seg=[r for r in imu if t0<=I(r,tcol)<=t1]
        print("\n================ IMU TIMING ACROSS JUMP ================")
        print(f"timestamp column={tcol} rows_between_KF={len(seg)}")
        if len(seg)>1:
            gaps=[(I(y,tcol)-I(x,tcol))*1e-6 for x,y in zip(seg,seg[1:])]
            print(f"gap mean={sum(gaps)/len(gaps):.3f}ms max={max(gaps):.3f}ms")
    else:
        print("\nIMU timing: timestamp column not recognized:", fields)

print("\nNOTE: this script uses the CURRENT backend/frontend CSV files and does not depend on jtzero_kimera_chain.csv.")
