#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
BACK=H+"/jtzero_500mm_v15_backend.csv"
CAM=H+"/jtzero_500mm_v15_camera.csv"
IMU=H+"/jtzero_500mm_v15.csv"
ATT=H+"/jtzero_500mm_v15_attitude.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

be=read(BACK); cam=read(CAM); imu=read(IMU); att=read(ATT)
b={iv(r,"keyframe"):r for r in be}
t0=iv(b[153],"timestamp_ns"); t1=iv(b[154],"timestamp_ns")

print("================ V17 KF153->KF154 DATA FLOW ================")
print(f"KF153 ts={t0}")
print(f"KF154 ts={t1}")
print(f"interval={(t1-t0)/1e9:.6f}s")

cw=[r for r in cam if t0<=iv(r,"corrected_timestamp_ns")<=t1]
sel=[r for r in cw if iv(r,"selected")!=0]
print("\nCAMERA:")
print(f"raw_rows={len(cw)} selected_rows={len(sel)}")
if cw:
    seq=[iv(r,"sequence") for r in cw]
    ts=[iv(r,"corrected_timestamp_ns") for r in cw]
    gaps=[(b-a)/1e6 for a,b in zip(ts,ts[1:])]
    sj=[b-a for a,b in zip(seq,seq[1:])]
    print(f"first_seq={seq[0]} last_seq={seq[-1]} seq_span={seq[-1]-seq[0]}")
    print(f"max_raw_gap_ms={max(gaps) if gaps else float('nan'):.3f}")
    print(f"sequence_jumps_gt1={sum(x!=1 for x in sj)} max_seq_jump={max(sj) if sj else 0}")
if sel:
    ts=[iv(r,"corrected_timestamp_ns") for r in sel]
    gaps=[(b-a)/1e6 for a,b in zip(ts,ts[1:])]
    print(f"selected_first_dt={(ts[0]-t0)/1e9:+.3f}s selected_last_dt={(ts[-1]-t0)/1e9:+.3f}s")
    print(f"max_selected_gap_ms={max(gaps) if gaps else float('nan'):.3f}")

iw=[r for r in imu if r.get("type")=="IMU" and t0<=iv(r,"mapped_ns")<=t1]
print("\nIMU:")
print(f"rows={len(iw)} rate={len(iw)/((t1-t0)/1e9):.1f}Hz")
if iw:
    ts=[iv(r,"mapped_ns") for r in iw]
    gaps=[(b-a)/1e6 for a,b in zip(ts,ts[1:])]
    print(f"first_dt={(ts[0]-t0)/1e9:+.6f}s last_dt={(ts[-1]-t0)/1e9:+.6f}s")
    print(f"max_gap_ms={max(gaps) if gaps else float('nan'):.3f} median_gap_ms={statistics.median(gaps) if gaps else float('nan'):.3f}")
    acc=[math.sqrt(fv(r,"ax")**2+fv(r,"ay")**2+fv(r,"az")**2) for r in iw]
    gyr=[math.sqrt(fv(r,"gx")**2+fv(r,"gy")**2+fv(r,"gz")**2) for r in iw]
    print(f"acc_norm_mean={statistics.mean(acc):.6f} std={statistics.pstdev(acc):.6f}")
    print(f"gyro_norm_mean={statistics.mean(gyr):.6f} max={max(gyr):.6f}")

aw=[r for r in att if t0<=iv(r,"recv_ns")<=t1]
print("\nATTITUDE (recv-clock diagnostic only):")
print(f"rows={len(aw)}")
if aw:
    print(f"R {fv(aw[0],'roll_deg'):.3f}->{fv(aw[-1],'roll_deg'):.3f} "
          f"P {fv(aw[0],'pitch_deg'):.3f}->{fv(aw[-1],'pitch_deg'):.3f} "
          f"Y {fv(aw[0],'yaw_deg'):.3f}->{fv(aw[-1],'yaw_deg'):.3f}")

print("\nBACKEND:")
a,bk=b[153],b[154]
dx=fv(bk,"px_m")-fv(a,"px_m");dy=fv(bk,"py_m")-fv(a,"py_m");dz=fv(bk,"pz_m")-fv(a,"pz_m")
print(f"dP=[{dx:+.4f},{dy:+.4f},{dz:+.4f}]m norm={1000*math.sqrt(dx*dx+dy*dy+dz*dz):.1f}mm")
print(f"V153=[{fv(a,'vx_m_s'):+.4f},{fv(a,'vy_m_s'):+.4f},{fv(a,'vz_m_s'):+.4f}]")
print(f"V154=[{fv(bk,'vx_m_s'):+.4f},{fv(bk,'vy_m_s'):+.4f},{fv(bk,'vz_m_s'):+.4f}]")

if len(cw)==0:
    print("\nDIAGNOSIS_HINT: no raw camera frames in the interval -> camera acquisition/feed stall.")
elif len(sel)==0:
    print("\nDIAGNOSIS_HINT: raw camera exists but no selected frames -> selection/rejection/timestamp path issue.")
elif len(iw) < 0.8*200*((t1-t0)/1e9):
    print("\nDIAGNOSIS_HINT: IMU coverage is incomplete -> inspect serial/mapping starvation.")
else:
    print("\nDIAGNOSIS_HINT: camera and IMU both continue -> gap is internal frontend/backend keyframe processing.")
