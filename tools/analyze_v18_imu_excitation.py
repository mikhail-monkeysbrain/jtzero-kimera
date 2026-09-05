#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v18_legs.csv"
BACK=H+"/jtzero_500mm_v18_backend.csv"
IMU=H+"/jtzero_500mm_v18.csv"
OUT=H+"/jtzero_v18_imu_excitation.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read(LEGS)
be=read(BACK)
imu=[r for r in read(IMU) if r.get("type")=="IMU"]
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

def mean(v): return statistics.mean(v) if v else float("nan")
def rms(v): return math.sqrt(mean([x*x for x in v])) if v else float("nan")
def std(v): return statistics.pstdev(v) if len(v)>1 else 0.0

rows=[]
print("================ V18 IMU EXCITATION ================")
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    dur=(t1-t0)*1e-9
    rr=[r for r in imu if t0<=iv(r,"mapped_ns")<=t1]
    if not rr:
        continue

    ax=[fv(r,"ax") for r in rr]; ay=[fv(r,"ay") for r in rr]; az=[fv(r,"az") for r in rr]
    gx=[fv(r,"gx") for r in rr]; gy=[fv(r,"gy") for r in rr]; gz=[fv(r,"gz") for r in rr]

    # Remove each leg's mean specific-force vector. This avoids relying on
    # absolute gravity/frame convention and isolates dynamic excitation.
    mx,my,mz=mean(ax),mean(ay),mean(az)
    dax=[x-mx for x in ax]; day=[y-my for y in ay]; daz=[z-mz for z in az]
    dyn=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(dax,day,daz)]

    # Split into temporal thirds: acceleration/start, cruise, braking/end.
    thirds=[]
    for lo,hi in ((0,1/3),(1/3,2/3),(2/3,1)):
        a=t0+int((t1-t0)*lo); b=t0+int((t1-t0)*hi)
        q=[r for r in rr if a<=iv(r,"mapped_ns")<=b]
        if q:
            qax=[fv(r,"ax") for r in q]; qay=[fv(r,"ay") for r in q]; qaz=[fv(r,"az") for r in q]
            qdyn=[math.sqrt((x-mx)**2+(y-my)**2+(z-mz)**2) for x,y,z in zip(qax,qay,qaz)]
            thirds.append((rms(qdyn),max(qdyn)))
        else:
            thirds.append((float("nan"),float("nan")))

    # High-pass-ish successive sample delta, insensitive to static gravity.
    da_step=[]
    for i in range(1,len(rr)):
        da_step.append(math.sqrt((ax[i]-ax[i-1])**2+(ay[i]-ay[i-1])**2+(az[i]-az[i-1])**2))

    # Fraction of samples with meaningful dynamic acceleration above thresholds.
    frac05=sum(x>0.05 for x in dyn)/len(dyn)
    frac10=sum(x>0.10 for x in dyn)/len(dyn)
    frac20=sum(x>0.20 for x in dyn)/len(dyn)

    xy=fv(L,"horizontal_m")*1000
    row=dict(
      leg=leg,direction=L["direction"],duration_s=dur,xy_mm=xy,scale=xy/500.0,
      dyn_acc_rms=rms(dyn),dyn_acc_max=max(dyn),
      dyn_acc_start_rms=thirds[0][0],dyn_acc_mid_rms=thirds[1][0],dyn_acc_end_rms=thirds[2][0],
      dyn_acc_start_max=thirds[0][1],dyn_acc_mid_max=thirds[1][1],dyn_acc_end_max=thirds[2][1],
      accel_step_rms=rms(da_step),
      accel_std_x=std(ax),accel_std_y=std(ay),accel_std_z=std(az),
      gyro_rms_x=rms(gx),gyro_rms_y=rms(gy),gyro_rms_z=rms(gz),
      gyro_std_x=std(gx),gyro_std_y=std(gy),gyro_std_z=std(gz),
      frac_dyn_gt_005=frac05,frac_dyn_gt_010=frac10,frac_dyn_gt_020=frac20
    )
    rows.append(row)
    tag=" GOOD-REF" if leg==5 else (" BAD" if leg in (3,6) else "")
    print(f'LEG {leg} {L["direction"]}: dur={dur:.2f}s XY={xy:.1f} scale={xy/500:.3f}{tag}')
    print(f'  dynAcc RMS={row["dyn_acc_rms"]:.4f} max={row["dyn_acc_max"]:.4f} m/s²')
    print(f'  thirds RMS start/mid/end={thirds[0][0]:.4f}/{thirds[1][0]:.4f}/{thirds[2][0]:.4f}')
    print(f'  thirds MAX start/mid/end={thirds[0][1]:.4f}/{thirds[1][1]:.4f}/{thirds[2][1]:.4f}')
    print(f'  fraction >0.05/0.10/0.20 = {frac05:.3f}/{frac10:.3f}/{frac20:.3f}')
    print(f'  accelStepRMS={row["accel_step_rms"]:.5f} gyroStd=[{row["gyro_std_x"]:.5f},{row["gyro_std_y"]:.5f},{row["gyro_std_z"]:.5f}]')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("\n================ LEG5 vs LEG6 EXCITATION ================")
r5=next(r for r in rows if r["leg"]==5)
r6=next(r for r in rows if r["leg"]==6)
for k in [
  "duration_s","scale","dyn_acc_rms","dyn_acc_max",
  "dyn_acc_start_rms","dyn_acc_mid_rms","dyn_acc_end_rms",
  "dyn_acc_start_max","dyn_acc_mid_max","dyn_acc_end_max",
  "accel_step_rms","accel_std_x","accel_std_y","accel_std_z",
  "gyro_std_x","gyro_std_y","gyro_std_z",
  "frac_dyn_gt_005","frac_dyn_gt_010","frac_dyn_gt_020"
]:
    print(f'{k}: LEG5={r5[k]:.6f} LEG6={r6[k]:.6f} delta={r6[k]-r5[k]:+.6f}')

print("\nNOTE:")
print("Dynamic acceleration here is computed after subtracting each leg's mean accel vector.")
print("It is frame/sign independent enough for comparing excitation strength, but is not an earth-frame acceleration estimate.")
print("Saved:",OUT)
