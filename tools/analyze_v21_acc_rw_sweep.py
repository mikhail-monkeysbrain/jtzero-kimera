#!/usr/bin/env python3
import csv, math, os, re

H="/home/vio"
LEGS=H+"/jtzero_500mm_v18_legs.csv"
BACK=H+"/jtzero_500mm_v18_backend.csv"
OUT=H+"/jtzero_v21_acc_rw_ranked.csv"

VALUES=[
 ("0.030","0p030"),
 ("0.015","0p015"),
 ("0.010","0p010"),
 ("0.005","0p005"),
 ("0.003","0p003"),
 ("0.001","0p001"),
]

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS)
be=read(BACK)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

def states(path):
    rr=read(path)
    return [(iv(r,"timestamp_ns"),fv(r,"px_m"),fv(r,"py_m"),fv(r,"pz_m")) for r in rr]
def near(s,t):return min(s,key=lambda q:abs(q[0]-t))

rows=[]
for label,tag in VALUES:
    # replay output naming inherited from replay_mono_imu_tbs_sweep_v12
    candidates=[
      f"{H}/jtzero_extrinsics_replay_v10_V18_ARW_{tag}.csv",
      f"{H}/jtzero_extrinsics_replay_v12_V18_ARW_{tag}.csv",
      f"{H}/jtzero_extrinsics_replay_V18_ARW_{tag}.csv",
    ]
    p=next((x for x in candidates if os.path.exists(x)),None)
    if not p:
        continue
    s=states(p)
    vals=[]
    for L in legs:
        t0=kf_ts[iv(L,"start_settled_kf")]
        t1=kf_ts[iv(L,"end_settled_kf")]
        a,b=near(s,t0),near(s,t1)
        dx,dy,dz=b[1]-a[1],b[2]-a[2],b[3]-a[3]
        xy=1000*math.hypot(dx,dy)
        vals.append((iv(L,"leg"),xy,1000*dz))
    errs=[xy-500 for _,xy,_ in vals]
    xy_rms=math.sqrt(sum(e*e for e in errs)/len(errs))
    z_rms=math.sqrt(sum(dz*dz for _,_,dz in vals)/len(vals))
    worst=max(abs(e) for e in errs)
    directional=abs(
      sum(xy for leg,xy,_ in vals if leg%2==1)/3 -
      sum(xy for leg,xy,_ in vals if leg%2==0)/3
    )
    # Penalize both RMS and session/directional instability.
    score=xy_rms + 0.5*z_rms + 0.5*worst + 0.5*directional
    row={
      "accel_random_walk":float(label),
      "score":score,"xy_rms_mm":xy_rms,"z_rms_mm":z_rms,
      "worst_leg_error_mm":worst,"directional_gap_mm":directional,
      **{f"leg{leg}_xy_mm":xy for leg,xy,dz in vals}
    }
    rows.append(row)

rows.sort(key=lambda r:r["score"])
if rows:
    with open(OUT,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V21 ACCEL RANDOM WALK SWEEP ================")
print("rank ARW      score  XY_RMS Z_RMS worstErr dirGap   L1    L2    L3    L4    L5    L6")
for i,r in enumerate(rows,1):
    print(f'{i:>2} {r["accel_random_walk"]:<8.3f} {r["score"]:>6.1f} '
          f'{r["xy_rms_mm"]:>6.1f} {r["z_rms_mm"]:>5.1f} {r["worst_leg_error_mm"]:>8.1f} '
          f'{r["directional_gap_mm"]:>6.1f} '
          f'{r["leg1_xy_mm"]:>5.1f} {r["leg2_xy_mm"]:>5.1f} {r["leg3_xy_mm"]:>5.1f} '
          f'{r["leg4_xy_mm"]:>5.1f} {r["leg5_xy_mm"]:>5.1f} {r["leg6_xy_mm"]:>5.1f}')
print("Saved:",OUT)
