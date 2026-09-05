#!/usr/bin/env python3
import csv, math, os

H="/home/vio"
BACK=H+"/jtzero_500mm_v15_backend.csv"
LEGS=H+"/jtzero_500mm_v15_legs.csv"
OUT=H+"/jtzero_v16_min_inliers_ranked.csv"
THRESH=[10,12,15,20,25,30]

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

be=read(BACK)
legs=read(LEGS)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

def states(path):
    rr=read(path)
    return [(iv(r,"timestamp_ns"),fv(r,"px_m"),fv(r,"py_m"),fv(r,"pz_m")) for r in rr]
def near(s,t): return min(s,key=lambda q:abs(q[0]-t))

rows=[]
for n in THRESH:
    p=f"{H}/jtzero_extrinsics_replay_v10_V15_MIN{n}.csv"
    if not os.path.exists(p): continue
    s=states(p)
    vals=[]
    for L in legs:
        t0=kf_ts[iv(L,"start_settled_kf")]
        t1=kf_ts[iv(L,"end_settled_kf")]
        a,b=near(s,t0),near(s,t1)
        dx,dy,dz=b[1]-a[1],b[2]-a[2],b[3]-a[3]
        xy=math.hypot(dx,dy)*1000
        vals.append((iv(L,"leg"),xy,dz*1000))
    errs=[v[1]-500 for v in vals]
    zr=math.sqrt(sum(v[2]*v[2] for v in vals)/len(vals))
    xr=math.sqrt(sum(e*e for e in errs)/len(errs))
    worst=max(abs(e) for e in errs)
    p0,pn=s[0],s[-1]
    final=1000*math.sqrt((pn[1]-p0[1])**2+(pn[2]-p0[2])**2+(pn[3]-p0[3])**2)
    rows.append({
      "min_inliers":n,
      "xy_rms_mm":xr,
      "z_rms_mm":zr,
      "worst_leg_error_mm":worst,
      "final_dp_mm":final,
      "score":xr+zr+worst,
      **{f"leg{leg}_xy_mm":xy for leg,xy,dz in vals}
    })

rows.sort(key=lambda r:(r["score"],r["worst_leg_error_mm"]))
if rows:
    with open(OUT,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V16 minNrMonoInliers SWEEP ================")
print("rank minInl  score   XY_RMS  Z_RMS  worstErr  finalDP   L1     L2     L3     L4     L5     L6")
for i,r in enumerate(rows,1):
    print(f'{i:>2} {r["min_inliers"]:>6} {r["score"]:>7.1f} {r["xy_rms_mm"]:>8.1f} '
          f'{r["z_rms_mm"]:>6.1f} {r["worst_leg_error_mm"]:>9.1f} {r["final_dp_mm"]:>8.1f} '
          f'{r["leg1_xy_mm"]:>6.1f} {r["leg2_xy_mm"]:>6.1f} {r["leg3_xy_mm"]:>6.1f} '
          f'{r["leg4_xy_mm"]:>6.1f} {r["leg5_xy_mm"]:>6.1f} {r["leg6_xy_mm"]:>6.1f}')
print("Saved:",OUT)
