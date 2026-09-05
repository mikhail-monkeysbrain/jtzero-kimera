#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v13_legs.csv"
BACKEND=H+"/jtzero_500mm_v13_backend.csv"
CAM=H+"/jtzero_500mm_v13_camera.csv"
OUT=H+"/jtzero_v14_keyframe_cadence.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS); be=read(BACKEND); cam=read(CAM)
be=sorted(be,key=lambda r:iv(r,"timestamp_ns"))
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}
sel=sorted([r for r in cam if iv(r,"selected")!=0],key=lambda r:iv(r,"corrected_timestamp_ns"))

def pct(xs,p):
    if not xs:return float("nan")
    s=sorted(xs); q=(len(s)-1)*p/100.0; a=int(math.floor(q)); b=int(math.ceil(q))
    if a==b:return s[a]
    return s[a]*(b-q)+s[b]*(q-a)

rows=[]
for L in legs:
    leg=iv(L,"leg"); t0=kf_ts[iv(L,"start_settled_kf")]; t1=kf_ts[iv(L,"end_press_kf")]
    bw=[r for r in be if t0<=iv(r,"timestamp_ns")<=t1]
    cw=[r for r in sel if t0<=iv(r,"corrected_timestamp_ns")<=t1]
    kdt=[]; kdp=[]; kxy=[]; frames_per_kf=[]
    for a,b in zip(bw,bw[1:]):
        dt=(iv(b,"timestamp_ns")-iv(a,"timestamp_ns"))*1e-3/1e6 # ms
        dx=fv(b,"px_m")-fv(a,"px_m");dy=fv(b,"py_m")-fv(a,"py_m");dz=fv(b,"pz_m")-fv(a,"pz_m")
        kdt.append(dt);kdp.append(1000*math.sqrt(dx*dx+dy*dy+dz*dz));kxy.append(1000*math.hypot(dx,dy))
    if len(bw)>=2:
        for a,b in zip(bw,bw[1:]):
            ta,tb=iv(a,"timestamp_ns"),iv(b,"timestamp_ns")
            frames_per_kf.append(sum(1 for r in cw if ta<iv(r,"corrected_timestamp_ns")<=tb))
    duration=(t1-t0)*1e-9
    rows.append(dict(
      leg=leg,direction=L["direction"],xy_mm=fv(L,"horizontal_m")*1000,
      duration_s=duration,backend_kf=len(bw),kf_rate_hz=(len(bw)-1)/duration if duration>0 and len(bw)>1 else 0,
      kf_dt_mean_ms=statistics.mean(kdt) if kdt else float("nan"),
      kf_dt_p90_ms=pct(kdt,90),kf_dt_max_ms=max(kdt) if kdt else float("nan"),
      dp_per_kf_mean_mm=statistics.mean(kdp) if kdp else float("nan"),
      dp_per_kf_p90_mm=pct(kdp,90),dp_per_kf_max_mm=max(kdp) if kdp else float("nan"),
      xy_per_kf_mean_mm=statistics.mean(kxy) if kxy else float("nan"),
      selected_frames=len(cw),
      frames_per_kf_mean=statistics.mean(frames_per_kf) if frames_per_kf else float("nan"),
      frames_per_kf_p90=pct(frames_per_kf,90),
      frames_per_kf_max=max(frames_per_kf) if frames_per_kf else float("nan"),
    ))

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V14 KEYFRAME CADENCE ================")
for r in rows:
    mark="  <-- LEG2" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: XY={r["xy_mm"]:.2f} '
          f'KF={r["backend_kf"]} rate={r["kf_rate_hz"]:.2f}Hz '
          f'dtMean={r["kf_dt_mean_ms"]:.1f}ms dtP90={r["kf_dt_p90_ms"]:.1f}ms dtMax={r["kf_dt_max_ms"]:.1f}ms '
          f'dP/KF={r["dp_per_kf_mean_mm"]:.1f}mm p90={r["dp_per_kf_p90_mm"]:.1f} max={r["dp_per_kf_max_mm"]:.1f} '
          f'frames/KF={r["frames_per_kf_mean"]:.2f} p90={r["frames_per_kf_p90"]:.1f} max={r["frames_per_kf_max"]:.0f}{mark}')

print("\n================ B->A KEYFRAME COMPARISON ================")
for r in rows:
    if r["leg"] in (2,4,6):
        print(f'LEG {r["leg"]}: rate={r["kf_rate_hz"]:.2f}Hz dtP90={r["kf_dt_p90_ms"]:.1f}ms '
              f'dP/KF={r["dp_per_kf_mean_mm"]:.1f}mm p90={r["dp_per_kf_p90_mm"]:.1f} '
              f'frames/KF={r["frames_per_kf_mean"]:.2f}')
print("Saved:",OUT)
