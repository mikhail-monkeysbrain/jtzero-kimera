#!/usr/bin/env python3
import csv, math, os, statistics
from collections import defaultdict

HOME="/home/vio"
LEGS=HOME+"/jtzero_500mm_v13_legs.csv"
BACKEND=HOME+"/jtzero_500mm_v13_backend.csv"
CAM=HOME+"/jtzero_500mm_v13_camera.csv"
IMU=HOME+"/jtzero_500mm_v13.csv"
ATT=HOME+"/jtzero_500mm_v13_attitude.csv"
RANGE=HOME+"/jtzero_500mm_v13_range.csv"
OUT=HOME+"/jtzero_v14_leg_diagnostics.csv"

def read_csv(path):
    with open(path,newline="") as f:
        return list(csv.DictReader(f))

def f(row,*keys,default=0.0):
    for k in keys:
        if k in row and row[k] not in ("",None):
            try:return float(row[k])
            except:pass
    return default

def i(row,*keys,default=0):
    for k in keys:
        if k in row and row[k] not in ("",None):
            try:return int(float(row[k]))
            except:pass
    return default

def wrap_deg(x):
    while x>180:x-=360
    while x<-180:x+=360
    return x

backend=read_csv(BACKEND)
legs=read_csv(LEGS)
cam=read_csv(CAM)
imu=read_csv(IMU)
att=read_csv(ATT)
rng=read_csv(RANGE)

kf_to_ts={i(r,"keyframe"):i(r,"timestamp_ns") for r in backend}
backend_sorted=sorted(backend,key=lambda r:i(r,"timestamp_ns"))
cam_sorted=sorted(cam,key=lambda r:i(r,"corrected_timestamp_ns","raw_timestamp_ns"))
imu_sorted=sorted(imu,key=lambda r:i(r,"mapped_ns","source_ns","recv_ns"))
att_sorted=sorted(att,key=lambda r:i(r,"recv_ns"))
rng_sorted=sorted(rng,key=lambda r:i(r,"recv_ns"))

def window(rows,t0,t1,keyfn):
    return [r for r in rows if t0 <= keyfn(r) <= t1]

def median_or_nan(xs):
    return statistics.median(xs) if xs else float("nan")

rows=[]
for L in legs:
    leg=i(L,"leg")
    sk=i(L,"start_settled_kf")
    ek=i(L,"end_settled_kf")
    t0=kf_to_ts.get(sk)
    t1=kf_to_ts.get(ek)
    if t0 is None or t1 is None:
        print(f"WARN leg {leg}: missing backend timestamps {sk}->{ek}")
        continue

    bw=window(backend_sorted,t0,t1,lambda r:i(r,"timestamp_ns"))
    cw=window(cam_sorted,t0,t1,lambda r:i(r,"corrected_timestamp_ns","raw_timestamp_ns"))
    iw=window(imu_sorted,t0,t1,lambda r:i(r,"mapped_ns","source_ns","recv_ns"))
    aw=window(att_sorted,t0,t1,lambda r:i(r,"recv_ns"))
    rw=window(rng_sorted,t0,t1,lambda r:i(r,"recv_ns"))

    speeds=[]
    jumps=[]
    prev=None
    for r in bw:
        vx,vy,vz=f(r,"vx_m_s"),f(r,"vy_m_s"),f(r,"vz_m_s")
        speeds.append(math.sqrt(vx*vx+vy*vy+vz*vz))
        if prev is not None:
            p0=(f(prev,"px_m"),f(prev,"py_m"),f(prev,"pz_m"))
            p1=(f(r,"px_m"),f(r,"py_m"),f(r,"pz_m"))
            dp=math.sqrt(sum((b-a)**2 for a,b in zip(p0,p1)))
            dt=(i(r,"timestamp_ns")-i(prev,"timestamp_ns"))*1e-9
            if dp>=0.08:
                jumps.append((i(r,"keyframe"),dp,dt,dp/dt if dt>0 else float("inf")))
        prev=r

    cam_ts=[i(r,"corrected_timestamp_ns","raw_timestamp_ns") for r in cw]
    cam_gaps=[(b-a)*1e-6 for a,b in zip(cam_ts,cam_ts[1:]) if b>a]
    cam_selected=sum(i(r,"selected")!=0 for r in cw)

    imu_ts=[i(r,"mapped_ns","source_ns","recv_ns") for r in iw]
    imu_gaps=[(b-a)*1e-6 for a,b in zip(imu_ts,imu_ts[1:]) if b>a]

    rolls=[f(r,"roll_deg") for r in aw]
    pitches=[f(r,"pitch_deg") for r in aw]
    yaws=[f(r,"yaw_deg") for r in aw]
    rdelta=pdelta=ydelta=float("nan")
    if aw:
        rdelta=rolls[-1]-rolls[0]
        pdelta=pitches[-1]-pitches[0]
        ydelta=wrap_deg(yaws[-1]-yaws[0])

    vert=[f(r,"vertical_m") for r in rw if f(r,"vertical_m")>0]

    xy=f(L,"horizontal_m")*1000
    dz=f(L,"dz_m")*1000

    rows.append({
        "leg":leg,
        "direction":L.get("direction",""),
        "xy_mm":xy,
        "xy_error_mm":xy-500.0,
        "dz_mm":dz,
        "duration_s":(t1-t0)*1e-9,
        "backend_states":len(bw),
        "peak_speed_mm_s":max(speeds)*1000 if speeds else float("nan"),
        "backend_jumps_ge80mm":len(jumps),
        "max_backend_jump_mm":max((x[1] for x in jumps),default=0)*1000,
        "max_backend_jump_speed_mm_s":max((x[3] for x in jumps),default=0)*1000,
        "camera_rows":len(cw),
        "camera_selected":cam_selected,
        "max_camera_gap_ms":max(cam_gaps) if cam_gaps else float("nan"),
        "median_camera_gap_ms":median_or_nan(cam_gaps),
        "imu_rows":len(iw),
        "max_imu_gap_ms":max(imu_gaps) if imu_gaps else float("nan"),
        "median_imu_gap_ms":median_or_nan(imu_gaps),
        "att_rows":len(aw),
        "droll_deg":rdelta,
        "dpitch_deg":pdelta,
        "dyaw_deg":ydelta,
        "range_rows":len(rw),
        "range_vertical_mean_m":sum(vert)/len(vert) if vert else float("nan"),
        "range_vertical_std_mm":statistics.pstdev(vert)*1000 if len(vert)>1 else 0.0 if vert else float("nan"),
    })

fields=list(rows[0].keys()) if rows else []
if rows:
    with open(OUT,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)

print("================ V14 LEG DIAGNOSTICS ================")
for r in rows:
    mark="  <-- OUTLIER" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: XY={r["xy_mm"]:.2f} err={r["xy_error_mm"]:+.2f} dz={r["dz_mm"]:+.2f} '
          f'dur={r["duration_s"]:.2f}s peakV={r["peak_speed_mm_s"]:.1f}mm/s jumps={r["backend_jumps_ge80mm"]} '
          f'maxJump={r["max_backend_jump_mm"]:.1f}mm camSel={r["camera_selected"]} camGapMax={r["max_camera_gap_ms"]:.2f}ms '
          f'imuGapMax={r["max_imu_gap_ms"]:.2f}ms dRPY=[{r["droll_deg"]:+.2f},{r["dpitch_deg"]:+.2f},{r["dyaw_deg"]:+.2f}] '
          f'range={r["range_vertical_mean_m"]:.3f}m±{r["range_vertical_std_mm"]:.1f}mm{mark}')

print("\n================ LEG2 vs LEG4/LEG6 ================")
base=[r for r in rows if r["leg"] in (4,6)]
l2=next((r for r in rows if r["leg"]==2),None)
if l2 and base:
    for k in ("duration_s","peak_speed_mm_s","backend_jumps_ge80mm","max_backend_jump_mm","camera_selected",
              "max_camera_gap_ms","max_imu_gap_ms","droll_deg","dpitch_deg","dyaw_deg","range_vertical_mean_m","range_vertical_std_mm"):
        vals=[r[k] for r in base if isinstance(r[k],(int,float)) and math.isfinite(float(r[k]))]
        if vals and math.isfinite(float(l2[k])):
            m=sum(vals)/len(vals)
            print(f'{k}: LEG2={l2[k]:.4f} normal_mean={m:.4f} delta={l2[k]-m:+.4f}')
print("Saved:",OUT)
