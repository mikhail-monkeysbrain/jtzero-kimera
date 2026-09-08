#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics, sys
from collections import defaultdict

RUNS = [
    ("FAST_5S", Path("/home/vio/jtzero_runs/20260908_200424_v25_CONTROLLED_AB4_FAST_5S")),
    ("MEDIUM_7P5S", Path("/home/vio/jtzero_runs/20260908_195640_v25_CONTROLLED_AB4_REPEATABILITY")),
    ("SLOW_10S", Path("/home/vio/jtzero_runs/20260908_200841_v25_CONTROLLED_AB4_SLOW_10S")),
]

TRUE_M = 0.5

def fnum(x, default=float("nan")):
    try: return float(x)
    except Exception: return default

def inum(x, default=0):
    try: return int(float(x))
    except Exception: return default

def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")

def median(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.median(v) if v else float("nan")

def rms(v):
    v=[x for x in v if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in v)/len(v)) if v else float("nan")

def std(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.pstdev(v) if len(v)>1 else 0.0 if v else float("nan")

def pct(v,q):
    v=sorted(x for x in v if math.isfinite(x))
    if not v: return float("nan")
    p=(len(v)-1)*q
    lo=int(math.floor(p)); hi=int(math.ceil(p))
    if lo==hi: return v[lo]
    a=p-lo
    return v[lo]*(1-a)+v[hi]*a

def corr(xs, ys):
    p=[(x,y) for x,y in zip(xs,ys) if math.isfinite(x) and math.isfinite(y)]
    if len(p)<3: return float("nan")
    x=[a for a,b in p]; y=[b for a,b in p]
    mx=mean(x); my=mean(y)
    sx=math.sqrt(sum((a-mx)**2 for a in x)); sy=math.sqrt(sum((b-my)**2 for b in y))
    if sx==0 or sy==0: return float("nan")
    return sum((a-mx)*(b-my) for a,b in p)/(sx*sy)

def read_csv(path):
    if not path.exists(): return []
    with path.open() as fh:
        return list(csv.DictReader(fh))

def nearest(rows, t, key="timestamp_ns"):
    if not rows: return None
    return min(rows, key=lambda r: abs(inum(r.get(key))-t))

def slice_ts(rows, t0, t1, key):
    out=[]
    for r in rows:
        t=inum(r.get(key))
        if t0 <= t <= t1: out.append(r)
    return out

def range_span(vals):
    vals=[x for x in vals if math.isfinite(x)]
    return max(vals)-min(vals) if vals else float("nan")

records=[]

for mode, R in RUNS:
    legs=read_csv(R/"jtzero_500mm_v25_legs.csv")
    backend=read_csv(R/"jtzero_500mm_v25_backend.csv")
    frontend=read_csv(R/"jtzero_500mm_v25_frontend.csv")
    events=read_csv(R/"jtzero_500mm_v25_events.csv")
    imu=read_csv(R/"jtzero_500mm_v25.csv")
    att=read_csv(R/"jtzero_500mm_v25_attitude.csv")
    rng=read_csv(R/"jtzero_500mm_v25_range.csv")
    cam=read_csv(R/"jtzero_500mm_v25_camera.csv")

    ev=defaultdict(dict)
    for e in events:
        n=inum(e.get("leg"))
        if 1 <= n <= 4:
            ev[n][e.get("event","")]=e

    for n in range(1,5):
        if n not in ev or "START" not in ev[n] or "END" not in ev[n]:
            print(f"[WARN] {mode} leg {n}: missing START/END")
            continue

        e0,e1=ev[n]["START"],ev[n]["END"]
        t0=inum(e0["state_timestamp_ns"]); t1=inum(e1["state_timestamp_ns"])
        w0=inum(e0["event_wall_ns"]); w1=inum(e1["event_wall_ns"])

        b=slice_ts(backend,t0,t1,"timestamp_ns")
        fr=slice_ts(frontend,t0,t1,"timestamp_ns")
        im=slice_ts(imu,t0,t1,"mapped_ns")
        ca=slice_ts(cam,t0,t1,"corrected_timestamp_ns")

        # attitude/range use receive wall clock
        at=slice_ts(att,w0,w1,"recv_ns")
        rg=slice_ts(rng,w0,w1,"recv_ns")

        if not b:
            continue

        x0=fnum(b[0]["px_m"]); y0=fnum(b[0]["py_m"]); z0=fnum(b[0]["pz_m"])
        x1=fnum(b[-1]["px_m"]); y1=fnum(b[-1]["py_m"]); z1=fnum(b[-1]["pz_m"])
        dx=x1-x0; dy=y1-y0; dz=z1-z0
        horizontal=math.hypot(dx,dy)
        error_mm=(horizontal-TRUE_M)*1000.0

        # Prefer legs CSV measured value if available
        legrow = next((r for r in legs if inum(r.get("leg"))==n), None)
        if legrow:
            for key in ("horizontal_m","horizontal","distance_m","measured_m"):
                if key in legrow and math.isfinite(fnum(legrow[key])):
                    horizontal=fnum(legrow[key]); error_mm=(horizontal-TRUE_M)*1000.0; break

        duration=(w1-w0)/1e9

        vx=[fnum(r["vx_m_s"]) for r in b]; vy=[fnum(r["vy_m_s"]) for r in b]; vz=[fnum(r["vz_m_s"]) for r in b]
        speed=[fnum(r["speed_m_s"]) for r in b]
        roll=[fnum(r["roll_deg"]) for r in b]; pitch=[fnum(r["pitch_deg"]) for r in b]; yaw=[fnum(r["yaw_deg"]) for r in b]
        bax=[fnum(r["bax"]) for r in b]; bay=[fnum(r["bay"]) for r in b]; baz=[fnum(r["baz"]) for r in b]
        bgx=[fnum(r["bgx"]) for r in b]; bgy=[fnum(r["bgy"]) for r in b]; bgz=[fnum(r["bgz"]) for r in b]

        # position path and speed consistency
        path_xy=0.0; int_vx=0.0; int_speed=0.0
        for a,c in zip(b[:-1],b[1:]):
            ddx=fnum(c["px_m"])-fnum(a["px_m"]); ddy=fnum(c["py_m"])-fnum(a["py_m"])
            path_xy += math.hypot(ddx,ddy)
            dt=(inum(c["timestamp_ns"])-inum(a["timestamp_ns"]))/1e9
            int_vx += 0.5*(fnum(a["vx_m_s"])+fnum(c["vx_m_s"]))*dt
            int_speed += 0.5*(fnum(a["speed_m_s"])+fnum(c["speed_m_s"]))*dt

        # frontend
        tracked=[fnum(r["tracked_features"]) for r in fr]
        inlier_ratio=[fnum(r["mono_inlier_ratio"]) for r in fr]
        put=[fnum(r["mono_putatives"]) for r in fr]
        inl=[fnum(r["mono_inliers"]) for r in fr]
        valid=[inum(r["mono_pose_valid"]) for r in fr]
        keyframes=[r for r in fr if inum(r["is_keyframe"])==1]
        pim=[r for r in fr if inum(r["pim_valid"])==1]
        pim_dt=[fnum(r["pim_dt_s"]) for r in pim]
        pim_dp=[math.sqrt(fnum(r["pim_dpx"])**2+fnum(r["pim_dpy"])**2+fnum(r["pim_dpz"])**2) for r in pim]
        pim_dv=[math.sqrt(fnum(r["pim_dvx"])**2+fnum(r["pim_dvy"])**2+fnum(r["pim_dvz"])**2) for r in pim]
        mono_t=[fnum(r["mono_body_t_norm"]) for r in fr if inum(r["mono_pose_valid"])==1 and "mono_body_t_norm" in r]
        mono_rot=[math.sqrt(fnum(r["mono_roll_deg"])**2+fnum(r["mono_pitch_deg"])**2+fnum(r["mono_yaw_deg"])**2)
                  for r in fr if inum(r["mono_pose_valid"])==1]

        # raw IMU
        ax=[fnum(r["ax"]) for r in im]; ay=[fnum(r["ay"]) for r in im]; az=[fnum(r["az"]) for r in im]
        gx=[fnum(r["gx"]) for r in im]; gy=[fnum(r["gy"]) for r in im]; gz=[fnum(r["gz"]) for r in im]
        ah=[math.hypot(a,bv) for a,bv in zip(ax,ay)]
        gn=[math.sqrt(a*a+bv*bv+c*c) for a,bv,c in zip(gx,gy,gz)]

        # FC attitude/range
        fc_roll=[fnum(r["roll_deg"]) for r in at]
        fc_pitch=[fnum(r["pitch_deg"]) for r in at]
        fc_yaw=[fnum(r["yaw_deg"]) for r in at]
        vertical=[fnum(r["vertical_m"]) for r in rg]
        current_cm=[fnum(r["current_cm"]) for r in rg]

        # camera timing
        cam_ts=[inum(r["corrected_timestamp_ns"]) for r in ca if inum(r.get("selected",1))==1]
        cam_dt=[(b-a)/1e9 for a,b in zip(cam_ts[:-1],cam_ts[1:]) if b>a]

        rec={
            "mode":mode,"leg":n,"duration_s":duration,"horizontal_mm":horizontal*1000,"error_mm":error_mm,
            "dx_mm":dx*1000,"dy_mm":dy*1000,"dz_mm":dz*1000,
            "path_xy_mm":path_xy*1000,"path_minus_net_mm":(path_xy-horizontal)*1000,
            "mean_speed":mean(speed),"max_speed":max(speed) if speed else float("nan"),
            "int_speed_mm":int_speed*1000,"int_vx_mm":int_vx*1000,
            "pos_vx_consistency_mm":(dx-int_vx)*1000,
            "backend_roll_span":range_span(roll),"backend_pitch_span":range_span(pitch),"backend_yaw_span":range_span(yaw),
            "bax_span":range_span(bax),"bay_span":range_span(bay),"baz_span":range_span(baz),
            "bgx_span":range_span(bgx),"bgy_span":range_span(bgy),"bgz_span":range_span(bgz),
            "frontend_frames":len(fr),"frontend_kf":len(keyframes),"mono_valid_frac":mean(valid),
            "tracked_mean":mean(tracked),"tracked_p10":pct(tracked,0.1),
            "inlier_ratio_mean":mean(inlier_ratio),"inlier_ratio_p10":pct(inlier_ratio,0.1),
            "putatives_mean":mean(put),"inliers_mean":mean(inl),
            "mono_body_t_mean":mean(mono_t),"mono_body_t_std":std(mono_t),
            "mono_rot_mean_deg":mean(mono_rot),
            "pim_dt_mean":mean(pim_dt),"pim_dp_mean":mean(pim_dp),"pim_dv_mean":mean(pim_dv),
            "imu_hacc_rms":rms(ah),"imu_hacc_p90":pct(ah,0.9),"imu_hacc_max":max(ah) if ah else float("nan"),
            "imu_gyro_rms":rms(gn),"imu_gyro_p90":pct(gn,0.9),"imu_gyro_max":max(gn) if gn else float("nan"),
            "imu_ax_mean":mean(ax),"imu_ay_mean":mean(ay),"imu_az_mean":mean(az),
            "imu_gx_mean":mean(gx),"imu_gy_mean":mean(gy),"imu_gz_mean":mean(gz),
            "fc_roll_span":range_span(fc_roll),"fc_pitch_span":range_span(fc_pitch),"fc_yaw_span":range_span(fc_yaw),
            "range_vertical_mean":mean(vertical),"range_vertical_span":range_span(vertical),
            "range_cm_span":range_span(current_cm),
            "camera_selected":len(cam_ts),"camera_dt_mean_ms":mean(cam_dt)*1000 if cam_dt else float("nan"),
            "camera_dt_p95_ms":pct(cam_dt,0.95)*1000 if cam_dt else float("nan"),
            "camera_gap_max_ms":max(cam_dt)*1000 if cam_dt else float("nan"),
        }
        records.append(rec)

if len(records)!=12:
    print(f"[WARN] expected 12 passes, got {len(records)}")

print("\n"+"="*140)
print("V26 12-PASS ROOT-CAUSE SCREEN")
print("="*140)
print("MODE          LEG  DUR_s   DIST_mm   ERR_mm   DZ_mm   Vmean   Vmax   INLIER  TRACK  HACC_RMS  GYRO_RMS  CAMgap")
for r in records:
    print(f"{r['mode']:<13} {r['leg']:>3d}  {r['duration_s']:6.3f}  {r['horizontal_mm']:8.2f}  {r['error_mm']:+7.2f}"
          f"  {r['dz_mm']:+7.2f}  {r['mean_speed']:6.3f}  {r['max_speed']:6.3f}"
          f"  {r['inlier_ratio_mean']:7.3f} {r['tracked_mean']:6.1f}"
          f"  {r['imu_hacc_rms']:8.4f} {r['imu_gyro_rms']:8.5f} {r['camera_gap_max_ms']:7.1f}")

# correlations against signed error and absolute error
numkeys=[k for k in records[0].keys() if k not in ("mode","leg","horizontal_mm","error_mm")]
err=[r["error_mm"] for r in records]
abserr=[abs(x) for x in err]
rank=[]
for k in numkeys:
    xs=[r[k] for r in records]
    cs=corr(xs,err); ca=corr(xs,abserr)
    rank.append((max(abs(cs) if math.isfinite(cs) else 0,abs(ca) if math.isfinite(ca) else 0),k,cs,ca))

print("\n"+"="*140)
print("CORRELATION SCREEN — exploratory only (n=12, not proof of causation)")
print("="*140)
print("FEATURE                              corr(error)   corr(|error|)")
for _,k,cs,ca in sorted(rank,reverse=True):
    if not (math.isfinite(cs) or math.isfinite(ca)): continue
    print(f"{k:<36} {cs:+11.3f}   {ca:+13.3f}")

print("\n"+"="*140)
print("GROUP SUMMARY")
print("="*140)
for mode,_ in RUNS:
    rr=[r for r in records if r["mode"]==mode]
    print(f"{mode}: n={len(rr)}  mean={mean([r['horizontal_mm'] for r in rr]):.2f} mm"
          f"  std={std([r['horizontal_mm'] for r in rr]):.2f} mm"
          f"  mean_err={mean([r['error_mm'] for r in rr]):+.2f} mm"
          f"  mean_duration={mean([r['duration_s'] for r in rr]):.3f} s"
          f"  mean_speed={mean([r['mean_speed'] for r in rr]):.4f} m/s")

# compact verdict candidates
print("\n"+"="*140)
print("TOP CANDIDATES TO INSPECT")
print("="*140)
for score,k,cs,ca in sorted(rank,reverse=True)[:15]:
    print(f"{k:<36} score={score:.3f} corr(error)={cs:+.3f} corr(|error|)={ca:+.3f}")

out=Path("/home/vio/jtzero_v26_12pass_rootcause.csv")
keys=["mode","leg","duration_s","horizontal_mm","error_mm"]+[k for k in records[0].keys() if k not in ("mode","leg","duration_s","horizontal_mm","error_mm")]
with out.open("w",newline="") as fh:
    w=csv.DictWriter(fh,fieldnames=keys)
    w.writeheader(); w.writerows(records)
print(f"\nCSV: {out}")
