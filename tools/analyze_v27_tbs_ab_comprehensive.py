#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics, sys

BASE=Path("/home/vio/jtzero_runs/20260908_195640_v25_CONTROLLED_AB4_REPEATABILITY")
ROOT=Path("/home/vio/jtzero_runs")
TRUE=500.0

def f(x):
    try:return float(x)
    except:return float("nan")
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def sd(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.pstdev(v) if len(v)>1 else 0.0
def read(p):
    with p.open() as fh:return list(csv.DictReader(fh))
def slice_t(rows,a,b,key):
    return [r for r in rows if a <= int(float(r[key])) <= b]
def span(v):
    v=[x for x in v if math.isfinite(x)]
    return max(v)-min(v) if v else float("nan")
def rms(v):
    v=[x for x in v if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in v)/len(v)) if v else float("nan")

cand=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5"))
if not cand:
    cand=sorted(ROOT.glob("*TBS_ROLD_7P5*"))
if not cand:
    raise SystemExit("Не найден архив V27 TBS_ROLD_7P5. Сначала выполните tools/run_v27_tbs_old_7p5_archived.sh")
TEST=cand[-1]

def extract(R):
    backend=read(R/"jtzero_500mm_v25_backend.csv")
    front=read(R/"jtzero_500mm_v25_frontend.csv")
    events=read(R/"jtzero_500mm_v25_events.csv")
    imu=read(R/"jtzero_500mm_v25.csv")
    cam=read(R/"jtzero_500mm_v25_camera.csv")
    att=read(R/"jtzero_500mm_v25_attitude.csv")
    rng=read(R/"jtzero_500mm_v25_range.csv")
    E={}
    for e in events:
        n=int(float(e["leg"]))
        if 1<=n<=4:E.setdefault(n,{})[e["event"]]=e
    out=[]
    for n in range(1,5):
        e0,e1=E[n]["START"],E[n]["END"]
        t0=int(e0["state_timestamp_ns"]); t1=int(e1["state_timestamp_ns"])
        w0=int(e0["event_wall_ns"]); w1=int(e1["event_wall_ns"])
        b=slice_t(backend,t0,t1,"timestamp_ns")
        fr=slice_t(front,t0,t1,"timestamp_ns")
        im=slice_t(imu,t0,t1,"mapped_ns")
        ca=slice_t(cam,t0,t1,"corrected_timestamp_ns")
        at=slice_t(att,w0,w1,"recv_ns")
        rg=slice_t(rng,w0,w1,"recv_ns")
        x0,y0,z0=[f(b[0][k]) for k in ("px_m","py_m","pz_m")]
        x1,y1,z1=[f(b[-1][k]) for k in ("px_m","py_m","pz_m")]
        dx,dy,dz=(x1-x0)*1000,(y1-y0)*1000,(z1-z0)*1000
        h=math.hypot(dx,dy); d3=math.sqrt(dx*dx+dy*dy+dz*dz)
        sign=1 if n%2 else -1
        cdx,cdy,cdz=sign*dx,sign*dy,sign*dz
        angle=math.degrees(math.atan2(cdz,math.hypot(cdx,cdy)))
        tracked=[f(r["tracked_features"]) for r in fr]
        inlr=[f(r["mono_inlier_ratio"]) for r in fr]
        valid=[f(r["mono_pose_valid"]) for r in fr]
        ax=[f(r["ax"]) for r in im]; ay=[f(r["ay"]) for r in im]
        gx=[f(r["gx"]) for r in im]; gy=[f(r["gy"]) for r in im]; gz=[f(r["gz"]) for r in im]
        ah=[math.hypot(a,b) for a,b in zip(ax,ay)]
        gn=[math.sqrt(a*a+b*b+c*c) for a,b,c in zip(gx,gy,gz)]
        cts=[int(r["corrected_timestamp_ns"]) for r in ca if int(float(r.get("selected",1)))==1]
        gaps=[(b-a)/1e6 for a,b in zip(cts[:-1],cts[1:]) if b>a]
        out.append({
          "leg":n,"direction":"A_TO_B" if n%2 else "B_TO_A",
          "duration_s":(w1-w0)/1e9,"horizontal_mm":h,"error_mm":h-TRUE,"dz_canon_mm":cdz,
          "tilt_deg":angle,"norm3d_mm":d3,"norm3d_error_mm":d3-TRUE,
          "tracked_mean":mean(tracked),"inlier_mean":mean(inlr),"mono_valid_frac":mean(valid),
          "imu_hacc_rms":rms(ah),"imu_gyro_rms":rms(gn),
          "backend_pitch_span":span([f(r["pitch_deg"]) for r in b]),
          "backend_roll_span":span([f(r["roll_deg"]) for r in b]),
          "fc_pitch_span":span([f(r["pitch_deg"]) for r in at]),
          "fc_roll_span":span([f(r["roll_deg"]) for r in at]),
          "range_span_m":span([f(r["vertical_m"]) for r in rg]),
          "camera_gap_max_ms":max(gaps) if gaps else float("nan"),
        })
    return out

base=extract(BASE); test=extract(TEST)

print("="*128)
print("V27 COMPREHENSIVE T_BS A/B — ONE-RUN MULTI-HYPOTHESIS CHECK")
print("="*128)
print(f"BASE: {BASE}")
print(f"TEST: {TEST}")
print()
print("LEG DIR    BASE_H  TEST_H   ΔH     BASE_Zc TEST_Zc  ΔZc    BASE_ang TEST_ang  BASE_err TEST_err")
for a,b in zip(base,test):
    print(f"{a['leg']:>3} {a['direction']:<6} {a['horizontal_mm']:7.2f} {b['horizontal_mm']:7.2f}"
          f" {b['horizontal_mm']-a['horizontal_mm']:+7.2f}"
          f" {a['dz_canon_mm']:+8.2f} {b['dz_canon_mm']:+8.2f} {b['dz_canon_mm']-a['dz_canon_mm']:+7.2f}"
          f" {a['tilt_deg']:+9.3f} {b['tilt_deg']:+8.3f}"
          f" {a['error_mm']:+9.2f} {b['error_mm']:+8.2f}")

metrics=["duration_s","horizontal_mm","error_mm","dz_canon_mm","tilt_deg","norm3d_error_mm",
         "tracked_mean","inlier_mean","mono_valid_frac","imu_hacc_rms","imu_gyro_rms",
         "backend_pitch_span","backend_roll_span","fc_pitch_span","fc_roll_span",
         "range_span_m","camera_gap_max_ms"]

print("\n"+"="*128)
print("MEAN METRIC CHANGE")
print("="*128)
print("METRIC                         BASE          TEST         TEST-BASE")
for k in metrics:
    x=mean([r[k] for r in base]); y=mean([r[k] for r in test])
    print(f"{k:<30} {x:12.5f} {y:12.5f} {y-x:+13.5f}")

print("\n"+"="*128)
print("DIRECTION SPLIT")
print("="*128)
for d in ("A_TO_B","B_TO_A"):
    A=[r for r in base if r["direction"]==d]; B=[r for r in test if r["direction"]==d]
    print(f"{d}:")
    print(f"  horizontal error: {mean([r['error_mm'] for r in A]):+7.2f} -> {mean([r['error_mm'] for r in B]):+7.2f} mm")
    print(f"  canonical Z:      {mean([r['dz_canon_mm'] for r in A]):+7.2f} -> {mean([r['dz_canon_mm'] for r in B]):+7.2f} mm")
    print(f"  tilt:             {mean([r['tilt_deg'] for r in A]):+7.3f} -> {mean([r['tilt_deg'] for r in B]):+7.3f} deg")

base_err=mean([abs(r["error_mm"]) for r in base]); test_err=mean([abs(r["error_mm"]) for r in test])
base_z=mean([abs(r["dz_canon_mm"]) for r in base]); test_z=mean([abs(r["dz_canon_mm"]) for r in test])
base_t=mean([abs(r["tilt_deg"]) for r in base]); test_t=mean([abs(r["tilt_deg"]) for r in test])

print("\n"+"="*128)
print("HYPOTHESIS CHECK")
print("="*128)
print(f"H1 T_BS causes false Z/tilt:   |Z| {base_z:.2f}->{test_z:.2f} mm, |angle| {base_t:.3f}->{test_t:.3f} deg")
print(f"H2 T_BS causes scale error:    mean |horizontal error| {base_err:.2f}->{test_err:.2f} mm")
print("H3 Frontend quality changes:   compare tracked_mean / inlier_mean / mono_valid_frac above")
print("H4 IMU dynamics changed:       compare imu_hacc_rms / imu_gyro_rms above")
print("H5 FC attitude changed:        compare fc_pitch_span / fc_roll_span above")
print("H6 Range geometry changed:     compare range_span_m above")
print("H7 Camera timing changed:      compare camera_gap_max_ms above")
print("H8 Direction asymmetry remains: compare A_TO_B vs B_TO_A horizontal error above")
print()
if test_z < base_z*0.35:
    print("GEOMETRY: strong improvement — T_BS rotation is a major source of false Z.")
elif test_z < base_z*0.7:
    print("GEOMETRY: partial improvement — T_BS contributes to false Z.")
else:
    print("GEOMETRY: no strong improvement — T_BS R_old did not remove false Z.")
if test_err < base_err*0.7:
    print("SCALE: substantial improvement — T_BS also materially affects horizontal scale.")
else:
    print("SCALE: no substantial improvement — main horizontal error remains elsewhere.")
