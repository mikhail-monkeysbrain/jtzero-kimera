#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics
from collections import defaultdict

ROOT=Path("/home/vio/jtzero_runs")
cand=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5"))
if not cand:
    cand=sorted(ROOT.glob("*TBS_ROLD_7P5*"))
if not cand:
    raise SystemExit("Не найден V27 TBS_ROLD_7P5 архив")
R=cand[-1]

def f(x):
    try:return float(x)
    except:return float("nan")
def i(x):
    try:return int(float(x))
    except:return 0
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def rms(v):
    v=[x for x in v if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in v)/len(v)) if v else float("nan")
def span(v):
    v=[x for x in v if math.isfinite(x)]
    return max(v)-min(v) if v else float("nan")
def read(name):
    with (R/name).open() as fh:return list(csv.DictReader(fh))

backend=read("jtzero_500mm_v25_backend.csv")
front=read("jtzero_500mm_v25_frontend.csv")
events=read("jtzero_500mm_v25_events.csv")
imu=read("jtzero_500mm_v25.csv")
cam=read("jtzero_500mm_v25_camera.csv")

E=defaultdict(dict)
for e in events:
    n=i(e["leg"])
    if 1<=n<=4:E[n][e["event"]]=e

def sl(rows,t0,t1,key):
    return [r for r in rows if t0 <= i(r[key]) <= t1]

def nearest_front(ts):
    if not front:return None
    return min(front,key=lambda r:abs(i(r["timestamp_ns"])-ts))

print("="*154)
print("V27 ERROR PHASE DECOMPOSITION — WHERE THE HORIZONTAL ERROR IS CREATED")
print("="*154)
print(f"RUN: {R}")
print("Фазы определяются по нормированному времени каждого измеряемого прохода: START 0-25%, MID 25-75%, END 75-100%.")
print("Это не предполагает постоянную скорость; цель — локализовать накопление ошибки и связанные диагностические изменения.\n")

allrows=[]
for n in range(1,5):
    e0,e1=E[n]["START"],E[n]["END"]
    t0=i(e0["state_timestamp_ns"]); t1=i(e1["state_timestamp_ns"])
    b=sl(backend,t0,t1,"timestamp_ns")
    if len(b)<2:continue
    direction="A_TO_B" if n%2 else "B_TO_A"
    sign=1 if n%2 else -1
    x0=f(b[0]["px_m"]); y0=f(b[0]["py_m"])
    total=math.hypot(f(b[-1]["px_m"])-x0,f(b[-1]["py_m"])-y0)*1000
    print("\n"+"-"*154)
    print(f"PASS {n} {direction}: final={total:.2f} mm error={total-500:+.2f} mm")
    print("-"*154)
    print("PHASE   t%      ΔXcanon_mm  ΔYcanon_mm  XYpath_mm  meanV  HACC_RMS  GYRO_RMS  inlier  track  mono_valid  PIM_dP  PIM_dV  ΔbA")
    phase_defs=[("START",0.00,0.25),("MID",0.25,0.75),("END",0.75,1.00)]
    for name,a,bp in phase_defs:
        q0=int(t0+(t1-t0)*a); q1=int(t0+(t1-t0)*bp)
        br=sl(backend,q0,q1,"timestamp_ns")
        fr=sl(front,q0,q1,"timestamp_ns")
        im=sl(imu,q0,q1,"mapped_ns")
        if len(br)<2:continue
        dx=sign*(f(br[-1]["px_m"])-f(br[0]["px_m"]))*1000
        dy=sign*(f(br[-1]["py_m"])-f(br[0]["py_m"]))*1000
        path=0.0
        for u,v in zip(br[:-1],br[1:]):
            path+=math.hypot(f(v["px_m"])-f(u["px_m"]),f(v["py_m"])-f(u["py_m"]))*1000
        speed=[f(r["speed_m_s"]) for r in br]
        ah=[math.hypot(f(r["ax"]),f(r["ay"])) for r in im]
        gn=[math.sqrt(f(r["gx"])**2+f(r["gy"])**2+f(r["gz"])**2) for r in im]
        inlr=[f(r["mono_inlier_ratio"]) for r in fr]
        tracked=[f(r["tracked_features"]) for r in fr]
        valid=[f(r["mono_pose_valid"]) for r in fr]
        pim=[r for r in fr if i(r.get("pim_valid",0))==1]
        pd=[math.sqrt(f(r["pim_dpx"])**2+f(r["pim_dpy"])**2+f(r["pim_dpz"])**2) for r in pim]
        dv=[math.sqrt(f(r["pim_dvx"])**2+f(r["pim_dvy"])**2+f(r["pim_dvz"])**2) for r in pim]
        if br:
            bax0,bay0,baz0=[f(br[0][k]) for k in ("bax","bay","baz")]
            bax1,bay1,baz1=[f(br[-1][k]) for k in ("bax","bay","baz")]
            dba=math.sqrt((bax1-bax0)**2+(bay1-bay0)**2+(baz1-baz0)**2)
        else:dba=float("nan")
        print(f"{name:<6} {a*100:3.0f}-{bp*100:3.0f}"
              f" {dx:+11.2f} {dy:+11.2f} {path:10.2f}"
              f" {mean(speed):6.3f} {rms(ah):9.4f} {rms(gn):9.5f}"
              f" {mean(inlr):7.3f} {mean(tracked):6.1f} {mean(valid):10.3f}"
              f" {mean(pd):7.4f} {mean(dv):7.4f} {dba:7.4f}")
        allrows.append((n,direction,name,dx,dy,path,mean(speed),rms(ah),rms(gn),mean(inlr),mean(tracked),mean(valid),mean(pd),mean(dv),dba,total-500))

print("\n"+"="*154)
print("CROSS-PASS PHASE SUMMARY")
print("="*154)
print("PHASE   mean_dXcanon  std_dXcanon  mean_path  mean_HACC  mean_GYRO  mean_inlier  mean_track  mean_PIMdP  mean_ΔbA")
for ph in ("START","MID","END"):
    rr=[r for r in allrows if r[2]==ph]
    vals=[r[3] for r in rr]
    s=statistics.pstdev(vals) if len(vals)>1 else 0.0
    print(f"{ph:<6} {mean(vals):13.2f} {s:12.2f} {mean([r[5] for r in rr]):10.2f}"
          f" {mean([r[7] for r in rr]):10.4f} {mean([r[8] for r in rr]):10.5f}"
          f" {mean([r[9] for r in rr]):11.3f} {mean([r[10] for r in rr]):10.1f}"
          f" {mean([r[12] for r in rr]):11.4f} {mean([r[14] for r in rr]):9.4f}")

print("\n"+"="*154)
print("HYPOTHESES THIS SINGLE ANALYSIS CAN SEPARATE")
print("="*154)
print("H1 Error created mainly during acceleration: compare START ΔX spread and diagnostics.")
print("H2 Error created mainly during braking/endpoint capture: compare END ΔX spread and diagnostics.")
print("H3 Error accumulates during central motion: compare MID ΔX spread.")
print("H4 Visual degradation drives error: look for low inlier/track/mono_valid in the phase with largest spread.")
print("H5 IMU excitation drives error: look for HACC/GYRO growth in the phase with largest spread.")
print("H6 IMU prediction disagreement drives error: compare PIM_dP/PIM_dV by phase.")
print("H7 Accelerometer offset estimate is absorbing motion: compare ΔbA by phase.")
print("H8 Directional effect: compare same phase in A_TO_B vs B_TO_A.")
print("H9 Extra path/oscillation: XYpath much larger than |ΔX,ΔY| in the problematic phase.")
print("H10 Endpoint/settling hypothesis: large END contribution or bias change with otherwise similar MID.")
