#!/usr/bin/env python3
import argparse,csv,math,subprocess,sys
from pathlib import Path

def load(p):
    if not p.exists(): return []
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d

def leg_summary(run):
    q=load(run/"jtzero_500mm_v25_legs.csv")
    if not q:return {}
    r=q[0]
    return dict(horizontal_mm=F(r,"horizontal_m")*1000,scale=F(r,"scale_horizontal"),dz_mm=F(r,"dz_m")*1000)

def cam_summary(run):
    q=load(run/"jtzero_v43_camera_forensic.csv")
    if not q:return {}
    r=q[-1]
    return dict(cam_mm=math.hypot(F(r,"net_x_m"),F(r,"net_y_m"))*1000)

def run(cmd):
    print("\n$ "+" ".join(map(str,cmd)),flush=True)
    rc=subprocess.call(cmd)
    if rc: raise SystemExit(rc)

ap=argparse.ArgumentParser()
ap.add_argument("--reference",required=True)
ap.add_argument("--bad",required=True)
ap.add_argument("--gated1",required=True)
ap.add_argument("--current",required=True)
a=ap.parse_args()
R={k:Path(v).expanduser() for k,v in vars(a).items()}
for k,p in R.items():
    if not p.is_dir(): raise SystemExit(f"missing {k}: {p}")

print("="*118)
print("V44.24 — 482-MM RUN MULTI-HYPOTHESIS CLOSURE (NO NEW PHYSICAL RUN)")
print("="*118)
for name in ("reference","bad","gated1","current"):
    L=leg_summary(R[name]); C=cam_summary(R[name])
    print(f"{name:9s}: Kimera={L.get('horizontal_mm',float('nan')):7.2f}mm "
          f"scale={L.get('scale',float('nan')):7.4f} dz={L.get('dz_mm',float('nan')):+7.2f}mm "
          f"camera={C.get('cam_mm',float('nan')):7.2f}mm")

cur=leg_summary(R["current"])
if cur:
    print(f"\ncurrent error vs 500mm = {cur['horizontal_mm']-500:+.2f}mm")
    print("Gate telemetry from V44.23: evaluated=26 rejected=0, max_jump=3.586deg, max_tilt=7.120deg.")
    print("Therefore any improvement in this run is NOT a gate-rejection effect.")

py=sys.executable; root=Path(__file__).resolve().parent
print("\n===== A. REFERENCE vs CURRENT — CROSS-RUN SCREEN =====")
run([py,str(root/"analyze_v44_11_kimera_regression_crossrun.py"),"--reference",str(R["reference"]),"--current",str(R["current"])])
print("\n===== B. REFERENCE vs CURRENT — VALID POSE DISCONTINUITY =====")
run([py,str(root/"analyze_v44_15_pose_discontinuity.py"),"--reference",str(R["reference"]),"--current",str(R["current"])])
print("\n===== C. REFERENCE vs CURRENT — LATE REVERSAL =====")
run([py,str(root/"analyze_v44_13_backend_reversal.py"),"--reference",str(R["reference"]),"--current",str(R["current"]),"--tail-start","0.65"])
print("\n===== D. REFERENCE vs CURRENT — GATE CANDIDATE SCREEN =====")
run([py,str(root/"analyze_v44_16_pose_pim_gate_screen.py"),"--reference",str(R["reference"]),"--current",str(R["current"])])

print("\n"+"="*118)
print("DECISION RULE")
print("="*118)
print("1) rejected=0 means do not credit the gate for the ~482mm endpoint.")
print("2) If current has no large VALID jump and little/no late reversal, the 417mm run was a transient estimator/front-end failure mode.")
print("3) If camera motion/excitation differs materially, motion-profile/run-to-run variance remains a confounder.")
print("4) Only a repeatable residual common to clean runs should drive the next production correction.")
print("="*118)
