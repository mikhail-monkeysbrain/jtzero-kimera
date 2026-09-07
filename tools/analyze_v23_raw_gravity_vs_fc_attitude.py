#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path
if len(sys.argv)<2:
 print("usage: analyze_v23_raw_gravity_vs_fc_attitude.py RUN [RUN ...]"); sys.exit(2)
mean=lambda x: statistics.mean(x) if x else float("nan")
med=lambda x: statistics.median(x) if x else float("nan")
def norm(v): return math.sqrt(sum(x*x for x in v))
def angle(a,b):
 c=sum(x*y for x,y in zip(a,b))/(norm(a)*norm(b))
 return math.degrees(math.acos(max(-1.,min(1.,c))))
def vm(rr,ks): return tuple(mean([float(r[k]) for r in rr]) for k in ks)
def wrap(x): return (x+180.)%360.-180.
def phase_window(b,l,p):
 rr=[r for r in b if int(r["leg"])==l and r["phase"]==p]
 t=[int(r["timestamp_ns"]) for r in rr]
 return (min(t),max(t)) if t else None
def central(rr,key,w):
 rr=sorted([r for r in rr if w[0]<=int(r[key])<=w[1]],key=lambda r:int(r[key]))
 k=len(rr)//4
 return rr[k:len(rr)-k] if len(rr)-2*k>=3 else rr
allr=[]
print("================ V23 RAW GRAVITY vs FC ATTITUDE ================")
print("Central 50% of stationary SETTLE_START/SETTLE_END windows.")
print("Raw accel-vector angular change is invariant to verified FRD->FLU sign transform.")
for a in sys.argv[1:]:
 run=Path(a)
 def rd(n):
  with (run/n).open() as f:return list(csv.DictReader(f))
 imu=rd("jtzero_500mm_v23.csv"); back=rd("jtzero_500mm_v23_backend.csv")
 fc=rd("jtzero_500mm_v23_attitude.csv"); legs=rd("jtzero_500mm_v23_legs.csv")
 print("\nRUN:",run)
 for lr in legs:
  l=int(lr["leg"]); d=lr["direction"]; w0=phase_window(back,l,"SETTLE_START"); w1=phase_window(back,l,"SETTLE_END")
  if not w0 or not w1: continue
  i0=central(imu,"mapped_ns",w0); i1=central(imu,"mapped_ns",w1)
  f0=central(fc,"mapped_ns",w0); f1=central(fc,"mapped_ns",w1)
  if not i0 or not i1 or not f0 or not f1:
   print(f"LEG {l} {d}: insufficient samples"); continue
  a0=vm(i0,("ax","ay","az")); a1=vm(i1,("ax","ay","az")); gt=angle(a0,a1)
  q0=vm(f0,("roll_deg","pitch_deg","yaw_deg")); q1=vm(f1,("roll_deg","pitch_deg","yaw_deg"))
  dr,dp,dy=[wrap(q1[j]-q0[j]) for j in range(3)]; ft=math.hypot(dr,dp); ratio=gt/ft if ft>1e-9 else float("nan")
  print(f"LEG {l} {d}: raw={len(i0)}/{len(i1)} FC={len(f0)}/{len(f1)}")
  print(f"  a0=[{a0[0]:+.5f},{a0[1]:+.5f},{a0[2]:+.5f}] |a0|={norm(a0):.6f}")
  print(f"  a1=[{a1[0]:+.5f},{a1[1]:+.5f},{a1[2]:+.5f}] |a1|={norm(a1):.6f}")
  print(f"  RAW gravity-vector angular change={gt:.4f} deg")
  print(f"  FC dRPY=[{dr:+.4f},{dp:+.4f},{dy:+.4f}] |dTilt|={ft:.4f} deg")
  print(f"  RAW/FC ratio={ratio:.3f} difference={gt-ft:+.4f} deg")
  allr.append((d,gt,ft,ratio))
print("\n================ DIRECTION SUMMARY ================")
for d in ("A->B","B->A"):
 rr=[x for x in allr if x[0]==d]
 if rr: print(f"{d}: n={len(rr)} RAW median={med([x[1] for x in rr]):.4f} deg FC median={med([x[2] for x in rr]):.4f} deg RAW/FC={med([x[3] for x in rr]):.3f}")
print("\n================ DECISION ================")
print("- RAW ~= FC repeatably: strong evidence for real/equivalent static IMU gravity-vector rotation.")
print("- RAW << FC: static gravity direction does not explain FC attitude; investigate FC estimator dynamics/history.")
print("- This cannot distinguish whole-rig tilt from local FC/IMU mechanical flex.")
