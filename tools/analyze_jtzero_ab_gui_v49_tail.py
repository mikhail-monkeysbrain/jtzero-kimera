#!/usr/bin/env python3
import csv
import numpy as np
from collections import defaultdict

P="/home/vio/jtzero_ab_gui_v49.csv"
STILLS=("A_START_STILL","B_STILL","A_END_STILL")
TAIL_S=5.0
G=9.80665

rows=defaultdict(lambda: defaultdict(list))
with open(P,newline="") as f:
    for r in csv.DictReader(f):
        ph=r["phase"]
        if ph in STILLS:
            rows[ph][r["name"]].append((int(r["fc_time_ms"]),float(r["value"])))

def packets(imu,phase):
    names=[f"I{imu}CX",f"I{imu}CY",f"I{imu}CZ"]
    d=defaultdict(dict)
    for n in names:
        for t,v in rows[phase][n]:
            d[t][n]=v
    out=[]
    for t in sorted(d):
        if all(n in d[t] for n in names):
            out.append([t,d[t][names[0]],d[t][names[1]],d[t][names[2]]])
    return np.asarray(out,float)

print(f"===== V49 STILL TAIL ANALYSIS: last {TAIL_S:.1f}s =====")
for imu in (0,1):
    print("\n"+"="*72)
    print(f"IMU{imu}")
    print("="*72)
    means={}
    for ph in STILLS:
        A=packets(imu,ph)
        t=A[:,0]
        end=t[-1]
        keep=t >= end-TAIL_S*1000
        X=A[keep,1:4]
        n=np.linalg.norm(X,axis=1)
        m=np.mean(X,axis=0)
        means[ph]=m
        print(
            f"{ph:14s}: n={len(X):2d} "
            f"mean=[{m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f}] "
            f"|mean|={np.linalg.norm(m):.6f} "
            f"mean|a|={np.mean(n):.6f} std|a|={np.std(n):.6f} "
            f"RMS_g={np.sqrt(np.mean((n-G)**2)):.6f}"
        )
    a0=means["A_START_STILL"]
    b=means["B_STILL"]
    a1=means["A_END_STILL"]
    print("\n--- TAIL A/B/A ---")
    print(f"A->B vector = [{b[0]-a0[0]:+.6f} {b[1]-a0[1]:+.6f} {b[2]-a0[2]:+.6f}]")
    print(f"A closure   = [{a1[0]-a0[0]:+.6f} {a1[1]-a0[1]:+.6f} {a1[2]-a0[2]:+.6f}]")
    print(f"|closure|   = {np.linalg.norm(a1-a0):.6f} m/s^2")
    print(f"A-B norm delta       = {np.linalg.norm(b)-np.linalg.norm(a0):+.6f} m/s^2")
    print(f"A closure norm delta = {np.linalg.norm(a1)-np.linalg.norm(a0):+.6f} m/s^2")
