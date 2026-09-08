#!/usr/bin/env python3
import csv
import numpy as np
from collections import defaultdict

P="/home/vio/jtzero_ab_gui_v50.csv"
STILLS=("A_START_STILL","B_STILL","A_END_STILL")
MOVES=("A_TO_B","B_TO_A")
by=defaultdict(lambda: defaultdict(list))

with open(P,newline="") as f:
    for r in csv.DictReader(f):
        ph=r["phase"]
        if ph in STILLS+MOVES:
            by[ph][r["name"]].append((int(r["fc_time_ms"]),float(r["value"])))

def packets(imu,stage,phase):
    names=[f"I{imu}{stage}X",f"I{imu}{stage}Y",f"I{imu}{stage}Z"]
    d=defaultdict(dict)
    for n in names:
        for t,v in by[phase][n]:
            d[t][n]=v
    out=[]
    for t in sorted(d):
        if all(n in d[t] for n in names):
            out.append([t,d[t][names[0]],d[t][names[1]],d[t][names[2]]])
    return np.asarray(out,float)

print("===== V50 CLEAN PHASE ANALYSIS =====")
for imu in (0,1):
    print("\n"+"="*72)
    print(f"IMU{imu}")
    print("="*72)
    means={}
    for ph in STILLS:
        A=packets(imu,"C",ph)
        xyz=A[:,1:4]
        n=np.linalg.norm(xyz,axis=1)
        m=np.mean(xyz,axis=0)
        means[ph]=m
        dur=(A[-1,0]-A[0,0])/1000 if len(A)>1 else 0
        print(
            f"{ph:14s}: n={len(A):3d} dur={dur:5.2f}s "
            f"mean=[{m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f}] "
            f"|mean|={np.linalg.norm(m):.6f} "
            f"mean|a|={np.mean(n):.6f} std|a|={np.std(n):.6f}"
        )

    a0=means["A_START_STILL"]; b=means["B_STILL"]; a1=means["A_END_STILL"]
    print("\n--- A/B/A ---")
    print(f"A->B vector = [{b[0]-a0[0]:+.6f} {b[1]-a0[1]:+.6f} {b[2]-a0[2]:+.6f}]")
    print(f"A closure   = [{a1[0]-a0[0]:+.6f} {a1[1]-a0[1]:+.6f} {a1[2]-a0[2]:+.6f}]")
    print(f"|closure|   = {np.linalg.norm(a1-a0):.6f} m/s^2")
    print(f"A-B norm delta       = {np.linalg.norm(b)-np.linalg.norm(a0):+.6f} m/s^2")
    print(f"A closure norm delta = {np.linalg.norm(a1)-np.linalg.norm(a0):+.6f} m/s^2")

print("\n===== MOTION PHASES =====")
for ph in MOVES:
    for imu in (0,1):
        A=packets(imu,"C",ph)
        xyz=A[:,1:4]
        if len(A)<2:
            continue
        d=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
        dur=(A[-1,0]-A[0,0])/1000
        print(
            f"{ph:6s} IMU{imu}: n={len(A):3d} dur={dur:5.2f}s "
            f"dRMS={np.sqrt(np.mean(d*d)):.6f} dP90={np.percentile(d,90):.6f}"
        )
