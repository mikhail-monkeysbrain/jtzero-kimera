#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(x):return statistics.mean(x) if x else float("nan")
def norm(v):return math.sqrt(sum(x*x for x in v))
def central(rows):
    if len(rows)<8:return rows
    k=len(rows)//4
    return rows[k:len(rows)-k]

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v39_accel_norm_by_position.py /home/vio/jtzero_dual_imu_translation_v39.csv")

rows=load(Path(sys.argv[1]))
print("================ V39 ACCEL NORM BY POSITION ================")
print("Compares stationary acceleration magnitude at A-start, B, and A-end for HIGHRES/IMU0/IMU1.")
print("A horizontal 500-mm translation cannot physically change local gravity magnitude by ~0.03 m/s^2.")
print("Therefore a repeatable A/B |a| shift indicates orientation-/processing-/calibration-dependent accelerometer magnitude error.")

sources=["HIGHRES_IMU","SCALED_IMU","SCALED_IMU2"]
states=[(0,"A_START"),(2,"B"),(4,"A_END")]
vals={}

for p,name in states:
    rr=central([r for r in rows if I(r,"phase")==p])
    print(f"\n{name}:")
    vals[p]={}
    for s in sources:
        vv=[r for r in rr if I(r,f"{s}_valid")==1]
        if not vv:
            print(f"  {s}: no data"); continue
        a=(mean([F(r,f"{s}_ax_flu") for r in vv]),
           mean([F(r,f"{s}_ay_flu") for r in vv]),
           mean([F(r,f"{s}_az_flu") for r in vv]))
        n=norm(a)
        vals[p][s]=(a,n)
        print(f"  {s}: ACC=[{a[0]:+.5f},{a[1]:+.5f},{a[2]:+.5f}] |a|={n:.6f} m/s^2")

print("\nCHANGES:")
for s in sources:
    if all(s in vals[p] for p in (0,2,4)):
        n0=vals[0][s][1]; nb=vals[2][s][1]; n1=vals[4][s][1]
        print(f"  {s}: A_START->B={nb-n0:+.6f} m/s^2  B->A_END={n1-nb:+.6f}  A closure={n1-n0:+.6f}")

print("\nINTERPRETATION:")
print("- Similar negative A->B and positive B->A |a| change on IMU0 and IMU1 means the effect is common-mode and position/orientation-linked.")
print("- Near-zero A closure means it is reversible, not a monotonic thermal drift.")
print("- Because stationary gravity magnitude should not change over 0.5 m horizontal travel, this points to accelerometer calibration/model/processing dependence on orientation, not real gravity change.")
print("- This can directly leave a world-Z residual even when attitude is otherwise correct, because rotation preserves vector magnitude.")
