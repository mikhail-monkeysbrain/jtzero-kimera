#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(x):return statistics.mean(x) if x else float("nan")
def norm(v):return math.sqrt(sum(q*q for q in v))
def central(rows):
    if len(rows)<8:return rows
    k=len(rows)//4
    return rows[k:len(rows)-k]

# Values read from FC v40 dump. These are sensor-frame ArduPilot calibration offsets.
OFFSETS={
    "SCALED_IMU":  (-0.1494493932, +0.8052263856, +0.03733253106),
    "SCALED_IMU2": (-0.1770949960, +0.7808007002, +0.08514022827),
}

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v39_offset_hypothesis.py CSV")

rows=load(Path(sys.argv[1]))
print("================ V39 OFFSET HYPOTHESIS ================")
print("Tests whether the observed A/B |a| change has the magnitude expected from a constant residual Y-axis offset.")
print("IMPORTANT: FC calibration offsets are applied in sensor frame before board rotation.")
print("Without proving the internal sensor-orientation sign, this analyzer tests both +/- Y signs and is diagnostic only.")

states=[(0,"A_START"),(2,"B"),(4,"A_END")]

for s in ("SCALED_IMU","SCALED_IMU2"):
    vals={}
    print(f"\n{s}: configured accel offset={OFFSETS[s]}")
    for p,name in states:
        rr=central([r for r in rows if I(r,"phase")==p and I(r,f"{s}_valid")==1])
        a=(mean([F(r,f"{s}_ax_flu") for r in rr]),
           mean([F(r,f"{s}_ay_flu") for r in rr]),
           mean([F(r,f"{s}_az_flu") for r in rr]))
        vals[p]=a
        print(f"  {name}: a={a} |a|={norm(a):.6f}")

    A,B=vals[0],vals[2]
    # Effective residual body-Y correction needed so stationary A and B have equal norm,
    # if X/Z are left unchanged.
    den=2.0*(A[1]-B[1])
    req=(sum(q*q for q in A)-sum(q*q for q in B))/den if abs(den)>1e-9 else float("nan")
    print(f"  effective FLU-Y correction needed for |A|=|B|: {req:+.6f} m/s^2")
    print(f"  configured |Y offset|: {abs(OFFSETS[s][1]):.6f} m/s^2")
    print(f"  magnitude ratio required/configured: {abs(req)/abs(OFFSETS[s][1]):.3f}")

    # Test adding/subtracting configured Y magnitude directly in current FLU coordinates.
    for sign,label in ((+1,"ADD +|offsetY|"),(-1,"ADD -|offsetY|")):
        oy=sign*abs(OFFSETS[s][1])
        nn={}
        for p,name in states:
            a=vals[p]
            aa=(a[0],a[1]+oy,a[2])
            nn[p]=norm(aa)
        print(f"  {label}: A={nn[0]:.6f} B={nn[2]:.6f} A_END={nn[4]:.6f} "
              f"A-B={nn[0]-nn[2]:+.6f} closure={nn[4]-nn[0]:+.6f}")

print("\nINTERPRETATION:")
print("- Required Y correction close in magnitude to configured Y offset is a strong clue, not proof.")
print("- Because offsets are stored/applied in sensor frame, the sign in current FLU frame must be verified from the IMU sensor-orientation mapping before changing FC parameters.")
print("- Do NOT edit INS_ACCOFFS/INS_ACC2OFFS from this result alone.")
