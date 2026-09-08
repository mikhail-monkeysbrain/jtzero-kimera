#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path
import numpy as np

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def norm(v):return float(np.linalg.norm(v))
def angle(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    c=float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0,min(1.0,c))))
def central(rr):
    if len(rr)<8:return rr
    k=len(rr)//4
    return rr[k:len(rr)-k]
def rodrigues(rv):
    th=np.linalg.norm(rv)
    if th<1e-12:return np.eye(3)
    k=rv/th
    K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]],float)
    return np.eye(3)+math.sin(th)*K+(1-math.cos(th))*(K@K)
def integrate(rows,prefix,phase,bias):
    rr=[r for r in rows if I(r,"phase")==phase and I(r,f"{prefix}_valid")==1]
    # dedup by source timestamp
    seen=set(); u=[]
    for r in rr:
        t=I(r,f"{prefix}_t_us")
        if t in seen:continue
        seen.add(t);u.append(r)
    u.sort(key=lambda r:I(r,f"{prefix}_t_us"))
    R=np.eye(3)
    for a,b in zip(u,u[1:]):
        ta=I(a,f"{prefix}_t_us"); tb=I(b,f"{prefix}_t_us")
        dt=(tb-ta)*1e-6
        if not (0<dt<=0.05):continue
        wa=np.array([F(a,f"{prefix}_{q}_flu") for q in ("gx","gy","gz")])-bias
        wb=np.array([F(b,f"{prefix}_{q}_flu") for q in ("gx","gy","gz")])-bias
        w=0.5*(wa+wb)
        R=R@rodrigues(w*dt)
    return R

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v39_gyro_gravity_consistency.py CSV")
rows=load(Path(sys.argv[1]))
print("================ V39 GYRO -> GRAVITY CONSISTENCY ================")
for prefix,label in (("SCALED_IMU","IMU0"),("SCALED_IMU2","IMU1")):
    s0=central([r for r in rows if I(r,"phase")==0 and I(r,f"{prefix}_valid")==1])
    sb=central([r for r in rows if I(r,"phase")==2 and I(r,f"{prefix}_valid")==1])
    if not s0 or not sb:continue
    a0=np.array([mean([F(r,f"{prefix}_{q}_flu") for r in s0]) for q in ("ax","ay","az")])
    aB=np.array([mean([F(r,f"{prefix}_{q}_flu") for r in sb]) for q in ("ax","ay","az")])
    bias=np.array([mean([F(r,f"{prefix}_{q}_flu") for r in s0]) for q in ("gx","gy","gz")])
    R=integrate(rows,prefix,1,bias)
    # R maps initial body to inertial orientation delta; gravity in new body is R^T g0
    pred=R.T@a0
    # rotation-vector magnitude from matrix
    theta=math.acos(max(-1,min(1,(np.trace(R)-1)/2)))
    if theta<1e-9:
        rv=np.zeros(3)
    else:
        rv=theta/(2*math.sin(theta))*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    print(f"{label}:")
    print(f"  gyro rotvec = [{rv[0]*180/math.pi:+.4f},{rv[1]*180/math.pi:+.4f},{rv[2]*180/math.pi:+.4f}] deg")
    print(f"  |gyro rotation| = {np.linalg.norm(rv)*180/math.pi:.4f} deg")
    print(f"  actual gravity A->B angle = {angle(a0,aB):.4f} deg")
    print(f"  gyro-predicted gravity change = {angle(a0,pred):.4f} deg")
    print(f"  predicted-vs-measured B gravity residual = {angle(pred,aB):.4f} deg")
    print(f"  A gravity = [{a0[0]:+.5f},{a0[1]:+.5f},{a0[2]:+.5f}]")
    print(f"  B measured= [{aB[0]:+.5f},{aB[1]:+.5f},{aB[2]:+.5f}]")
    print(f"  B predicted= [{pred[0]:+.5f},{pred[1]:+.5f},{pred[2]:+.5f}]")
print("\nDECISION:")
print("- residual << 1 deg means independent gyro integration predicts the endpoint accelerometer gravity direction.")
print("- If this holds on both physical IMUs, a real rotation of the shared FC/IMU rigid frame relative to gravity is strongly supported.")
print("- Fixed sensor/board transforms cannot by themselves create a reversible time-varying gyro rotation that also predicts the endpoint gravity vector.")
