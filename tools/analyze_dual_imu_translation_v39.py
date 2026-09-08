#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

SRC=["HIGHRES_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3"]

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k):return float(r[k])
def I(r,k):return int(float(r[k]))
def mean(xs):return statistics.mean(xs) if xs else float("nan")
def norm(v):return math.sqrt(sum(x*x for x in v))
def angle(a,b):
    na,nb=norm(a),norm(b)
    if na<=0 or nb<=0:return float("nan")
    c=sum(x*y for x,y in zip(a,b))/(na*nb)
    return math.degrees(math.acos(max(-1.0,min(1.0,c))))
def central(rr):
    if len(rr)<8:return rr
    k=len(rr)//4
    return rr[k:len(rr)-k]

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_dual_imu_translation_v39.py CSV")

rows=load(Path(sys.argv[1]))
print("================ V39 A-B-A ДВА IMU ================")

ids=sorted(set(I(r,"HIGHRES_IMU_id") for r in rows if I(r,"HIGHRES_IMU_valid")==1))
print("HIGHRES_IMU id:",ids)
print("HIGHRES переключений:",sum(1 for a,b in zip(rows,rows[1:]) if I(a,"HIGHRES_IMU_id")!=I(b,"HIGHRES_IMU_id")))

phase_stats={}
for p,name in ((0,"A_START"),(2,"B"),(4,"A_END")):
    rr=central([r for r in rows if I(r,"phase")==p])
    print(f"\n{name}: n={len(rr)}")
    phase_stats[p]={}
    for s in SRC:
        v=[r for r in rr if I(r,f"{s}_valid")==1]
        if not v:
            print(f"  {s}: НЕТ ДАННЫХ")
            continue
        a=tuple(mean([F(r,f"{s}_{k}_flu") for r in v]) for k in ("ax","ay","az"))
        g=tuple(mean([F(r,f"{s}_{k}_flu") for r in v]) for k in ("gx","gy","gz"))
        phase_stats[p][s]=(a,g)
        print(f"  {s}: ACC=[{a[0]:+.5f},{a[1]:+.5f},{a[2]:+.5f}] |a|={norm(a):.5f} "
              f"GYRO=[{g[0]:+.6f},{g[1]:+.6f},{g[2]:+.6f}] |g|={norm(g):.6f}")

for a,b,label in ((0,2,"A->B"),(2,4,"B->A"),(0,4,"A closure")):
    print(f"\n{label}:")
    for s in SRC[:3]:
        if s not in phase_stats.get(a,{}) or s not in phase_stats.get(b,{}):
            continue
        aa=phase_stats[a][s][0]; bb=phase_stats[b][s][0]
        print(f"  {s}: gravity angle={angle(aa,bb):.4f} deg")

print("\nСРАВНЕНИЕ IMU0/IMU1 ПО ПОЗАМ:")
for p,name in ((0,"A_START"),(2,"B"),(4,"A_END")):
    if "SCALED_IMU" in phase_stats[p] and "SCALED_IMU2" in phase_stats[p]:
        a=phase_stats[p]["SCALED_IMU"][0]; b=phase_stats[p]["SCALED_IMU2"][0]
        print(f"  {name}: угол IMU0↔IMU1={angle(a,b):.4f} deg")

print("\nDECISION:")
print("- Оба IMU показывают близкий A/B сдвиг => эффект common-mode, не специфичен primary IMU0.")
print("- Только IMU0/HIGHRES показывает большой A/B сдвиг, IMU1 стабилен => проблема локализуется в IMU0/его обработке.")
print("- HIGHRES id должен оставаться 0; переключение id во время A-B-A будет отдельным найденным механизмом.")
