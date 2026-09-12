#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

def f(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def i(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def pct(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    j=min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))
    return xs[j]

def main():
    ap=argparse.ArgumentParser(description="Проверка согласованности ATTITUDE с оптическим потоком без предположения о неподвижном IMU")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    vals=[]
    for j in range(1,len(rows)):
        r0,r1=rows[j-1],rows[j]
        if i(r1,"valid")!=1 or i(r1,"flow_sent")!=1: continue
        dt=f(r1,"dt_s")
        if not (0<dt<0.2): continue
        roll0,pitch0=f(r0,"fc_roll"),f(r0,"fc_pitch")
        roll1,pitch1=f(r1,"fc_roll"),f(r1,"fc_pitch")
        if not all(map(math.isfinite,[roll0,pitch0,roll1,pitch1])): continue
        droll=math.remainder(roll1-roll0,2*math.pi)/dt
        dpitch=math.remainder(pitch1-pitch0,2*math.pi)/dt
        gyro_x,gyro_y=f(r1,"fc_gyro_x"),f(r1,"fc_gyro_y")
        flow_x,flow_y=f(r1,"flow_body_x"),f(r1,"flow_body_y")
        if not all(map(math.isfinite,[gyro_x,gyro_y,flow_x,flow_y])): continue
        vals.append((math.hypot(droll,dpitch),math.hypot(gyro_x,gyro_y),math.hypot(flow_x,flow_y),
                     droll,dpitch,gyro_x,gyro_y,flow_x,flow_y,j))
    if not vals: raise SystemExit("Нет валидных интервалов")
    print("===== ATTITUDE / GYRO / FLOW — СОГЛАСОВАННОСТЬ =====")
    print(f"интервалов = {len(vals)}")
    for idx,name in [(0,"|dRP/dt| из ATTITUDE"),(1,"|gyroXY|"),(2,"|flow|")]:
        xs=[v[idx] for v in vals]
        print(f"{name}: median/p95/max = {statistics.median(xs):.4f}/{pct(xs,.95):.4f}/{max(xs):.4f} рад/с")
    # Compare ATTITUDE finite-difference to gyro; this should agree regardless of camera motion.
    dif=[math.hypot(v[3]-v[5],v[4]-v[6]) for v in vals]
    print(f"ATTITUDE-rate vs gyro residual: median/p95 = {statistics.median(dif):.4f}/{pct(dif,.95):.4f} рад/с")
    print()
    # Correlations for magnitude, simple Pearson
    def corr(a,b):
        ma=sum(a)/len(a); mb=sum(b)/len(b)
        da=[x-ma for x in a]; db=[x-mb for x in b]
        den=math.sqrt(sum(x*x for x in da)*sum(y*y for y in db))
        return sum(x*y for x,y in zip(da,db))/den if den>1e-12 else float("nan")
    att=[v[0] for v in vals]; gy=[v[1] for v in vals]; fl=[v[2] for v in vals]
    print(f"corr |ATT rate| vs |gyro| = {corr(att,gy):.3f}")
    print(f"corr |gyro| vs |flow|     = {corr(gy,fl):.3f}")
    print()
    fast=[v for v in vals if v[1]>=0.05]
    print(f"интервалов с |gyroXY|>=0.05 = {len(fast)}")
    if fast:
        ratio=[v[2]/v[1] for v in fast if v[1]>1e-6]
        print(f"|flow|/|gyroXY| median/p05/p95 = {statistics.median(ratio):.3f}/{pct(ratio,.05):.3f}/{pct(ratio,.95):.3f}")
    print()
    print("ПРИМЕЧАНИЕ:")
    print("  Этот анализ НЕ предполагает, что IMU оставался в одной точке.")
    print("  Поэтому он годится для проверки timestamp/ATTITUDE, но НЕ доказывает")
    print("  корректность rotation compensation при наклоне на проставках.")
    if statistics.median(dif)<0.08:
        print("ВЫВОД: ATTITUDE и gyro в целом согласованы; грубой временной ошибки FC-телеметрии не видно.")
    else:
        print("ВЫВОД: ATTITUDE finite-difference плохо согласуется с gyro; нужен отдельный time-alignment аудит.")

if __name__=="__main__":
    main()
