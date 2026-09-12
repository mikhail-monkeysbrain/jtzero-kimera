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

def rms(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else float("nan")

def med(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return statistics.median(xs) if xs else float("nan")

def pctl(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    return xs[min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))]

def main():
    ap=argparse.ArgumentParser(description="Проверка знаков/осей компенсации вращения OpticalFlow")
    ap.add_argument("csv",type=Path)
    ap.add_argument("--gyro-threshold",type=float,default=0.05,
                    help="минимальный |gyro XY| для вращательного участка, рад/с")
    ap.add_argument("--max-vh",type=float,default=0.08,
                    help="необязательный потолок EKF |vH| для отбора, м/с")
    args=ap.parse_args()

    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows: raise SystemExit("Пустой CSV")
    req={"flow_send_x","flow_send_y","fc_gyro_x","fc_gyro_y","fc_gyro_z",
         "fc_roll","fc_pitch","fc_yaw","luna_m","dt_s","flow_sent","valid"}
    miss=req-set(rows[0])
    if miss: raise SystemExit("Не хватает полей: "+", ".join(sorted(miss)))

    valid=[]
    rot=[]
    for r in rows:
        if i(r,"valid")!=1 or i(r,"flow_sent")!=1: continue
        dt=f(r,"dt_s")
        vals=[f(r,"flow_send_x"),f(r,"flow_send_y"),f(r,"fc_gyro_x"),f(r,"fc_gyro_y")]
        if not (0<dt<0.2) or not all(map(math.isfinite,vals)): continue
        valid.append(r)
        gxy=math.hypot(f(r,"fc_gyro_x"),f(r,"fc_gyro_y"))
        if gxy>=args.gyro_threshold:
            rot.append(r)

    print("===== ПРОВЕРКА ВРАЩАТЕЛЬНОЙ КОМПЕНСАЦИИ OPTICAL FLOW =====")
    print(f"всего валидных кадров = {len(valid)}")
    print(f"кадров с |gyro XY| >= {args.gyro_threshold:.3f} рад/с = {len(rot)}")
    if len(rot)<20:
        print("РЕЗУЛЬТАТ: вращательных кадров недостаточно. Нужен отдельный rotation-only тест.")
        return

    fx=[f(r,"flow_send_x") for r in rot]
    fy=[f(r,"flow_send_y") for r in rot]
    gx=[f(r,"fc_gyro_x") for r in rot]
    gy=[f(r,"fc_gyro_y") for r in rot]
    gz=[f(r,"fc_gyro_z") for r in rot]
    rng=[f(r,"luna_m") for r in rot]

    # Expected for the current ArduPilot convention used by our publisher:
    # raw body flow should approximately equal body gyro XY for pure rotation,
    # so compensated residual is (-flow + gyro).
    cur=[math.hypot(-a+c,-b+d) for a,b,c,d in zip(fx,fy,gx,gy)]

    candidates={
      "ТЕКУЩЕЕ: flow=(+gx,+gy)": [(a-c,b-d) for a,b,c,d in zip(fx,fy,gx,gy)],
      "ОБА ЗНАКА: flow=(-gx,-gy)": [(a+c,b+d) for a,b,c,d in zip(fx,fy,gx,gy)],
      "SWAP: flow=(+gy,+gx)": [(a-d,b-c) for a,b,c,d in zip(fx,fy,gx,gy)],
      "SWAP: flow=(-gy,-gx)": [(a+d,b+c) for a,b,c,d in zip(fx,fy,gx,gy)],
      "SWAP Y-: flow=(+gy,-gx)": [(a-d,b+c) for a,b,c,d in zip(fx,fy,gx,gy)],
      "SWAP X-: flow=(-gy,+gx)": [(a+d,b-c) for a,b,c,d in zip(fx,fy,gx,gy)],
    }

    scores=[]
    for name,res in candidates.items():
        rr=[math.hypot(x,y) for x,y in res]
        scores.append((rms(rr),med(rr),pctl(rr,.95),name))
    scores.sort()

    print(f"|gyro XY| median/p95 = {med([math.hypot(a,b) for a,b in zip(gx,gy)]):.4f}/{pctl([math.hypot(a,b) for a,b in zip(gx,gy)],.95):.4f} рад/с")
    print(f"|gyro Z|  median/p95 = {med([abs(x) for x in gz]):.4f}/{pctl([abs(x) for x in gz],.95):.4f} рад/с")
    print(f"|flow|    median/p95 = {med([math.hypot(a,b) for a,b in zip(fx,fy)]):.4f}/{pctl([math.hypot(a,b) for a,b in zip(fx,fy)],.95):.4f} рад/с")
    print(f"TF-Luna median/min/max = {med(rng):.3f}/{min(rng):.3f}/{max(rng):.3f} м")
    print()
    print("ОСТАТОК ПО ВАРИАНТАМ ОСЕЙ/ЗНАКОВ (меньше = лучше):")
    for rr,mm,p95,name in scores:
        print(f"  {name:32s} RMS={rr:.4f}  med={mm:.4f}  p95={p95:.4f} рад/с")

    best=scores[0]
    current=next(x for x in scores if x[3].startswith("ТЕКУЩЕЕ"))
    print()
    print(f"лучший вариант: {best[3]}")
    print(f"текущий/лучший RMS = {current[0]/best[0]:.2f}x" if best[0]>1e-9 else "текущий/лучший RMS = inf")

    # Estimate false horizontal speed caused by the current compensation residual.
    h=[x+0.021 for x in rng]
    false_v=[rr*hh for rr,hh in zip(cur,h)]
    print(f"текущая residual-speed median/p95 = {med(false_v):.3f}/{pctl(false_v,.95):.3f} м/с")

    if best[3].startswith("ТЕКУЩЕЕ") and current[0] < 0.15:
        print("ВЫВОД: грубой ошибки знаков/перестановки XY не видно. Ищем timing, lever-arm, perspective/height.")
    elif current[0] > best[0]*1.5:
        print("ВЫВОД: ТЕКУЩЕЕ соглашение осей/знаков существенно хуже альтернативы. Это сильный кандидат на причину ложного движения при вращении.")
    else:
        print("ВЫВОД: явной одной ошибки знаков не доказано; нужен чистый rotation-only прогон без XY-переноса.")

if __name__=="__main__":
    main()
