#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, math, statistics
from pathlib import Path

def f(row,k,default=float("nan")):
    try: return float(row.get(k,""))
    except: return default

def med(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.median(v) if v else float("nan")

def hypot2(a,b): return math.hypot(a,b)

def load(path):
    with open(path,newline="") as fh:
        return list(csv.DictReader(fh))

def quiet_score(r):
    vx,vy=f(r,"ekf_vx_ned",0),f(r,"ekf_vy_ned",0)
    flowx,flowy=f(r,"flow_body_x",0),f(r,"flow_body_y",0)
    return hypot2(vx,vy)+0.12*hypot2(flowx,flowy)

def window_stats(rows, lo, hi):
    seg=rows[lo:hi]
    return {
        "n":len(seg),
        "x":med([f(r,"ekf_x_ned") for r in seg if f(r,"ekf_local_valid",0)>0.5]),
        "y":med([f(r,"ekf_y_ned") for r in seg if f(r,"ekf_local_valid",0)>0.5]),
        "z":med([f(r,"ekf_z_ned") for r in seg if f(r,"ekf_local_valid",0)>0.5]),
        "range":med([f(r,"luna_m") for r in seg]),
        "vh":med([hypot2(f(r,"ekf_vx_ned",0),f(r,"ekf_vy_ned",0)) for r in seg]),
    }

def integrate_raw(rows, lo, hi, include_stale=False, camera_minus_range_z=-0.021):
    n=e=0.0
    used=0
    stale=invalid=0
    for r in rows[lo:hi]:
        dt=f(r,"dt_s",0)
        valid=f(r,"valid",0)>0.5
        sent=f(r,"flow_sent",0)>0.5
        if not valid:
            invalid+=1
            continue
        if not sent:
            stale+=1
            if not include_stale:
                continue
        if not (0<dt<0.2):
            continue
        # TF-Luna measures distance from the rangefinder. Camera optical centre is
        # ~21 mm higher in the current mount (camera_z=0.050, range_z=0.071),
        # so camera-to-ground height is range - (camera_z-range_z) = range + 0.021 m.
        h=f(r,"luna_m",float("nan"))
        if math.isfinite(h):
            h = h - camera_minus_range_z
        if not math.isfinite(h) or h<=0.02: continue
        fx=f(r,"flow_body_x",float("nan")); fy=f(r,"flow_body_y",float("nan"))
        gx=f(r,"fc_gyro_x",float("nan")); gy=f(r,"fc_gyro_y",float("nan"))
        roll=f(r,"fc_roll",float("nan")); pitch=f(r,"fc_pitch",float("nan")); yaw=f(r,"fc_yaw",float("nan"))
        if not all(map(math.isfinite,[fx,fy,gx,gy,roll,pitch,yaw])): continue
        comp_x=-fx+gx
        comp_y=-fy+gy
        dbx=(-comp_y)*h*dt
        dby=( comp_x)*h*dt
        cr,sr=math.cos(roll),math.sin(roll)
        cp,sp=math.cos(pitch),math.sin(pitch)
        cy,sy=math.cos(yaw),math.sin(yaw)
        r00=cy*cp
        r01=cy*sp*sr-sy*cr
        r10=sy*cp
        r11=sy*sp*sr+cy*cr
        n += r00*dbx + r01*dby
        e += r10*dbx + r11*dby
        used+=1
    return n,e,used,invalid,stale

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv")
    args=ap.parse_args()
    rows=load(args.csv)
    if len(rows)<80:
        raise SystemExit("Слишком мало строк")

    # Find main movement interval from combined EKF speed + range change.
    vh=[hypot2(f(r,"ekf_vx_ned",0),f(r,"ekf_vy_ned",0)) for r in rows]
    flow=[hypot2(f(r,"flow_body_x",0),f(r,"flow_body_y",0)) for r in rows]
    rng=[f(r,"luna_m",float("nan")) for r in rows]
    score=[]
    for i,r in enumerate(rows):
        dr=0
        if i>0 and math.isfinite(rng[i]) and math.isfinite(rng[i-1]):
            dr=abs(rng[i]-rng[i-1])*3.0
        score.append(vh[i]+0.10*flow[i]+dr)

    # threshold deliberately low; then merge into one dominant cluster.
    active=[i for i,x in enumerate(score) if x>0.025]
    if not active:
        raise SystemExit("Автоматически не найден участок движения")
    # largest dense cluster allowing gaps up to 25 rows
    clusters=[]
    cur=[active[0]]
    for x in active[1:]:
        if x-cur[-1] <= 25: cur.append(x)
        else: clusters.append(cur); cur=[x]
    clusters.append(cur)
    cluster=max(clusters,key=lambda c:(c[-1]-c[0],len(c)))
    m0,m1=cluster[0],cluster[-1]

    # Quiet windows before and after movement. About 2-3 s at typical current cadence.
    W=35
    pre_hi=max(W,m0-8)
    pre_lo=max(0,pre_hi-W)
    post_lo=min(len(rows)-W,m1+8)
    post_hi=min(len(rows),post_lo+W)

    A=window_stats(rows,pre_lo,pre_hi)
    B=window_stats(rows,post_lo,post_hi)
    dn=B["x"]-A["x"]; de=B["y"]-A["y"]
    ekf_dist=hypot2(dn,de)
    rn,re,used,invalid,stale=integrate_raw(rows,pre_hi,post_lo,include_stale=False)
    raw_dist=hypot2(rn,re)
    ran,rae,used_all,invalid_all,stale_all=integrate_raw(rows,pre_hi,post_lo,include_stale=True)
    raw_all_dist=hypot2(ran,rae)

    t0=f(rows[pre_hi],"mono_ns",0)/1e9
    t1=f(rows[post_lo],"mono_ns",0)/1e9
    max_v=max(vh[pre_hi:post_lo] or [0])
    max_flow=max(flow[pre_hi:post_lo] or [0])
    ranges=[x for x in rng[pre_hi:post_lo] if math.isfinite(x)]

    print("===== JT-ZERO — ОДИН РУЧНОЙ ПЕРЕНОС: КАМЕРА → FC =====")
    print("CSV:", args.csv)
    print()
    print("Автоматически выбран основной участок движения:")
    print(f"  строки {pre_hi}..{post_lo}, длительность ~{max(0,t1-t0):.2f} с")
    print("  Перед началом и после конца используются спокойные окна по ~35 кадров.")
    print()
    print("ТОЧКА A — до подъёма/переноса")
    print(f"  FC N/E = {A['x']:+.4f} / {A['y']:+.4f} м")
    print(f"  TF-Luna median = {A['range']:.3f} м")
    print()
    print("ТОЧКА B — после постановки и остановки")
    print(f"  FC N/E = {B['x']:+.4f} / {B['y']:+.4f} м")
    print(f"  TF-Luna median = {B['range']:.3f} м")
    print()
    print("FC / EKF — что в итоге решил полётный контроллер")
    print(f"  ΔN/E = {dn*1000:+.1f} / {de*1000:+.1f} мм")
    print(f"  горизонтальное перемещение = {ekf_dist*1000:.1f} мм")
    print()
    print("RAW CAMERA — независимая интеграция Optical Flow")
    print("  Учитывается смещение по высоте между камерой и TF-Luna: camera-range = -21 мм.")
    print("  SENT-ONLY (только реально отправленные FC измерения):")
    print(f"    ΔN/E = {rn*1000:+.1f} / {re*1000:+.1f} мм")
    print(f"    горизонтальное перемещение = {raw_dist*1000:.1f} мм")
    print(f"    использовано кадров = {used}; invalid = {invalid}; valid-but-not-sent = {stale}")
    print("  ALL-VALID (включая valid кадры, отброшенные stale-gate):")
    print(f"    ΔN/E = {ran*1000:+.1f} / {rae*1000:+.1f} мм")
    print(f"    горизонтальное перемещение = {raw_all_dist*1000:.1f} мм")
    print(f"    использовано кадров = {used_all}")
    print(f"  Потеря из-за stale-gate по вектору = {(raw_all_dist-raw_dist)*1000:+.1f} мм по модулю (ориентир, не скалярная сумма пути)")
    print()
    print("ДИНАМИКА")
    print(f"  max |Vxy| FC = {max_v:.3f} м/с")
    print(f"  max |flow| = {max_flow:.3f} рад/с")
    if ranges:
        print(f"  TF-Luna min/max = {min(ranges):.3f} / {max(ranges):.3f} м")
    print()
    print("СРАВНЕНИЕ С ФИЗИЧЕСКИМ ПЕРЕНОСОМ")
    print("  Физически оператор сообщил примерно 400–500 мм.")
    ref=raw_all_dist
    if 0.4 <= ref <= 0.55:
        print("  ALL-VALID RAW близка к физическому переносу → значимая часть ошибки может быть между frontend/stale policy и EKF.")
    elif ref < 0.30:
        print("  Даже ALL-VALID RAW существенно меньше 400–500 мм → недоизмерение действительно возникает до EKF.")
    else:
        print("  ALL-VALID RAW находится между ожидаемым и явно неверным диапазоном → одного этого прогона недостаточно для окончательного вывода о масштабе.")
    if raw_dist>0.05:
        print(f"  EKF / SENT-RAW по модулю = {ekf_dist/raw_dist:.3f}")
    if raw_all_dist>0.05:
        print(f"  EKF / ALL-VALID-RAW по модулю = {ekf_dist/raw_all_dist:.3f}")
    print()
    print("ПРИМЕЧАНИЕ: границы движения выбраны автоматически. Если они не совпали с фактическим переносом,")
    print("этот вывод не считать доказательством — скрипт печатает выбранный интервал именно для проверки.")

if __name__=="__main__":
    main()
