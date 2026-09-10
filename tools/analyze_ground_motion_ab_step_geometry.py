#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Покадровый разбор Ground Motion V2/V3 A/B по уже записанному CSV.

Не подбирает scale и не делает вывод из PATH как из истинной дистанции.
Использует только накопленные V2/V3 координаты и attitude, чтобы проверить:
- где появляются различия V2/V3;
- связаны ли они с изменениями roll/pitch/yaw;
- насколько сильно направления покадровых шагов V3 отличаются от V2;
- есть ли редкие крупные шаги, которые раздувают PATH.
"""
import csv, math, statistics, sys


def angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2*math.pi
    while d < -math.pi:
        d += 2*math.pi
    return d


def med(v):
    return statistics.median(v) if v else float('nan')


def pct(v, p):
    if not v:
        return float('nan')
    q = sorted(v)
    x = (len(q)-1)*p
    i = int(math.floor(x)); j = min(len(q)-1, i+1)
    a = x-i
    return q[i]*(1-a)+q[j]*a


def pearson(a,b):
    if len(a) != len(b) or len(a) < 3:
        return float('nan')
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    if va<=0 or vb<=0: return float('nan')
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)


def main(path):
    with open(path, newline='') as f:
        r=list(csv.DictReader(f))
    need=['mono_ns','v2_x_m','v2_y_m','v2_path_m','v3_x_m','v3_y_m','v3_path_m','roll','pitch','yaw','luna_slant_m','height_v3_m','inliers','scatter_m']
    miss=[k for k in need if k not in (r[0].keys() if r else [])]
    if miss: raise SystemExit('Нет колонок: '+', '.join(miss))
    F=lambda row,k: float(row[k])

    steps=[]
    for i in range(1,len(r)):
        a,b=r[i-1],r[i]
        dx2=F(b,'v2_x_m')-F(a,'v2_x_m'); dy2=F(b,'v2_y_m')-F(a,'v2_y_m')
        dx3=F(b,'v3_x_m')-F(a,'v3_x_m'); dy3=F(b,'v3_y_m')-F(a,'v3_y_m')
        dp2=F(b,'v2_path_m')-F(a,'v2_path_m'); dp3=F(b,'v3_path_m')-F(a,'v3_path_m')
        m2=math.hypot(dx2,dy2); m3=math.hypot(dx3,dy3)
        if max(m2,m3,abs(dp2),abs(dp3)) <= 1e-9:
            continue
        dt=(F(b,'mono_ns')-F(a,'mono_ns'))*1e-9
        droll=angdiff(F(b,'roll'),F(a,'roll'))
        dpitch=angdiff(F(b,'pitch'),F(a,'pitch'))
        dyaw=angdiff(F(b,'yaw'),F(a,'yaw'))
        theta2=math.atan2(dy2,dx2) if m2>1e-9 else float('nan')
        theta3=math.atan2(dy3,dx3) if m3>1e-9 else float('nan')
        dtheta=angdiff(theta3,theta2) if m2>1e-9 and m3>1e-9 else float('nan')
        steps.append(dict(i=i,dt=dt,m2=m2,m3=m3,dp2=dp2,dp3=dp3,
                          dx2=dx2,dy2=dy2,dx3=dx3,dy3=dy3,
                          droll=droll,dpitch=dpitch,dyaw=dyaw,dtheta=dtheta,
                          pitch=F(b,'pitch'),yaw=F(b,'yaw'),
                          luna=F(b,'luna_slant_m'),h=F(b,'height_v3_m'),
                          inliers=int(F(b,'inliers')),scatter=F(b,'scatter_m')))
    both=[s for s in steps if s['m2']>1e-9 and s['m3']>1e-9]
    print(path)
    print(f'accepted update rows = {len(steps)}, both V2/V3 = {len(both)}')
    if not both: return

    m2=[s['m2']*1000 for s in both]; m3=[s['m3']*1000 for s in both]
    ratios=[s['m3']/s['m2'] for s in both if s['m2']>1e-7]
    dth=[math.degrees(s['dtheta']) for s in both if math.isfinite(s['dtheta'])]
    dyaw=[math.degrees(s['dyaw']) for s in both]
    dpitch=[math.degrees(s['dpitch']) for s in both]
    droll=[math.degrees(s['droll']) for s in both]
    print(f'V2 step mm: median={med(m2):.3f} p90={pct(m2,.90):.3f} p99={pct(m2,.99):.3f} max={max(m2):.3f}')
    print(f'V3 step mm: median={med(m3):.3f} p90={pct(m3,.90):.3f} p99={pct(m3,.99):.3f} max={max(m3):.3f}')
    print(f'V3/V2 step magnitude: median={med(ratios):.5f} p10={pct(ratios,.10):.5f} p90={pct(ratios,.90):.5f}')
    print(f'angle(V3)-angle(V2): median={med(dth):+.3f} deg p10={pct(dth,.10):+.3f} p90={pct(dth,.90):+.3f} maxabs={max(abs(x) for x in dth):.3f}')
    print(f'per-step dYaw:   median={med(dyaw):+.4f} deg p90abs={pct([abs(x) for x in dyaw],.90):.4f} maxabs={max(abs(x) for x in dyaw):.4f}')
    print(f'per-step dPitch: median={med(dpitch):+.4f} deg p90abs={pct([abs(x) for x in dpitch],.90):.4f} maxabs={max(abs(x) for x in dpitch):.4f}')
    print(f'per-step dRoll:  median={med(droll):+.4f} deg p90abs={pct([abs(x) for x in droll],.90):.4f} maxabs={max(abs(x) for x in droll):.4f}')

    absdth=[abs(math.degrees(s['dtheta'])) for s in both]
    absdyaw=[abs(math.degrees(s['dyaw'])) for s in both]
    absdpitch=[abs(math.degrees(s['dpitch'])) for s in both]
    abshchange=[abs(s['h'] - (r[s['i']-1] and F(r[s['i']-1],'height_v3_m')))*1000 for s in both]
    print('\nCORRELATION (диагностика, не причинность):')
    print(f'|angle V3-V2| vs |dYaw|   r={pearson(absdth,absdyaw):+.3f}')
    print(f'|angle V3-V2| vs |dPitch| r={pearson(absdth,absdpitch):+.3f}')
    print(f'V3 step size vs |dYaw|    r={pearson(m3,absdyaw):+.3f}')
    print(f'V3 step size vs |dPitch|  r={pearson(m3,absdpitch):+.3f}')
    print(f'V3 step size vs |dH|      r={pearson(m3,abshchange):+.3f}')

    # Вклад крупнейших V3 шагов в PATH: показывает, раздувают ли PATH редкие выбросы.
    total=sum(m3)
    order=sorted(both,key=lambda s:s['m3'],reverse=True)
    print('\nTOP V3 STEPS:')
    for s in order[:12]:
        print(f"row={s['i']:4d} m2={s['m2']*1000:7.2f} m3={s['m3']*1000:7.2f} "
              f"ratio={(s['m3']/s['m2'] if s['m2']>1e-9 else float('nan')):6.3f} "
              f"dAng={math.degrees(s['dtheta']):+7.2f}deg "
              f"dYaw={math.degrees(s['dyaw']):+7.3f} dPitch={math.degrees(s['dpitch']):+7.3f} "
              f"H={s['h']*1000:6.1f} inl={s['inliers']:3d} scatter={s['scatter']*1000:5.2f}mm")
    top10=sum(s['m3']*1000 for s in order[:10])
    print(f'\nTop-10 V3 steps contribute {top10:.2f} / {total:.2f} mm = {100*top10/total:.1f}% of V3 step-length sum')

    # Общая направленность шагов относительно итогового вектора V3.
    sx=sum(s['dx3'] for s in both); sy=sum(s['dy3'] for s in both)
    phi=math.atan2(sy,sx)
    along=[]; cross=[]; backwards=0.0
    for s in both:
        al=s['dx3']*math.cos(phi)+s['dy3']*math.sin(phi)
        cr=-s['dx3']*math.sin(phi)+s['dy3']*math.cos(phi)
        along.append(al*1000); cross.append(cr*1000)
        if al<0: backwards += -al*1000
    print('\nV3 RELATIVE TO FINAL DIRECTION:')
    print(f'forward signed sum = {sum(along):.2f} mm')
    print(f'absolute cross sum = {sum(abs(x) for x in cross):.2f} mm')
    print(f'backward component sum = {backwards:.2f} mm')
    print(f'cross step median abs = {med([abs(x) for x in cross]):.3f} mm, p90={pct([abs(x) for x in cross],.90):.3f} mm')

if __name__=='__main__':
    if len(sys.argv)!=2:
        print('Использование: analyze_ground_motion_ab_step_geometry.py RUN.csv')
        raise SystemExit(2)
    main(sys.argv[1])
