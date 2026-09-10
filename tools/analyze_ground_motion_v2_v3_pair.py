#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Парная проверка математики Ground Motion V2/V3 без нового физического прогона.

Скрипт не пытается восстановить optical flow из CSV: в текущем V3 CSV нет feature
correspondences. Вместо этого он проверяет ту часть модели, которую можно проверить
строго по уже записанным данным: высоту, attitude и масштабную чувствительность.
Никаких коэффициентов коррекции не подбирает.
"""
import csv, math, sys


def load(path):
    with open(path, newline='') as f:
        rows=list(csv.DictReader(f))
    rows=[r for r in rows if r.get('height_m') and float(r['height_m'])>0]
    if not rows: raise SystemExit(f'Нет валидных строк: {path}')
    return rows

def med(v):
    v=sorted(v); n=len(v)
    return v[n//2] if n%2 else .5*(v[n//2-1]+v[n//2])

def report(path):
    r=load(path)
    sl=[float(x['luna_slant_m']) for x in r]
    h=[float(x['height_m']) for x in r]
    roll=[math.degrees(float(x['roll'])) for x in r]
    pitch=[math.degrees(float(x['pitch'])) for x in r]
    x=[float(q['x_m']) for q in r]; y=[float(q['y_m']) for q in r]
    pathm=[float(q['path_m']) for q in r]
    # Берём максимум накопленного пути как диагностический итог: после STOP CSV может
    # содержать DONE-кадры, но накопитель уже не меняется.
    net=max(math.hypot(a,b) for a,b in zip(x,y)); p=max(pathm)
    ratio=[hh/ss for hh,ss in zip(h,sl) if ss>0]
    print('\n'+path)
    print(f'  rows={len(r)}')
    print(f'  Luna slant median = {med(sl)*1000:.2f} mm')
    print(f'  V3 vertical H med = {med(h)*1000:.2f} mm')
    print(f'  H/slant median    = {med(ratio):.6f}')
    print(f'  roll median       = {med(roll):+.3f} deg')
    print(f'  pitch median      = {med(pitch):+.3f} deg')
    expected=math.cos(math.radians(med(roll)))*math.cos(math.radians(med(pitch)))
    print(f'  cos(r)*cos(p)     = {expected:.6f}')
    print(f'  max NET           = {net*1000:.2f} mm')
    print(f'  max PATH          = {p*1000:.2f} mm')
    print(f'  NET/500           = {net/0.5:.6f}')
    print('  NOTE: этот CSV не содержит feature pairs, поэтому честный V2 replay из него невозможен.')

if len(sys.argv)<2:
    print('Использование: analyze_ground_motion_v2_v3_pair.py RUN1.csv [RUN2.csv ...]')
    raise SystemExit(2)
for p in sys.argv[1:]: report(p)
