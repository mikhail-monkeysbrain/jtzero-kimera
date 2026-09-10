#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Анализ одного live A/B прогона Ground Motion V2/V3.

Не подбирает коэффициенты. Автоматически выделяет интервал, где накопители
реально менялись, и показывает attitude, Luna, итоговые NET/PATH и развитие
расхождения V2/V3 по ходу движения.
"""
import csv
import math
import statistics
import sys

if len(sys.argv) != 2:
    print("Использование: analyze_ground_motion_ab_run.py RUN.csv")
    raise SystemExit(2)

path = sys.argv[1]
with open(path, newline="") as fp:
    rows = list(csv.DictReader(fp))

required = {
    "mono_ns","frame","luna_slant_m","height_v3_m",
    "v2_x_m","v2_y_m","v2_path_m",
    "v3_x_m","v3_y_m","v3_path_m",
    "inliers","scatter_m","roll","pitch","yaw",
}
missing = required.difference(rows[0].keys() if rows else [])
if missing:
    raise SystemExit("Не хватает колонок: " + ", ".join(sorted(missing)))

def F(r, k):
    return float(r[k])

def changed(a, b):
    keys = ("v2_x_m","v2_y_m","v2_path_m","v3_x_m","v3_y_m","v3_path_m")
    return any(abs(F(a,k)-F(b,k)) > 1e-9 for k in keys)

nz = []
for i,r in enumerate(rows):
    q = max(
        math.hypot(F(r,"v2_x_m"),F(r,"v2_y_m")),
        math.hypot(F(r,"v3_x_m"),F(r,"v3_y_m")),
        F(r,"v2_path_m"),F(r,"v3_path_m"),
    )
    if q > 0:
        nz.append(i)
if not nz:
    raise SystemExit("Накопленное движение не найдено")

start = nz[0]
stop = start
for i in range(start + 1, len(rows)):
    if changed(rows[i], rows[i-1]):
        stop = i
m = rows[start:stop+1]

def deg(name):
    return [math.degrees(F(r,name)) for r in m]

def mm(name):
    return [1000*F(r,name) for r in m]

print(path)
print(f"rows total      = {len(rows)}")
print(f"measurement     = {start}..{stop} ({len(m)} rows)")
print(f"frames          = {m[0]['frame']}..{m[-1]['frame']}")
print(f"duration        = {(F(m[-1],'mono_ns')-F(m[0],'mono_ns'))*1e-9:.3f} s")

for name in ("roll","pitch","yaw"):
    v=deg(name)
    print(f"{name:5s}: median={statistics.median(v):+7.3f} deg min={min(v):+7.3f} max={max(v):+7.3f} span={max(v)-min(v):6.3f}")
for name in ("luna_slant_m","height_v3_m"):
    v=mm(name)
    print(f"{name:13s}: median={statistics.median(v):7.2f} mm min={min(v):7.2f} max={max(v):7.2f}")

last=m[-1]
for p in ("v2","v3"):
    net=1000*math.hypot(F(last,p+"_x_m"),F(last,p+"_y_m"))
    pathmm=1000*F(last,p+"_path_m")
    print(f"{p.upper()} final: NET={net:.2f} mm PATH={pathmm:.2f} mm PATH/NET={pathmm/net:.5f}")

print("\nPROGRESS:")
for q in [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]:
    j=min(len(m)-1,round((len(m)-1)*q))
    r=m[j]
    n2=1000*math.hypot(F(r,"v2_x_m"),F(r,"v2_y_m"))
    n3=1000*math.hypot(F(r,"v3_x_m"),F(r,"v3_y_m"))
    print(f"{q:4.0%} V2={n2:7.1f} V3={n3:7.1f} D={n3-n2:+7.1f} mm "
          f"pitch={math.degrees(F(r,'pitch')):+6.2f} yaw={math.degrees(F(r,'yaw')):+7.2f} "
          f"inl={int(F(r,'inliers')):3d} scatter={1000*F(r,'scatter_m'):5.2f} mm")
