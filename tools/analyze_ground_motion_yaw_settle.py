#!/usr/bin/env python3
"""Проверка стабильности yaw FC между статикой A/B в Ground Motion CSV.

Скрипт использует только стандартную библиотеку Python и ничего не меняет.
Для каждого прогона считает средний yaw в первых и последних N секундах,
его разброс, а также yaw в окне движения.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import fmean, pstdev

RAD2DEG = 180.0 / math.pi


def unwrap(values):
    if not values:
        return []
    out = [values[0]]
    for a in values[1:]:
        p = out[-1]
        while a - p > math.pi:
            a -= 2.0 * math.pi
        while a - p < -math.pi:
            a += 2.0 * math.pi
        out.append(a)
    return out


def mean_std_deg(values):
    vals = unwrap(values)
    if not vals:
        return float('nan'), float('nan')
    return fmean(vals) * RAD2DEG, pstdev(vals) * RAD2DEG if len(vals) > 1 else 0.0


def read(path):
    rows = []
    with path.open(newline='') as f:
        rd = csv.DictReader(f)
        for r in rd:
            try:
                rows.append({
                    't': int(r['mono_ns']),
                    'valid': int(r['valid']) != 0,
                    'vx': float(r['vx_mps']),
                    'vy': float(r['vy_mps']),
                    'x': float(r['x_m']),
                    'yaw': float(r['yaw']),
                    'roll': float(r['roll']),
                    'pitch': float(r['pitch']),
                })
            except (ValueError, KeyError):
                continue
    return rows


def analyze(path, static_sec, motion_speed):
    rows = read(path)
    if not rows:
        print(f"{path}: нет данных")
        return
    t0 = rows[0]['t']
    t1 = rows[-1]['t']
    ns = int(static_sec * 1e9)
    first = [r for r in rows if r['t'] <= t0 + ns]
    last = [r for r in rows if r['t'] >= t1 - ns]
    active = [i for i, r in enumerate(rows)
              if r['valid'] and math.hypot(r['vx'], r['vy']) >= motion_speed]
    motion = rows[active[0]:active[-1] + 1] if active else []

    y0, s0 = mean_std_deg([r['yaw'] for r in first])
    y1, s1 = mean_std_deg([r['yaw'] for r in last])
    ym, sm = mean_std_deg([r['yaw'] for r in motion])
    r0, rs0 = mean_std_deg([r['roll'] for r in first])
    r1, rs1 = mean_std_deg([r['roll'] for r in last])
    p0, ps0 = mean_std_deg([r['pitch'] for r in first])
    p1, ps1 = mean_std_deg([r['pitch'] for r in last])
    direction = 'A->B' if rows[-1]['x'] >= 0 else 'B->A'

    print(f"\n=== {path.name}  {direction} ===")
    print(f"yaw static START={y0:+7.3f} deg std={s0:.3f}; END={y1:+7.3f} deg std={s1:.3f}; delta={y1-y0:+.3f} deg")
    print(f"yaw motion mean={ym:+7.3f} deg std={sm:.3f}")
    print(f"roll static START/END={r0:+.3f}/{r1:+.3f} deg delta={r1-r0:+.3f} deg")
    print(f"pitch static START/END={p0:+.3f}/{p1:+.3f} deg delta={p1-p0:+.3f} deg")


def main():
    ap = argparse.ArgumentParser(description='Анализ settling yaw между концами рельсы')
    ap.add_argument('csv', nargs='+', type=Path)
    ap.add_argument('--static-sec', type=float, default=2.0,
                    help='длина окна в начале/конце CSV, секунд (default 2)')
    ap.add_argument('--motion-speed', type=float, default=0.02,
                    help='порог движения, м/с (default 0.02)')
    args = ap.parse_args()
    for p in args.csv:
        analyze(p, args.static_sec, args.motion_speed)


if __name__ == '__main__':
    main()
