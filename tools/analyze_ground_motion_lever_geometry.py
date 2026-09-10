#!/usr/bin/env python3
"""JT-Zero: проверка влияния TF-Luna -> OV9281 lever arm на высоту камеры.

Методика:
- используется та же FLU/NWU математика, что и live V3;
- текущая V3 высота считается высотой точки TF-Luna;
- добавляется вертикальная компонента R_W_B * t_LC, где t_LC — вектор от
  оптической точки TF-Luna к центру проекции камеры в body FLU;
- по умолчанию используется только надёжно измеренный из CAD горизонтальный
  lever arm: +49.16 мм вдоль body +X. Z намеренно = 0, потому что точное
  положение pinhole-центра OV9281 вдоль оптической оси из STEP не известно.

Скрипт НЕ подгоняет результат к 500 мм. Он только показывает геометрическую
поправку высоты и грубый scale-only эффект на уже полученный NET.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path


def matmul3(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def matvec3(a, v):
    return [sum(a[r][k] * v[k] for k in range(3)) for r in range(3)]


def rzryrx(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def attitude_flu_to_nwu(r, p, y):
    s = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    return matmul3(matmul3(s, rzryrx(r, p, y)), s)


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    if len(s) == 1:
        return s[0]
    x = (len(s) - 1) * q
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return s[lo]
    u = x - lo
    return s[lo] * (1.0 - u) + s[hi] * u


def main():
    ap = argparse.ArgumentParser(description="CAD lever-arm анализ для Ground Motion V3")
    ap.add_argument("csv", type=Path, help="CSV из GROUND_MOTION_V3_SYNC_AB")
    ap.add_argument("--tx-mm", type=float, default=49.16,
                    help="TF-Luna -> camera, body FLU +X, мм (default: 49.16)")
    ap.add_argument("--ty-mm", type=float, default=0.22,
                    help="TF-Luna -> camera, body FLU +Y, мм (default: 0.22)")
    ap.add_argument("--tz-mm", type=float, default=0.0,
                    help="TF-Luna -> camera, body FLU +Z, мм; default 0 — не угадываем pinhole Z")
    args = ap.parse_args()

    t = [args.tx_mm / 1000.0, args.ty_mm / 1000.0, args.tz_mm / 1000.0]

    rows = []
    with args.csv.open(newline="") as f:
        rd = csv.DictReader(f)
        required = {"state", "sync_h_m", "sync_roll", "sync_pitch", "sync_yaw", "sync_x_m", "sync_y_m"}
        missing = required - set(rd.fieldnames or [])
        if missing:
            raise SystemExit("CSV не похож на V3 SYNC A/B; нет колонок: " + ", ".join(sorted(missing)))
        for r in rd:
            try:
                if int(float(r["state"])) != 1:
                    continue
                h = float(r["sync_h_m"])
                roll = float(r["sync_roll"])
                pitch = float(r["sync_pitch"])
                yaw = float(r["sync_yaw"])
                sx = float(r["sync_x_m"])
                sy = float(r["sync_y_m"])
            except (ValueError, KeyError):
                continue
            if h <= 0 or not all(math.isfinite(x) for x in (h, roll, pitch, yaw, sx, sy)):
                continue
            rw_b = attitude_flu_to_nwu(roll, pitch, yaw)
            dz = matvec3(rw_b, t)[2]
            hc = h + dz
            rows.append((h, dz, hc, roll, pitch, yaw, sx, sy))

    if not rows:
        raise SystemExit("В state=1 нет пригодных строк")

    hs = [r[0] for r in rows]
    dzs = [r[1] for r in rows]
    hcs = [r[2] for r in rows]
    pitches = [math.degrees(r[4]) for r in rows]
    rolls = [math.degrees(r[3]) for r in rows]

    # Последняя запись state=1 — endpoint текущего V3 SYNC.
    sx, sy = rows[-1][6], rows[-1][7]
    net = math.hypot(sx, sy)

    # Это только scale-only sanity check; точная ветка V3 должна пересчитать
    # footprints каждого кадра с h_cam(t).
    ratios = [hc / h for h, _, hc, *_ in rows if h > 0]
    scale_ratio = statistics.median(ratios)
    rough_net = net * scale_ratio

    print("================ JT-ZERO CAD LEVER GEOMETRY ================")
    print(f"CSV: {args.csv}")
    print(f"t_LC body FLU [mm] = [{args.tx_mm:+.2f}, {args.ty_mm:+.2f}, {args.tz_mm:+.2f}]")
    print(f"measurement rows    = {len(rows)}")
    print()
    print(f"pitch median/span   = {statistics.median(pitches):+.3f} deg / {min(pitches):+.3f} .. {max(pitches):+.3f}")
    print(f"roll  median/span   = {statistics.median(rolls):+.3f} deg / {min(rolls):+.3f} .. {max(rolls):+.3f}")
    print(f"V3 sync H median    = {statistics.median(hs)*1000:.3f} mm")
    print(f"lever dH median     = {statistics.median(dzs)*1000:+.3f} mm")
    print(f"lever dH p05/p95    = {pct(dzs,0.05)*1000:+.3f} / {pct(dzs,0.95)*1000:+.3f} mm")
    print(f"H camera median     = {statistics.median(hcs)*1000:.3f} mm")
    print()
    print(f"SYNC endpoint NET   = {net*1000:.3f} mm")
    print(f"median H ratio      = {scale_ratio:.6f}")
    print(f"rough scale-only NET= {rough_net*1000:.3f} mm")
    print("=============================================================")
    print("ПРИМЕЧАНИЕ: rough scale-only NET — не новый результат V3.")
    print("Для строгого A/B надо пересчитать ray-plane footprints с H_camera на каждом кадре.")


if __name__ == "__main__":
    main()
