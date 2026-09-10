#!/usr/bin/env python3
"""Диагностика влияния FC yaw на направление Ground Motion по рельсе.

Использует уже записанные CSV, ничего не меняет в estimator/FC.
Для каждого valid шага dx/dy строит два диагностических интеграла:
  R(+yaw) * dxy  — ожидаемый yaw-neutral/body-like вариант при текущих соглашениях;
  R(-yaw) * dxy  — контроль знака.
Также сравнивает парную обратимость A->B / B->A до и после yaw-neutral преобразования.

Без pandas и сторонних зависимостей.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def circ_mean_rad(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    s = sum(math.sin(v) for v in vals)
    c = sum(math.cos(v) for v in vals)
    return math.atan2(s, c)


def heading_deg(x: float, y: float, reverse_to_forward: bool = False) -> float:
    if reverse_to_forward:
        x, y = -x, -y
    return math.degrees(math.atan2(y, x))


def rot(x: float, y: float, a: float) -> tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


@dataclass
class Row:
    t_ns: int
    valid: bool
    dx: float
    dy: float
    vx: float
    vy: float
    yaw: float


@dataclass
class Result:
    path: Path
    direction: str
    raw_x: float
    raw_y: float
    plus_x: float
    plus_y: float
    minus_x: float
    minus_y: float
    yaw_motion: float
    motion_valid: int
    motion_total: int


def read_rows(path: Path) -> List[Row]:
    out: List[Row] = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        req = {"mono_ns", "valid", "dx_m", "dy_m", "vx_mps", "vy_mps", "yaw"}
        miss = req.difference(rd.fieldnames or [])
        if miss:
            raise RuntimeError(f"{path}: нет колонок: {', '.join(sorted(miss))}")
        for r in rd:
            out.append(Row(
                t_ns=int(r["mono_ns"]),
                valid=int(r["valid"]) != 0,
                dx=float(r["dx_m"]),
                dy=float(r["dy_m"]),
                vx=float(r["vx_mps"]),
                vy=float(r["vy_mps"]),
                yaw=float(r["yaw"]),
            ))
    return out


def motion_window(rows: List[Row], speed: float) -> Optional[tuple[int, int]]:
    idx = [i for i, r in enumerate(rows) if r.valid and math.hypot(r.vx, r.vy) >= speed]
    if not idx:
        return None
    return idx[0], idx[-1]


def analyze(path: Path, speed: float) -> Result:
    rows = read_rows(path)
    raw_x = sum(r.dx for r in rows if r.valid)
    raw_y = sum(r.dy for r in rows if r.valid)
    direction = "A->B" if raw_x >= 0 else "B->A"

    win = motion_window(rows, speed)
    if win is None:
        i0, i1 = 0, len(rows) - 1
    else:
        i0, i1 = win

    plus_x = plus_y = minus_x = minus_y = 0.0
    yaws: List[float] = []
    mv_valid = 0
    for i in range(i0, i1 + 1):
        r = rows[i]
        if not r.valid:
            continue
        mv_valid += 1
        yaws.append(r.yaw)
        px, py = rot(r.dx, r.dy, +r.yaw)
        mx, my = rot(r.dx, r.dy, -r.yaw)
        plus_x += px; plus_y += py
        minus_x += mx; minus_y += my

    return Result(
        path=path,
        direction=direction,
        raw_x=raw_x,
        raw_y=raw_y,
        plus_x=plus_x,
        plus_y=plus_y,
        minus_x=minus_x,
        minus_y=minus_y,
        yaw_motion=circ_mean_rad(yaws),
        motion_valid=mv_valid,
        motion_total=max(0, i1 - i0 + 1),
    )


def vec_line(label: str, x: float, y: float, reverse: bool) -> str:
    h = heading_deg(x, y, reverse)
    return f"{label:<14} xy=({x*1000:+7.1f},{y*1000:+7.1f}) mm NET={math.hypot(x,y)*1000:7.1f} mm heading={h:+7.2f} deg"


def pair_closure(a: Result, b: Result, attrx: str, attry: str) -> tuple[float, float, float, float]:
    ax, ay = getattr(a, attrx), getattr(a, attry)
    bx, by = getattr(b, attrx), getattr(b, attry)
    sx, sy = ax + bx, ay + by
    return sx, sy, math.hypot(sx, sy), math.degrees(math.atan2(sy, sx))


def main() -> int:
    ap = argparse.ArgumentParser(description="Диагностика yaw-neutral Ground Motion")
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--motion-speed", type=float, default=0.02)
    args = ap.parse_args()

    rr = [analyze(p, args.motion_speed) for p in args.csv]
    for r in rr:
        reverse = r.direction == "B->A"
        raw_h = heading_deg(r.raw_x, r.raw_y, reverse)
        yaw_deg = math.degrees(r.yaw_motion)
        print(f"\n=== {r.path.name}  {r.direction} ===")
        print(vec_line("RAW", r.raw_x, r.raw_y, reverse))
        print(f"yaw motion mean={yaw_deg:+7.2f} deg; RAW heading + yaw={wrap_deg(raw_h+yaw_deg):+7.2f} deg; RAW heading - yaw={wrap_deg(raw_h-yaw_deg):+7.2f} deg")
        print(vec_line("R(+yaw)", r.plus_x, r.plus_y, reverse))
        print(vec_line("R(-yaw)", r.minus_x, r.minus_y, reverse))
        pct = 100.0 * r.motion_valid / r.motion_total if r.motion_total else 0.0
        print(f"motion valid={r.motion_valid}/{r.motion_total} ({pct:.1f}%)")

    print("\n===== ПАРНАЯ ОБРАТИМОСТЬ =====")
    pair_no = 0
    for a, b in zip(rr[0::2], rr[1::2]):
        if a.direction == b.direction:
            continue
        pair_no += 1
        print(f"pair {pair_no}:")
        for label, ax, ay in (
            ("RAW", "raw_x", "raw_y"),
            ("R(+yaw)", "plus_x", "plus_y"),
            ("R(-yaw)", "minus_x", "minus_y"),
        ):
            sx, sy, mag, _ = pair_closure(a, b, ax, ay)
            print(f"  {label:<9} closure=({sx*1000:+7.1f},{sy*1000:+7.1f}) mm |C|={mag*1000:6.1f} mm")

    # Сводка по направлениям для yaw и yaw-neutral heading.
    print("\n===== СВОДКА НАПРАВЛЕНИЙ =====")
    for direction in ("A->B", "B->A"):
        g = [r for r in rr if r.direction == direction]
        if not g:
            continue
        raw_heads = [heading_deg(r.raw_x, r.raw_y, direction == "B->A") for r in g]
        plus_heads = [heading_deg(r.plus_x, r.plus_y, direction == "B->A") for r in g]
        yaws = [math.degrees(r.yaw_motion) for r in g]
        def m(v): return statistics.fmean(v)
        def sd(v): return statistics.stdev(v) if len(v) > 1 else 0.0
        print(
            f"{direction}: yaw={m(yaws):+.2f}±{sd(yaws):.2f} deg; "
            f"RAW heading={m(raw_heads):+.2f}±{sd(raw_heads):.2f} deg; "
            f"R(+yaw) heading={m(plus_heads):+.2f}±{sd(plus_heads):.2f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
