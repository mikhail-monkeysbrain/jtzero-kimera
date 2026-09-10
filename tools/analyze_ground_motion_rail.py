#!/usr/bin/env python3
"""Анализ повторных Ground Motion прогонов по рельсе с жёсткими ограничителями.

Без pandas и сторонних зависимостей. Скрипт не меняет данные и параметры.

Пример:
  python3 tools/analyze_ground_motion_rail.py --expected-mm 500 \
    /home/vio/jtzero_runs/20260910_220145_GROUND_MOTION_MVP.csv \
    /home/vio/jtzero_runs/20260910_220234_GROUND_MOTION_MVP.csv

Окно физического движения оценивается по первому/последнему валидному кадру,
где |v_xy| >= --motion-speed. Все кадры между этими моментами считаются частью
движения, включая invalid-кадры. Это позволяет увидеть, не сконцентрированы ли
потери трекинга именно во время перемещения.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional


RAD2DEG = 180.0 / math.pi


def mean(values: Iterable[float]) -> float:
    vals = list(values)
    return statistics.fmean(vals) if vals else float("nan")


def stdev(values: Iterable[float]) -> float:
    vals = list(values)
    return statistics.stdev(vals) if len(vals) >= 2 else 0.0


def percentile(values: Iterable[float], q: float) -> float:
    vals = sorted(values)
    if not vals:
        return float("nan")
    if len(vals) == 1:
        return vals[0]
    p = (len(vals) - 1) * q
    lo = int(math.floor(p))
    hi = int(math.ceil(p))
    if lo == hi:
        return vals[lo]
    u = p - lo
    return vals[lo] * (1.0 - u) + vals[hi] * u


def unwrap_angles(values: List[float]) -> List[float]:
    if not values:
        return []
    out = [values[0]]
    for a in values[1:]:
        prev = out[-1]
        while a - prev > math.pi:
            a -= 2.0 * math.pi
        while a - prev < -math.pi:
            a += 2.0 * math.pi
        out.append(a)
    return out


@dataclass
class Row:
    t_ns: int
    valid: bool
    quality: float
    dx: float
    dy: float
    vx: float
    vy: float
    x: float
    y: float
    h: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class RunResult:
    path: Path
    direction: str
    rows: int
    valid_pct: float
    net_mm: float
    scale: float
    final_x_mm: float
    final_y_mm: float
    heading_deg: float
    path_mm: float
    path_excess_mm: float
    motion_rows: int
    motion_valid_pct: float
    motion_quality_mean: float
    motion_quality_p10: float
    longest_invalid_frames: int
    longest_invalid_ms: float
    invalid_runs: int
    height_mean_mm: float
    height_p10_mm: float
    height_p90_mm: float
    roll_span_deg: float
    pitch_span_deg: float
    yaw_start_deg: float
    yaw_end_deg: float
    yaw_delta_deg: float
    yaw_span_deg: float


def read_rows(path: Path) -> List[Row]:
    rows: List[Row] = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        required = {
            "mono_ns", "valid", "quality", "dx_m", "dy_m", "vx_mps", "vy_mps",
            "x_m", "y_m", "height_m", "roll", "pitch", "yaw"
        }
        missing = required.difference(rd.fieldnames or [])
        if missing:
            raise RuntimeError(f"{path}: нет колонок: {', '.join(sorted(missing))}")
        for r in rd:
            try:
                rows.append(
                    Row(
                        t_ns=int(r["mono_ns"]),
                        valid=(int(r["valid"]) != 0),
                        quality=float(r["quality"]),
                        dx=float(r["dx_m"]),
                        dy=float(r["dy_m"]),
                        vx=float(r["vx_mps"]),
                        vy=float(r["vy_mps"]),
                        x=float(r["x_m"]),
                        y=float(r["y_m"]),
                        h=float(r["height_m"]),
                        roll=float(r["roll"]),
                        pitch=float(r["pitch"]),
                        yaw=float(r["yaw"]),
                    )
                )
            except (ValueError, TypeError) as e:
                raise RuntimeError(f"{path}: плохая строка CSV: {e}") from e
    return rows


def motion_window(rows: List[Row], speed_threshold: float) -> Optional[tuple[int, int]]:
    active = [
        i for i, r in enumerate(rows)
        if r.valid and math.hypot(r.vx, r.vy) >= speed_threshold
    ]
    if not active:
        return None
    return active[0], active[-1]


def invalid_streak_stats(rows: List[Row], i0: int, i1: int) -> tuple[int, float, int]:
    longest_frames = 0
    longest_ms = 0.0
    runs = 0
    start: Optional[int] = None

    def close(end_idx: int) -> None:
        nonlocal longest_frames, longest_ms, runs, start
        if start is None:
            return
        runs += 1
        frames = end_idx - start + 1
        longest_frames = max(longest_frames, frames)
        # Время между соседними валидными/оконными кадрами лучше оценивает потерянный интервал.
        t0 = rows[start].t_ns
        t1 = rows[end_idx].t_ns
        if end_idx + 1 < len(rows):
            t1 = rows[end_idx + 1].t_ns
        ms = max(0.0, (t1 - t0) * 1e-6)
        longest_ms = max(longest_ms, ms)
        start = None

    for i in range(i0, i1 + 1):
        if not rows[i].valid:
            if start is None:
                start = i
        elif start is not None:
            close(i - 1)
    if start is not None:
        close(i1)
    return longest_frames, longest_ms, runs


def analyze(path: Path, expected_m: float, speed_threshold: float) -> RunResult:
    rows = read_rows(path)
    if not rows:
        raise RuntimeError(f"{path}: пустой CSV")

    valid_count = sum(r.valid for r in rows)
    x = rows[-1].x
    y = rows[-1].y
    net = math.hypot(x, y)
    direction = "A->B" if x >= 0 else "B->A"
    heading = math.degrees(math.atan2(y, x))
    if direction == "B->A":
        # Для сравнения направления рельсы разворачиваем обратный вектор на 180°.
        heading = math.degrees(math.atan2(-y, -x))

    path_len = sum(math.hypot(r.dx, r.dy) for r in rows if r.valid)
    win = motion_window(rows, speed_threshold)
    if win is None:
        i0, i1 = 0, len(rows) - 1
    else:
        i0, i1 = win
    motion = rows[i0 : i1 + 1]
    mv_valid = [r for r in motion if r.valid]
    q = [r.quality for r in mv_valid]
    hs = [r.h for r in motion if math.isfinite(r.h) and r.h > 0.0]
    rolls = [r.roll for r in motion if math.isfinite(r.roll)]
    pitches = [r.pitch for r in motion if math.isfinite(r.pitch)]
    yaws = unwrap_angles([r.yaw for r in motion if math.isfinite(r.yaw)])
    longest_frames, longest_ms, invalid_runs = invalid_streak_stats(rows, i0, i1)

    def span_deg(vals: List[float]) -> float:
        return (max(vals) - min(vals)) * RAD2DEG if vals else float("nan")

    yaw_start = yaws[0] * RAD2DEG if yaws else float("nan")
    yaw_end = yaws[-1] * RAD2DEG if yaws else float("nan")

    return RunResult(
        path=path,
        direction=direction,
        rows=len(rows),
        valid_pct=100.0 * valid_count / len(rows),
        net_mm=net * 1000.0,
        scale=net / expected_m,
        final_x_mm=x * 1000.0,
        final_y_mm=y * 1000.0,
        heading_deg=heading,
        path_mm=path_len * 1000.0,
        path_excess_mm=(path_len - net) * 1000.0,
        motion_rows=len(motion),
        motion_valid_pct=(100.0 * len(mv_valid) / len(motion)) if motion else 0.0,
        motion_quality_mean=mean(q),
        motion_quality_p10=percentile(q, 0.10),
        longest_invalid_frames=longest_frames,
        longest_invalid_ms=longest_ms,
        invalid_runs=invalid_runs,
        height_mean_mm=mean(hs) * 1000.0,
        height_p10_mm=percentile(hs, 0.10) * 1000.0,
        height_p90_mm=percentile(hs, 0.90) * 1000.0,
        roll_span_deg=span_deg(rolls),
        pitch_span_deg=span_deg(pitches),
        yaw_start_deg=yaw_start,
        yaw_end_deg=yaw_end,
        yaw_delta_deg=(yaw_end - yaw_start),
        yaw_span_deg=span_deg(yaws),
    )


def print_run(r: RunResult, expected_mm: float) -> None:
    print(f"\n=== {r.path.name}  {r.direction} ===")
    print(
        f"NET={r.net_mm:7.1f} mm  err={r.net_mm-expected_mm:+6.1f} mm "
        f"({(r.scale-1.0)*100:+5.2f}%)  xy=({r.final_x_mm:+7.1f},{r.final_y_mm:+7.1f}) mm"
    )
    print(
        f"heading(rail)={r.heading_deg:+6.2f} deg  PATH={r.path_mm:7.1f} mm  "
        f"PATH-NET={r.path_excess_mm:6.1f} mm"
    )
    print(
        f"valid all={r.valid_pct:5.1f}%  motion={r.motion_valid_pct:5.1f}%  "
        f"q_motion mean={r.motion_quality_mean:.3f} p10={r.motion_quality_p10:.3f}"
    )
    print(
        f"invalid in motion: runs={r.invalid_runs} longest={r.longest_invalid_frames} frames / "
        f"{r.longest_invalid_ms:.1f} ms"
    )
    print(
        f"height motion mean={r.height_mean_mm:.1f} mm p10..p90={r.height_p10_mm:.1f}..{r.height_p90_mm:.1f} mm"
    )
    print(
        f"att motion spans: roll={r.roll_span_deg:.3f} deg pitch={r.pitch_span_deg:.3f} deg "
        f"yaw={r.yaw_span_deg:.3f} deg; yaw start/end/delta="
        f"{r.yaw_start_deg:+.2f}/{r.yaw_end_deg:+.2f}/{r.yaw_delta_deg:+.2f} deg"
    )


def print_group(results: List[RunResult], expected_mm: float) -> None:
    print("\n===== СВОДКА =====")
    for direction in ("A->B", "B->A"):
        rr = [r for r in results if r.direction == direction]
        if not rr:
            continue
        nets = [r.net_mm for r in rr]
        heads = [r.heading_deg for r in rr]
        print(
            f"{direction}: n={len(rr)} mean NET={mean(nets):.1f} mm "
            f"err={mean(nets)-expected_mm:+.1f} mm ({(mean(nets)/expected_mm-1)*100:+.2f}%) "
            f"STD={stdev(nets):.1f} mm; heading mean={mean(heads):.2f} deg STD={stdev(heads):.2f} deg"
        )

    # Файлы передаются хронологически. Если направление чередуется, считаем пары соседей.
    print("\nПарная обратимость соседних A/B запусков:")
    pair_no = 0
    for a, b in zip(results[0::2], results[1::2]):
        if a.direction == b.direction:
            continue
        ab = a if a.direction == "A->B" else b
        ba = b if b.direction == "B->A" else a
        pair_no += 1
        sx = ab.final_x_mm + ba.final_x_mm
        sy = ab.final_y_mm + ba.final_y_mm
        closure = math.hypot(sx, sy)
        sym_x = 0.5 * (ab.final_x_mm - ba.final_x_mm)
        sym_y = 0.5 * (ab.final_y_mm - ba.final_y_mm)
        sym_net = math.hypot(sym_x, sym_y)
        heading_delta = ab.heading_deg - ba.heading_deg
        while heading_delta > 180.0:
            heading_delta -= 360.0
        while heading_delta < -180.0:
            heading_delta += 360.0
        print(
            f"pair {pair_no}: closure=({sx:+.1f},{sy:+.1f}) mm |C|={closure:.1f} mm; "
            f"sym_NET={sym_net:.1f} mm; dHeading={heading_delta:+.2f} deg"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Анализ Ground Motion прогонов по рельсе")
    ap.add_argument("csv", nargs="+", type=Path, help="CSV-файлы в хронологическом порядке")
    ap.add_argument("--expected-mm", type=float, default=500.0, help="Ход между ограничителями, мм")
    ap.add_argument(
        "--motion-speed", type=float, default=0.02,
        help="Порог |v_xy| для границ окна движения, м/с (по умолчанию 0.02)"
    )
    args = ap.parse_args()
    expected_m = args.expected_mm / 1000.0
    results: List[RunResult] = []
    for p in args.csv:
        r = analyze(p, expected_m, args.motion_speed)
        results.append(r)
        print_run(r, args.expected_mm)
    print_group(results, args.expected_mm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
