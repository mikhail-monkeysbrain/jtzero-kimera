#!/usr/bin/env python3
# JT-Zero: 3D visualization of Ground Motion route.
# Reads CSV produced by tools/ground_motion_live.cpp.
# X/Y come from camera ground-motion estimator, Z comes from TF-Luna height.
#
# Examples:
#   python3 tools/visualize_ground_motion_3d.py /home/vio/jtzero_runs/..._GROUND_MOTION_LIVE.csv
#   python3 tools/visualize_ground_motion_3d.py --latest
#   python3 tools/visualize_ground_motion_3d.py --latest --live
#   python3 tools/visualize_ground_motion_3d.py run.csv --absolute-height
#   python3 tools/visualize_ground_motion_3d.py run.csv --save route_3d.png

import argparse
import csv
import math
import time
from pathlib import Path

import matplotlib.pyplot as plt


def newest_csv():
    root = Path.home() / "jtzero_runs"
    files = sorted(root.glob("*_GROUND_MOTION_LIVE.csv"), key=lambda p: p.stat().st_mtime)
    if not files:
        raise SystemExit(f"Не найдено файлов *_GROUND_MOTION_LIVE.csv в {root}")
    return files[-1]


def load_rows(path):
    rows = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        need = {"mono_ns", "x_m", "y_m", "height_m"}
        missing = need - set(rd.fieldnames or [])
        if missing:
            raise RuntimeError("В CSV отсутствуют столбцы: " + ", ".join(sorted(missing)))
        for r in rd:
            try:
                rows.append(
                    {
                        "t": int(r["mono_ns"]),
                        "x": float(r["x_m"]),
                        "y": float(r["y_m"]),
                        "h": float(r["height_m"]),
                        "path": float(r.get("path_m", "0") or 0),
                        "inliers": int(float(r.get("inliers", "0") or 0)),
                    }
                )
            except (ValueError, TypeError):
                continue
    return rows


def trim_idle(rows):
    if len(rows) < 3:
        return rows
    eps = 1e-6

    first = None
    for i, r in enumerate(rows):
        if abs(r["x"]) > eps or abs(r["y"]) > eps or r["path"] > eps:
            first = max(0, i - 1)
            break
    if first is None:
        return rows

    last = len(rows) - 1
    for i in range(len(rows) - 1, first, -1):
        a, b = rows[i - 1], rows[i]
        moved = (
            abs(b["x"] - a["x"]) > eps
            or abs(b["y"] - a["y"]) > eps
            or abs(b["path"] - a["path"]) > eps
        )
        if moved:
            last = min(len(rows) - 1, i + 1)
            break
    return rows[first : last + 1]


def equal_3d_axes(ax, xs, ys, zs):
    if not xs:
        return
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    cx, cy, cz = (xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2
    span = max(xmax - xmin, ymax - ymin, zmax - zmin, 0.05)
    half = span * 0.55
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_zlim(cz - half, cz + half)


def draw(ax, rows, absolute_height):
    ax.cla()

    if not rows:
        ax.text2D(0.05, 0.95, "Нет данных", transform=ax.transAxes)
        return

    h0 = rows[0]["h"]
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    zs = [r["h"] if absolute_height else r["h"] - h0 for r in rows]

    ax.plot(xs, ys, zs, linewidth=2.0, label="Маршрут")
    ax.scatter([xs[0]], [ys[0]], [zs[0]], s=55, marker="o", label="Старт")
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=65, marker="X", label="Финиш")

    # Vertical projection onto the start-height plane helps read XY motion.
    zplane = h0 if absolute_height else 0.0
    ax.plot(xs, ys, [zplane] * len(xs), linewidth=1.0, alpha=0.35, label="Проекция XY")

    dx = xs[-1] - xs[0]
    dy = ys[-1] - ys[0]
    dz = zs[-1] - zs[0]
    net3 = math.sqrt(dx * dx + dy * dy + dz * dz)
    netxy = math.hypot(dx, dy)
    path = rows[-1]["path"] - rows[0]["path"]
    hmin, hmax = min(r["h"] for r in rows), max(r["h"] for r in rows)

    ax.set_xlabel("X, м")
    ax.set_ylabel("Y, м")
    ax.set_zlabel("Высота, м" if absolute_height else "ΔZ относительно старта, м")
    ax.set_title("JT-Zero — 3D маршрут")

    info = (
        f"NET XY: {netxy*1000:.1f} мм\n"
        f"NET 3D: {net3*1000:.1f} мм\n"
        f"Путь XY: {path*1000:.1f} мм\n"
        f"Высота: {hmin*1000:.1f}…{hmax*1000:.1f} мм\n"
        f"Точек: {len(rows)}"
    )
    ax.text2D(
        0.02,
        0.98,
        info,
        transform=ax.transAxes,
        va="top",
        bbox=dict(boxstyle="round", alpha=0.15),
    )

    equal_3d_axes(ax, xs, ys, zs)
    ax.legend(loc="upper right")
    ax.grid(True)


def main():
    ap = argparse.ArgumentParser(
        description="3D-визуализация маршрута JT-Zero из Ground Motion CSV"
    )
    ap.add_argument("csv", nargs="?", help="CSV из ground_motion_live")
    ap.add_argument("--latest", action="store_true", help="взять последний *_GROUND_MOTION_LIVE.csv")
    ap.add_argument("--live", action="store_true", help="обновлять график по мере записи CSV")
    ap.add_argument("--absolute-height", action="store_true", help="Z = абсолютная высота TF-Luna; по умолчанию Z относительно старта")
    ap.add_argument("--all", action="store_true", help="не обрезать неподвижный участок до/после измерения")
    ap.add_argument("--refresh-ms", type=int, default=250, help="период обновления в live-режиме")
    ap.add_argument("--save", help="сохранить PNG после построения")
    args = ap.parse_args()

    if args.latest:
        path = newest_csv()
    elif args.csv:
        path = Path(args.csv)
    else:
        raise SystemExit("Укажи CSV или используй --latest")

    if not path.exists():
        raise SystemExit(f"Файл не найден: {path}")

    print(f"CSV: {path}")

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    fig.canvas.manager.set_window_title("JT-Zero — 3D маршрут")

    def reload_and_draw():
        rows = load_rows(path)
        if not args.all:
            rows = trim_idle(rows)
        draw(ax, rows, args.absolute_height)
        fig.tight_layout()
        return rows

    if args.live:
        plt.ion()
        while plt.fignum_exists(fig.number):
            try:
                reload_and_draw()
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(max(0.05, args.refresh_ms / 1000.0))
            except KeyboardInterrupt:
                break
        plt.ioff()
    else:
        rows = reload_and_draw()
        if args.save:
            out = Path(args.save)
            fig.savefig(out, dpi=160, bbox_inches="tight")
            print(f"PNG: {out}")
        plt.show()


if __name__ == "__main__":
    main()
