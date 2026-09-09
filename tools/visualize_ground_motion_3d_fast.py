#!/usr/bin/env python3
# Lightweight real-time 3D demo for JT-Zero Ground Motion.
# Uses OpenCV only; no matplotlib. Designed for Raspberry Pi demo use.

import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np

WIN = "JT-Zero — 3D маршрут (быстрый просмотр)"


def newest_csv():
    root = Path.home() / "jtzero_runs"
    files = sorted(root.glob("*_GROUND_MOTION_LIVE.csv"), key=lambda p: p.stat().st_mtime)
    if not files:
        raise SystemExit(f"Не найдено *_GROUND_MOTION_LIVE.csv в {root}")
    return files[-1]


def load_rows(path):
    rows = []
    try:
        with path.open(newline="") as f:
            rd = csv.DictReader(f)
            for r in rd:
                try:
                    rows.append({
                        "x": float(r["x_m"]),
                        "y": float(r["y_m"]),
                        "h": float(r["height_m"]),
                        "path": float(r.get("path_m", "0") or 0),
                    })
                except Exception:
                    continue
    except FileNotFoundError:
        pass
    return rows


def trim_idle(rows):
    if len(rows) < 2:
        return rows
    eps = 1e-6
    first = None
    for i, r in enumerate(rows):
        if abs(r["x"]) > eps or abs(r["y"]) > eps or r["path"] > eps:
            first = max(0, i - 1)
            break
    if first is None:
        return rows[-1:]
    return rows[first:]


def clean_height(rows):
    if len(rows) < 3:
        return rows
    hs = [r["h"] for r in rows]
    med = float(np.median(hs))
    out = []
    prev = med
    lim = max(0.10, 0.50 * max(med, 0.05))
    for r in rows:
        q = dict(r)
        if abs(q["h"] - med) > lim:
            q["h"] = prev
        else:
            prev = q["h"]
        out.append(q)
    return out


def project_iso(x, y, z, scale, ox, oy):
    # Simple isometric projection.
    u = ox + scale * (0.866 * x - 0.866 * y)
    v = oy + scale * (0.50 * x + 0.50 * y - z)
    return int(round(u)), int(round(v))


def draw_axis(img, origin, end, label):
    cv2.line(img, origin, end, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(img, label, (end[0] + 6, end[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220,220,220), 1, cv2.LINE_AA)


def render(rows, w=1280, h=720):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (18,18,18)

    cv2.putText(img, "JT-ZERO — 3D МАРШРУТ", (28,42),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (245,245,245), 2, cv2.LINE_AA)

    if not rows:
        cv2.putText(img, "ОЖИДАНИЕ ДАННЫХ...", (40,100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,210,255), 2, cv2.LINE_AA)
        return img

    h0 = rows[0]["h"]
    pts = [(r["x"], r["y"], r["h"] - h0) for r in rows]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]

    max_span = max(
        max(xs)-min(xs) if len(xs)>1 else 0,
        max(ys)-min(ys) if len(ys)>1 else 0,
        max(zs)-min(zs) if len(zs)>1 else 0,
        0.10
    )

    scale = min(760.0 / max_span, 1800.0)
    ox, oy = 560, 420

    # Reference axes: 0.25 m or current scale fraction.
    axis_len = max(0.10, min(0.50, max_span))
    o = project_iso(0,0,0,scale,ox,oy)
    draw_axis(img, o, project_iso(axis_len,0,0,scale,ox,oy), "X")
    draw_axis(img, o, project_iso(0,axis_len,0,scale,ox,oy), "Y")
    draw_axis(img, o, project_iso(0,0,axis_len,scale,ox,oy), "Z")

    # Ground grid on Z=0.
    step = axis_len / 4.0
    for i in range(-4,5):
        a = project_iso(i*step, -4*step, 0, scale, ox, oy)
        b = project_iso(i*step,  4*step, 0, scale, ox, oy)
        cv2.line(img, a, b, (45,45,45), 1, cv2.LINE_AA)
        a = project_iso(-4*step, i*step, 0, scale, ox, oy)
        b = project_iso( 4*step, i*step, 0, scale, ox, oy)
        cv2.line(img, a, b, (45,45,45), 1, cv2.LINE_AA)

    # Route; downsample only for drawing speed.
    if len(pts) > 1200:
        stride = max(1, len(pts)//1200)
        pts_draw = pts[::stride]
        if pts_draw[-1] != pts[-1]:
            pts_draw.append(pts[-1])
    else:
        pts_draw = pts

    pix = [project_iso(x,y,z,scale,ox,oy) for x,y,z in pts_draw]
    for a,b in zip(pix[:-1],pix[1:]):
        cv2.line(img, a, b, (80,180,255), 2, cv2.LINE_AA)

    if pix:
        cv2.circle(img, pix[0], 6, (255,180,80), -1, cv2.LINE_AA)
        cv2.circle(img, pix[-1], 7, (80,220,120), -1, cv2.LINE_AA)

    dx = rows[-1]["x"] - rows[0]["x"]
    dy = rows[-1]["y"] - rows[0]["y"]
    dz = (rows[-1]["h"] - h0) - (rows[0]["h"] - h0)
    net_xy = math.hypot(dx,dy)
    net_3d = math.sqrt(dx*dx+dy*dy+dz*dz)
    path = rows[-1]["path"] - rows[0]["path"]
    hmin = min(r["h"] for r in rows)
    hmax = max(r["h"] for r in rows)

    panel_x = 930
    stats = [
        f"NET XY: {net_xy*1000:.1f} мм",
        f"NET 3D: {net_3d*1000:.1f} мм",
        f"Путь XY: {path*1000:.1f} мм",
        f"Высота: {hmin*1000:.1f}...{hmax*1000:.1f} мм",
        f"Точек: {len(rows)}",
    ]
    y0 = 110
    for i,s in enumerate(stats):
        cv2.putText(img, s, (panel_x, y0+i*42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.66, (230,230,230), 1, cv2.LINE_AA)

    cv2.putText(img, "СТАРТ", (panel_x, 360),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255,180,80), 2, cv2.LINE_AA)
    cv2.putText(img, "ФИНИШ", (panel_x, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (80,220,120), 2, cv2.LINE_AA)
    cv2.putText(img, "Q / ESC — ВЫХОД", (panel_x, 675),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (190,190,190), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description="Быстрый 3D просмотр Ground Motion")
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--latest", action="store_true")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--fps", type=float, default=10.0, help="частота обновления окна")
    args = ap.parse_args()

    path = newest_csv() if args.latest else Path(args.csv) if args.csv else None
    if path is None:
        raise SystemExit("Укажи CSV или --latest")

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)
    cv2.moveWindow(WIN, 0, 0)

    period = 1.0 / max(1.0, min(args.fps, 30.0))

    while True:
        t0 = time.monotonic()
        rows = clean_height(trim_idle(load_rows(path)))
        img = render(rows)
        cv2.imshow(WIN, img)
        k = cv2.waitKey(1)
        if k in (27, ord("q"), ord("Q")):
            break
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
        if not args.live:
            # Static file: keep window responsive without re-reading CSV.
            while True:
                k = cv2.waitKey(30)
                if k in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    cv2.destroyAllWindows()
                    return
        dt = time.monotonic() - t0
        if dt < period:
            time.sleep(period - dt)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
