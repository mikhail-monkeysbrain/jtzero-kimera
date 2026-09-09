#!/usr/bin/env python3
# JT-Zero: lightweight real-time 3D demo.
# OpenCV only. Reads ONLY newly appended CSV rows; no matplotlib and no full-file reread.

import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np

WIN = "JT-Zero — 3D маршрут — БЫСТРЫЙ РЕЖИМ"


class CsvTail:
    def __init__(self, path):
        self.path = Path(path)
        self.fp = None
        self.header = None
        self.rows = []
        self.pos = 0

    def open_if_ready(self):
        if self.fp is not None:
            return True
        if not self.path.exists():
            return False
        self.fp = self.path.open("r", newline="")
        line = self.fp.readline()
        if not line:
            self.fp.close(); self.fp = None
            return False
        self.header = next(csv.reader([line]))
        self.pos = self.fp.tell()
        return True

    def update(self):
        if not self.open_if_ready():
            return
        self.fp.seek(self.pos)
        while True:
            line = self.fp.readline()
            if not line:
                break
            self.pos = self.fp.tell()
            try:
                vals = next(csv.reader([line]))
                if len(vals) != len(self.header):
                    continue
                r = dict(zip(self.header, vals))
                self.rows.append({
                    "x": float(r["x_m"]),
                    "y": float(r["y_m"]),
                    "h": float(r["height_m"]),
                    "path": float(r.get("path_m", "0") or 0),
                })
            except Exception:
                continue

        # Bounded memory for demo. Keep enough route history.
        if len(self.rows) > 5000:
            self.rows = self.rows[-5000:]


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
    # Use recent window for robust current-height reference.
    recent = rows[-101:]
    med = float(np.median([r["h"] for r in recent]))
    lim = max(0.10, 0.50 * max(med, 0.05))
    out = []
    prev = med
    for r in rows:
        q = dict(r)
        if abs(q["h"] - med) > lim:
            q["h"] = prev
        else:
            prev = q["h"]
        out.append(q)
    return out


def project_iso(x, y, z, scale, ox, oy):
    return (
        int(round(ox + scale * (0.866*x - 0.866*y))),
        int(round(oy + scale * (0.50*x + 0.50*y - z))),
    )


def render(rows, w=1280, h=720):
    img = np.full((h, w, 3), 18, dtype=np.uint8)
    cv2.putText(img, "JT-ZERO — 3D МАРШРУТ — БЫСТРЫЙ РЕЖИМ", (24,40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.82, (245,245,245), 2, cv2.LINE_AA)

    if not rows:
        cv2.putText(img, "ОЖИДАНИЕ ДАННЫХ", (36,95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0,210,255), 2, cv2.LINE_AA)
        return img

    rows = clean_height(trim_idle(rows))
    h0 = rows[0]["h"]
    pts = [(r["x"], r["y"], r["h"]-h0) for r in rows]
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]

    span=max(max(xs)-min(xs) if len(xs)>1 else 0,
             max(ys)-min(ys) if len(ys)>1 else 0,
             max(zs)-min(zs) if len(zs)>1 else 0, 0.10)
    scale=min(700.0/span, 1600.0)
    ox,oy=560,430

    # lightweight grid
    grid=max(0.10,min(0.50,span))
    step=grid/4
    for i in range(-4,5):
        a=project_iso(i*step,-4*step,0,scale,ox,oy); b=project_iso(i*step,4*step,0,scale,ox,oy)
        cv2.line(img,a,b,(42,42,42),1,cv2.LINE_AA)
        a=project_iso(-4*step,i*step,0,scale,ox,oy); b=project_iso(4*step,i*step,0,scale,ox,oy)
        cv2.line(img,a,b,(42,42,42),1,cv2.LINE_AA)

    # At most 800 segments for display.
    stride=max(1,len(pts)//800)
    draw_pts=pts[::stride]
    if draw_pts[-1] != pts[-1]:
        draw_pts.append(pts[-1])
    pix=[project_iso(*p,scale,ox,oy) for p in draw_pts]
    if len(pix)>1:
        cv2.polylines(img,[np.asarray(pix,np.int32)],False,(90,190,255),2,cv2.LINE_AA)
    cv2.circle(img,pix[0],6,(255,180,80),-1,cv2.LINE_AA)
    cv2.circle(img,pix[-1],7,(80,220,120),-1,cv2.LINE_AA)

    dx=rows[-1]["x"]-rows[0]["x"]; dy=rows[-1]["y"]-rows[0]["y"]
    dz=rows[-1]["h"]-rows[0]["h"]
    netxy=math.hypot(dx,dy); net3=math.sqrt(dx*dx+dy*dy+dz*dz)
    path=rows[-1]["path"]-rows[0]["path"]
    hmin=min(r["h"] for r in rows); hmax=max(r["h"] for r in rows)

    stats=[
        f"NET XY: {netxy*1000:.1f} мм",
        f"NET 3D: {net3*1000:.1f} мм",
        f"Путь XY: {path*1000:.1f} мм",
        f"Высота: {hmin*1000:.0f}...{hmax*1000:.0f} мм",
        f"Точек: {len(rows)}",
    ]
    for i,s in enumerate(stats):
        cv2.putText(img,s,(935,115+i*42),cv2.FONT_HERSHEY_SIMPLEX,0.62,(230,230,230),1,cv2.LINE_AA)
    cv2.putText(img,"Q / ESC — ВЫХОД",(935,675),cv2.FONT_HERSHEY_SIMPLEX,0.52,(190,190,190),1,cv2.LINE_AA)
    return img


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--live",action="store_true")
    ap.add_argument("--fps",type=float,default=15.0)
    args=ap.parse_args()

    tail=CsvTail(args.csv)
    cv2.namedWindow(WIN,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN,1280,720)
    cv2.moveWindow(WIN,0,0)

    period=1.0/max(1.0,min(args.fps,30.0))
    while True:
        t0=time.monotonic()
        tail.update()
        cv2.imshow(WIN,render(tail.rows))
        k=cv2.waitKey(1)
        if k in (27,ord("q"),ord("Q")): break
        if cv2.getWindowProperty(WIN,cv2.WND_PROP_VISIBLE)<1: break
        if not args.live:
            time.sleep(0.03)
        else:
            dt=time.monotonic()-t0
            if dt<period: time.sleep(period-dt)

    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
