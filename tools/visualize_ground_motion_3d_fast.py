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


def render(rows, w=960, h=540):
    img = np.full((h, w, 3), 18, dtype=np.uint8)
    cv2.putText(img, "JT-ZERO 3D ROUTE - FAST", (20,34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.78, (245,245,245), 2, cv2.LINE_AA)

    if not rows:
        cv2.putText(img, "WAITING FOR DATA", (28,78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0,210,255), 2, cv2.LINE_AA)
        return img

    rows = clean_height(trim_idle(rows))
    h0 = rows[0]["h"]
    pts = [(r["x"], r["y"], r["h"]-h0) for r in rows]

    # Raw isometric coordinates, before screen scale/translation.
    raw = [(0.866*x - 0.866*y, 0.50*x + 0.50*y - z) for x,y,z in pts]
    us=[p[0] for p in raw]; vs=[p[1] for p in raw]

    # Dedicated plot area leaves room for stats on the right.
    left, top, right, bottom = 28, 65, 700, 510
    pw, ph = right-left, bottom-top
    umin,umax=min(us),max(us); vmin,vmax=min(vs),max(vs)
    du=max(umax-umin,0.08); dv=max(vmax-vmin,0.08)

    # Fit the ACTUAL projected route to the viewport with 12% margins.
    scale=min(pw/(du*1.24), ph/(dv*1.24), 2200.0)
    uc=(umin+umax)*0.5; vc=(vmin+vmax)*0.5
    ox=(left+right)*0.5 - scale*uc
    oy=(top+bottom)*0.5 - scale*vc

    def pp(p):
        return (int(round(ox+scale*p[0])), int(round(oy+scale*p[1])))

    # Compact background grid centered on route bounds.
    grid_col=(42,42,42)
    for frac in (-0.5,-0.25,0,0.25,0.5):
        y=int(round((top+bottom)*0.5 + frac*ph*0.9))
        cv2.line(img,(left,y),(right,y),grid_col,1,cv2.LINE_AA)
        x=int(round((left+right)*0.5 + frac*pw*0.9))
        cv2.line(img,(x,top),(x,bottom),grid_col,1,cv2.LINE_AA)

    # Bound drawing work.
    stride=max(1,len(raw)//700)
    raw_draw=raw[::stride]
    if raw_draw[-1] != raw[-1]:
        raw_draw.append(raw[-1])
    pix=[pp(p) for p in raw_draw]

    if len(pix)>1:
        cv2.polylines(img,[np.asarray(pix,np.int32)],False,(90,190,255),2,cv2.LINE_AA)
    cv2.circle(img,pix[0],6,(255,180,80),-1,cv2.LINE_AA)
    cv2.circle(img,pix[-1],7,(80,220,120),-1,cv2.LINE_AA)

    dx=rows[-1]["x"]-rows[0]["x"]
    dy=rows[-1]["y"]-rows[0]["y"]
    dz=rows[-1]["h"]-rows[0]["h"]
    netxy=math.hypot(dx,dy)
    net3=math.sqrt(dx*dx+dy*dy+dz*dz)
    path=rows[-1]["path"]-rows[0]["path"]
    hmin=min(r["h"] for r in rows); hmax=max(r["h"] for r in rows)

    stats=[
        f"NET XY: {netxy*1000:.1f} mm",
        f"NET 3D: {net3*1000:.1f} mm",
        f"PATH XY: {path*1000:.1f} mm",
        f"HEIGHT: {hmin*1000:.0f}..{hmax*1000:.0f} mm",
        f"POINTS: {len(rows)}",
    ]
    for i,s in enumerate(stats):
        cv2.putText(img,s,(725,105+i*37),
                    cv2.FONT_HERSHEY_SIMPLEX,0.48,(230,230,230),1,cv2.LINE_AA)
    cv2.putText(img,"Q / ESC - EXIT",(725,505),
                cv2.FONT_HERSHEY_SIMPLEX,0.46,(190,190,190),1,cv2.LINE_AA)
    return img

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--live",action="store_true")
    ap.add_argument("--fps",type=float,default=20.0)
    args=ap.parse_args()

    tail=CsvTail(args.csv)
    cv2.namedWindow(WIN,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN,960,540)
    cv2.moveWindow(WIN,0,0)

    period=1.0/max(1.0,min(args.fps,30.0))
    last_count=-1
    last_img=None
    while True:
        t0=time.monotonic()
        tail.update()
        if len(tail.rows)!=last_count or last_img is None:
            last_img=render(tail.rows)
            last_count=len(tail.rows)
        cv2.imshow(WIN,last_img)
        k=cv2.waitKey(1)
        if k in (27,ord("q"),ord("Q")): break
        if cv2.getWindowProperty(WIN,cv2.WND_PROP_VISIBLE)<1: break
        dt=time.monotonic()-t0
        if dt<period: time.sleep(period-dt)

    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
