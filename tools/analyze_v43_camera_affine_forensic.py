#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
from pathlib import Path

CX = 315.98271077441063
CY = 239.88148589100641

def q(v, x):
    s = sorted(v)
    return s[min(len(s)-1, max(0, int(round((len(s)-1)*x))))]

def main():
    p = argparse.ArgumentParser(description="Analyze V43 camera affine forensic CSV")
    p.add_argument("--run", help="V43 archive directory")
    p.add_argument("--file", help="Recovered forensic CSV path")
    a = p.parse_args()

    if not a.run and not a.file:
        p.error("one of --run or --file is required")

    f = Path(a.file) if a.file else Path(a.run) / "jtzero_v43_camera_forensic.csv"
    if not f.exists():
        raise SystemExit(f"file not found: {f}")

    with f.open(newline="") as fh:
        rows = list(csv.DictReader(fh))

    if not rows:
        print("=" * 100)
        print("V43 — CAMERA AFFINE FORENSIC")
        print("=" * 100)
        print("accepted updates in forensic CSV: 0")
        print("VERDICT: FORENSIC LOG EMPTY — do not interpret affine statistics.")
        print("=" * 100)
        raise SystemExit(2)

    def vals(k):
        return [float(r[k]) for r in rows]

    tx = vals("tx_px")
    ty = vals("ty_px")
    sc = vals("aff_scale")
    rot = vals("aff_rot_deg")
    h = vals("height_m")
    dx = vals("dx_m")
    dy = vals("dy_m")
    st = vals("step_m")
    c0x = vals("c0x")
    c0y = vals("c0y")

    nx = sum(dx)
    ny = sum(dy)
    net = math.hypot(nx, ny)

    cent_dx = []
    cent_dy = []
    axis_vs_cent = []

    for r in rows:
        A = float(r["aff_a"])
        B = float(r["aff_b"])
        x = float(r["c0x"]) - CX
        y = float(r["c0y"]) - CY
        txx = float(r["tx_px"])
        tyy = float(r["ty_px"])

        # Similarity matrix [[A, -B], [B, A]].
        ddx = txx + (A - 1.0) * x - B * y
        ddy = tyy + B * x + (A - 1.0) * y

        cent_dx.append(ddx)
        cent_dy.append(ddy)

        axis_mag = math.hypot(txx, tyy)
        cent_mag = math.hypot(ddx, ddy)
        axis_vs_cent.append(axis_mag / max(1e-12, cent_mag))

    axis_px_norm = math.hypot(sum(tx), sum(ty))
    cent_px_norm = math.hypot(sum(cent_dx), sum(cent_dy))

    print("=" * 100)
    print("V43 — CAMERA AFFINE FORENSIC")
    print("=" * 100)
    print(f"file: {f}")
    print(f"accepted updates: {len(rows)}")
    print(f"logged camera net: {net*1000:.2f} mm  components=({nx*1000:.2f},{ny*1000:.2f})")

    for name, v in [
        ("height m", h),
        ("affine scale", sc),
        ("|rotation| deg", [abs(x) for x in rot]),
        ("|tx,ty| px", [math.hypot(x,y) for x,y in zip(tx,ty)]),
        ("step mm", [1000*x for x in st]),
        ("centroid x px", c0x),
        ("centroid y px", c0y),
        ("axis/centroid affine-motion ratio", axis_vs_cent),
    ]:
        print(
            f"{name:34s} "
            f"mean={statistics.mean(v):.6f} "
            f"median={statistics.median(v):.6f} "
            f"p90={q(v,.9):.6f} "
            f"max={max(v):.6f}"
        )

    print()
    print("SUMMED PIXEL MOTION")
    print(f"affine translation at optical axis: "
          f"tx={sum(tx):.3f} px ty={sum(ty):.3f} px norm={axis_px_norm:.3f} px")
    print(f"affine-predicted motion at inlier centroid: "
          f"dx={sum(cent_dx):.3f} px dy={sum(cent_dy):.3f} px norm={cent_px_norm:.3f} px")

    print()
    print("500 MM RECONCILIATION")
    print(f"net/truth scale={net/0.5:.6f} error={(net-0.5)*1000:+.2f} mm")
    print(f"path={sum(st)*1000:.2f} mm path/net={sum(st)/net:.6f}")
    print(f"axis/centroid summed pixel norm ratio={axis_px_norm/max(1e-12,cent_px_norm):.6f}")

    print()
    print("INTERPRETATION")
    print("If the summed axis/centroid ratio is close to 1.0, affine parameterization at the optical axis")
    print("cannot explain the ~13% metric excess. If it is near ~1.13, this is a strong direct clue.")
    print("=" * 100)

if __name__ == "__main__":
    main()
