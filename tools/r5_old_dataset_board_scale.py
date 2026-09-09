#!/usr/bin/env python3
import argparse
import statistics
from pathlib import Path

import cv2
import numpy as np


def collect_views(root, square_mm, marker_mm):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        (7, 5), square_mm / 1000.0, marker_mm / 1000.0, dictionary
    )
    detector = cv2.aruco.CharucoDetector(board)
    board_points = np.asarray(board.getChessboardCorners(), dtype=np.float32)

    obj_points = []
    img_points = []
    rejected = 0

    images = sorted(root.glob("frame_*.png"))
    for path in images:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.shape != (480, 640):
            rejected += 1
            continue

        cc, ci, _, _ = detector.detectBoard(gray)
        if ci is None or cc is None or len(ci) < 12:
            rejected += 1
            continue

        ids = np.asarray(ci, dtype=np.int32).reshape(-1)
        pts = np.asarray(cc, dtype=np.float32).reshape(-1, 2)
        obj = board_points[ids].reshape(-1, 3)

        obj_points.append(obj)
        img_points.append(pts)

    return images, obj_points, img_points, rejected


def fit(root, square_mm, marker_mm):
    images, obj, img, rejected = collect_views(root, square_mm, marker_mm)
    if len(obj) < 10:
        raise RuntimeError(f"only {len(obj)} usable views")

    K = np.eye(3, dtype=np.float64)
    K[0, 2] = (640 - 1) * 0.5
    K[1, 2] = (480 - 1) * 0.5
    D = np.zeros((5, 1), dtype=np.float64)

    criteria = (
        cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS,
        100,
        1e-12,
    )

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj, img, (640, 480), K, D, flags=0, criteria=criteria
    )

    per_view = []
    for op, ip, rv, tv in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, D)
        e = proj.reshape(-1, 2) - ip.reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(e * e, axis=1)))))

    return {
        "input_images": len(images),
        "usable": len(obj),
        "rejected": rejected,
        "rms": float(rms),
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "D": D.reshape(-1).tolist(),
        "per_view_median": statistics.median(per_view),
    }


def show(label, r):
    print(label)
    print("-" * 100)
    print(
        f"images={r['input_images']} usable={r['usable']} rejected={r['rejected']} "
        f"RMS={r['rms']:.6f}px"
    )
    print(
        f"fx={r['fx']:.9f} fy={r['fy']:.9f} "
        f"cx={r['cx']:.9f} cy={r['cy']:.9f}"
    )
    print("D=[" + ", ".join(f"{x:.9f}" for x in r["D"]) + "]")
    print(f"per_view_median={r['per_view_median']:.6f}px")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="/home/vio/charuco_calibration")
    args = ap.parse_args()
    root = Path(args.input).expanduser()

    print("=" * 100)
    print("R5 — OLD OV9281 DATASET: BOARD-SCALE INVARIANCE CHECK")
    print("=" * 100)

    old = fit(root, 27.324, 20.043)
    actual = fit(root, 26.470, 19.411)

    show("ORIGINAL BOARD MODEL 27.324 / 20.043 mm", old)
    print()
    show("ACTUAL BOARD MODEL 26.470 / 19.411 mm", actual)

    print("\nCROSS-CHECK")
    print("-" * 100)

    fx_ratio = actual["fx"] / old["fx"]
    fy_ratio = actual["fy"] / old["fy"]
    print(f"fx ratio actual/original = {fx_ratio:.9f} ({(fx_ratio-1)*100:+.6f}%)")
    print(f"fy ratio actual/original = {fy_ratio:.9f} ({(fy_ratio-1)*100:+.6f}%)")
    print(f"R1 required focal multiplier = 1.092500 (+9.250%)")
    print(f"MOVE500 equivalent multiplier = 1.105610 (+10.561%)")

    max_delta = max(abs(fx_ratio - 1.0), abs(fy_ratio - 1.0)) * 100.0
    if max_delta < 0.5:
        print(
            "VERDICT: BOARD-SCALE EFFECT EXCLUDED — corrected physical target dimensions "
            "do not materially change fitted focal length."
        )
    else:
        print(
            "VERDICT: BOARD MODEL CHANGES FIT — investigate target geometry before "
            "changing camera intrinsics."
        )
    print("=" * 100)


if __name__ == "__main__":
    main()
