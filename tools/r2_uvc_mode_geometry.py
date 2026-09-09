#!/usr/bin/env python3
import argparse
import statistics
import time

import cv2
import numpy as np


def capture_mode(device, fps, square_mm, marker_mm, frames, warmup):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {device}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, fps)

    width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    actual_fps = float(cap.get(cv2.CAP_PROP_FPS))

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        (7, 5), square_mm / 1000.0, marker_mm / 1000.0, dictionary
    )
    detector = cv2.aruco.CharucoDetector(board)
    obj_all = np.asarray(board.getChessboardCorners(), dtype=np.float64)

    for _ in range(warmup):
        ok, _ = cap.read()
        if not ok:
            cap.release()
            raise RuntimeError(f"warmup capture failed at requested {fps} FPS")

    scales = []
    centers = []
    counts = []
    accepted = 0

    while accepted < frames:
        ok, image = cap.read()
        if not ok or image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cc, ci, _, _ = detector.detectBoard(gray)
        if ci is None or cc is None or len(ci) < 8:
            continue

        ids = np.asarray(ci, dtype=np.int32).reshape(-1)
        pts = np.asarray(cc, dtype=np.float64).reshape(-1, 2)
        obj = obj_all[ids, :2]

        local_scales = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                od = float(np.linalg.norm(obj[i] - obj[j]))
                if od < 1e-9:
                    continue
                pd = float(np.linalg.norm(pts[i] - pts[j]))
                local_scales.append(pd / od)

        if not local_scales:
            continue

        scales.append(statistics.median(local_scales))
        centers.append(np.mean(pts, axis=0))
        counts.append(len(ids))
        accepted += 1

    cap.release()

    centers = np.asarray(centers)
    return {
        "requested_fps": fps,
        "actual_fps": actual_fps,
        "width": width,
        "height": height,
        "frames": accepted,
        "corners_med": statistics.median(counts),
        "scale_med": statistics.median(scales),
        "scale_min": min(scales),
        "scale_max": max(scales),
        "center_x_med": float(np.median(centers[:, 0])),
        "center_y_med": float(np.median(centers[:, 1])),
    }


def print_mode(r):
    print(
        f"requested={r['requested_fps']:3d} actual={r['actual_fps']:7.3f} "
        f"size={r['width']}x{r['height']} frames={r['frames']} "
        f"corners_med={r['corners_med']:.1f}"
    )
    print(
        f"  charuco_scale_med={r['scale_med']:.6f} px/m "
        f"range=[{r['scale_min']:.6f},{r['scale_max']:.6f}]"
    )
    print(
        f"  detected_center_med=({r['center_x_med']:.3f},"
        f"{r['center_y_med']:.3f}) px"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--device",
        default="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0",
    )
    ap.add_argument("--square-mm", type=float, default=26.47)
    ap.add_argument("--marker-mm", type=float, default=19.411)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=30)
    args = ap.parse_args()

    print("=" * 100)
    print("R2 — OV9281 100 FPS vs 120 FPS STATIC CHARUCO GEOMETRY")
    print("=" * 100)
    print("Do not move camera, UAV, target, or lens between modes.")

    r100 = capture_mode(
        args.device, 100, args.square_mm, args.marker_mm, args.frames, args.warmup
    )
    time.sleep(0.5)
    r120 = capture_mode(
        args.device, 120, args.square_mm, args.marker_mm, args.frames, args.warmup
    )

    print("\nMODE 100")
    print_mode(r100)
    print("\nMODE 120")
    print_mode(r120)

    ratio = r100["scale_med"] / r120["scale_med"]
    delta_pct = (ratio - 1.0) * 100.0
    print("\nCROSS-CHECK")
    print("-" * 100)
    print(f"pixel-scale ratio 100/120 = {ratio:.6f} ({delta_pct:+.3f}%)")
    print("R1 ChArUco focal multiplier = 1.092500 (+9.250%)")
    print("MOVE500 equivalent multiplier = 1.105610 (+10.561%)")

    if abs(delta_pct) >= 5.0:
        print(
            "VERDICT: MODE-GEOMETRY DIFFERENCE DETECTED — 100 and 120 FPS do not "
            "produce the same effective image geometry."
        )
    elif abs(delta_pct) <= 1.0:
        print(
            "VERDICT: MODE-GEOMETRY MATCH — 100 vs 120 FPS does not explain the "
            "~9–11% focal discrepancy."
        )
    else:
        print(
            "VERDICT: SMALL MODE-GEOMETRY DIFFERENCE — measurable, but insufficient "
            "by itself to explain the full focal discrepancy."
        )
    print("=" * 100)


if __name__ == "__main__":
    main()
