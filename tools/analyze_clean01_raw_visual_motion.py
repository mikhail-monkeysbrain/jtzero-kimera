#!/usr/bin/env python3
import argparse, csv, math, statistics
from pathlib import Path
import cv2
import numpy as np

def load_csv(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def I(x): return int(x)

def read_frame(mj, rec):
    mj.seek(int(rec["offset"]))
    b = mj.read(int(rec["bytes"]))
    if len(b) != int(rec["bytes"]):
        raise RuntimeError("short MJPEG read")
    img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("JPEG decode failed")
    return img

def analyze(root, max_corners=900):
    ev = load_csv(root/"events.csv")
    fr = load_csv(root/"selected_frames.csv")
    s = next(r for r in ev if r["event"]=="MOVE_START")
    e = next(r for r in ev if r["event"]=="MOVE_END")
    t0 = I(s["state_timestamp_ns"]); t1 = I(e["state_timestamp_ns"])
    seq = [r for r in fr if t0 <= I(r["timestamp_ns"]) <= t1]
    if len(seq) < 3:
        raise RuntimeError(f"{root}: too few MOVE frames")

    with (root/"selected.mjpg").open("rb") as mj:
        prev = read_frame(mj, seq[0])
        h, w = prev.shape[:2]
        center = np.array([w*0.5, h*0.5, 1.0], dtype=np.float64)

        H_total = np.eye(3, dtype=np.float64)
        pair_dx=[]; pair_dy=[]; inliers=[]; residuals=[]
        good_pairs=0; bad_pairs=0

        for rec in seq[1:]:
            cur = read_frame(mj, rec)

            p0 = cv2.goodFeaturesToTrack(
                prev, maxCorners=max_corners, qualityLevel=0.01,
                minDistance=7, blockSize=7
            )
            if p0 is None or len(p0) < 30:
                bad_pairs += 1
                prev = cur
                continue

            p1, st, err = cv2.calcOpticalFlowPyrLK(
                prev, cur, p0, None,
                winSize=(21,21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, 0.01)
            )
            ok = st.reshape(-1).astype(bool)
            a = p0.reshape(-1,2)[ok]
            b = p1.reshape(-1,2)[ok]

            if len(a) < 20:
                bad_pairs += 1
                prev = cur
                continue

            H, mask = cv2.findHomography(a, b, cv2.RANSAC, 2.0)
            if H is None or mask is None:
                bad_pairs += 1
                prev = cur
                continue

            mask = mask.reshape(-1).astype(bool)
            nin = int(mask.sum())
            if nin < 15:
                bad_pairs += 1
                prev = cur
                continue

            pa = a[mask]
            pb = b[mask]
            pa_h = np.c_[pa, np.ones(len(pa))]
            pred_h = (H @ pa_h.T).T
            pred = pred_h[:,:2] / pred_h[:,2:3]
            res = np.linalg.norm(pred-pb, axis=1)

            c2 = H @ center
            c2 = c2[:2] / c2[2]
            d = c2 - center[:2]
            pair_dx.append(float(d[0]))
            pair_dy.append(float(d[1]))
            inliers.append(nin)
            residuals.append(float(np.median(res)))

            H_total = H @ H_total
            good_pairs += 1
            prev = cur

        end_c = H_total @ center
        end_c = end_c[:2] / end_c[2]
        net = end_c - center[:2]
        pair_path = sum(math.hypot(x,y) for x,y in zip(pair_dx,pair_dy))

    return {
        "name": root.name,
        "frames": len(seq),
        "good": good_pairs,
        "bad": bad_pairs,
        "net_x": float(net[0]),
        "net_y": float(net[1]),
        "net": float(math.hypot(net[0],net[1])),
        "path": float(pair_path),
        "inliers_med": statistics.median(inliers) if inliers else float("nan"),
        "res_med": statistics.median(residuals) if residuals else float("nan"),
        "pair_dx_med": statistics.median(pair_dx) if pair_dx else float("nan"),
        "pair_dy_med": statistics.median(pair_dy) if pair_dy else float("nan"),
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    a=ap.parse_args()
    out=[analyze(Path(x)) for x in a.runs]

    print("="*132)
    print("CLEAN-01 — RAW IMAGE PLANAR MOTION COMPARISON (NO KIMERA / NO IMU / NO METRIC SCALE)")
    print("="*132)
    print(f"{'RUN':42s} {'frames':>6s} {'pairs':>9s} {'net px':>10s} {'x px':>10s} {'y px':>10s} {'path px':>10s} {'inl med':>9s} {'res px':>8s}")
    print("-"*132)
    for r in out:
        print(f"{r['name']:42s} {r['frames']:6d} {r['good']:4d}/{r['good']+r['bad']:<4d} "
              f"{r['net']:10.1f} {r['net_x']:+10.1f} {r['net_y']:+10.1f} {r['path']:10.1f} "
              f"{r['inliers_med']:9.1f} {r['res_med']:8.3f}")

    print()
    vals=[r["net"] for r in out]
    if len(vals)>=2:
        med=statistics.median(vals)
        print(f"net-pixel median={med:.1f}px  min={min(vals):.1f}px  max={max(vals):.1f}px  range={max(vals)-min(vals):.1f}px")
    print()
    print("INTERPRETATION")
    print("-"*132)
    print("This is a relative visual-input check only. Pixel displacement is NOT converted to millimetres.")
    print("If BAD raw-image motion is comparable to GOOD while VIO differs catastrophically, the run-to-run failure is inside state estimation.")
    print("If BAD raw-image motion is substantially different, the three physical/image inputs were not equivalent enough for a pure VIO A/B.")
    print("="*132)

if __name__=="__main__":
    main()
