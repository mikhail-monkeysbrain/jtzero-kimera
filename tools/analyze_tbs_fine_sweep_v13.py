#!/usr/bin/env python3
import csv
import math
import os
import sys

HOME = "/home/vio"
MANIFEST = os.path.join(HOME, "jtzero_tbs_fine_v13_manifest.csv")
BACKEND = os.path.join(HOME, "jtzero_500mm_v13_backend.csv")
LEGS = os.path.join(HOME, "jtzero_500mm_v13_legs.csv")
OUT = os.path.join(HOME, "jtzero_tbs_fine_v13_ranked.csv")

def load_kf_times(path):
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out[int(r["keyframe"])] = int(r["timestamp_ns"])
    return out

def load_leg_windows(path, kf_times):
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            leg = int(r["leg"])
            skf = int(r["start_settled_kf"])
            ekf = int(r["end_settled_kf"])
            if skf not in kf_times or ekf not in kf_times:
                raise RuntimeError(f"missing backend timestamp for leg {leg}: {skf}->{ekf}")
            out.append({
                "leg": leg,
                "direction": r["direction"],
                "start_ns": kf_times[skf],
                "end_ns": kf_times[ekf],
            })
    return out

def load_states(path):
    s = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            s.append((
                int(r["timestamp_ns"]),
                float(r["px_m"]),
                float(r["py_m"]),
                float(r["pz_m"]),
            ))
    if not s:
        raise RuntimeError(f"empty states: {path}")
    return s

def nearest(states, t):
    return min(states, key=lambda q: abs(q[0] - t))

def metrics(states, legs):
    vecs = []
    z2 = 0.0
    xyerr2 = 0.0
    xy_sum = 0.0
    for w in legs:
        a = nearest(states, w["start_ns"])
        b = nearest(states, w["end_ns"])
        dx, dy, dz = b[1]-a[1], b[2]-a[2], b[3]-a[3]
        xy = math.hypot(dx, dy)
        vecs.append((w["leg"], dx, dy, dz, xy))
        z2 += dz*dz
        xyerr2 += (xy-0.500)**2
        xy_sum += xy

    n = len(vecs)
    z_rms = math.sqrt(z2/n)*1000.0
    xy_rms = math.sqrt(xyerr2/n)*1000.0
    xy_mean = xy_sum/n*1000.0

    pair2 = 0.0
    pair_xy2 = 0.0
    pair_count = 0
    by_leg = {v[0]: v for v in vecs}
    for a_leg, b_leg in ((1,2),(3,4),(5,6)):
        if a_leg not in by_leg or b_leg not in by_leg:
            continue
        a, b = by_leg[a_leg], by_leg[b_leg]
        sx, sy, sz = a[1]+b[1], a[2]+b[2], a[3]+b[3]
        pair2 += sx*sx + sy*sy + sz*sz
        pair_xy2 += sx*sx + sy*sy
        pair_count += 1
    pair_rms = math.sqrt(pair2/pair_count)*1000.0 if pair_count else float("nan")
    pair_xy_rms = math.sqrt(pair_xy2/pair_count)*1000.0 if pair_count else float("nan")

    p0, pn = states[0], states[-1]
    final_dp = math.sqrt((pn[1]-p0[1])**2 + (pn[2]-p0[2])**2 + (pn[3]-p0[3])**2)*1000.0

    score = z_rms + xy_rms + pair_rms
    return {
        "z_rms_mm": z_rms,
        "xy_err_rms_mm": xy_rms,
        "xy_mean_mm": xy_mean,
        "pair_closure_rms_mm": pair_rms,
        "pair_xy_closure_rms_mm": pair_xy_rms,
        "final_dp_mm": final_dp,
        "score_mm": score,
    }

def main():
    for p in (MANIFEST, BACKEND, LEGS):
        if not os.path.exists(p):
            raise SystemExit(f"missing: {p}")

    kf_times = load_kf_times(BACKEND)
    legs = load_leg_windows(LEGS, kf_times)

    rows = []
    with open(MANIFEST, newline="") as f:
        for m in csv.DictReader(f):
            tag = m["tag"]
            path = os.path.join(HOME, f"jtzero_extrinsics_replay_v10_{tag}.csv")
            if not os.path.exists(path):
                print(f"WARN missing states: {path}", file=sys.stderr)
                continue
            try:
                mm = metrics(load_states(path), legs)
            except Exception as e:
                print(f"WARN {tag}: {e}", file=sys.stderr)
                continue
            rows.append({
                "tag": tag,
                "roll_deg": float(m["roll_deg"]),
                "pitch_deg": float(m["pitch_deg"]),
                **mm,
            })

    if not rows:
        raise SystemExit("no v13 fine-sweep results found")

    rows.sort(key=lambda r: r["score_mm"])
    fields = [
        "rank","tag","roll_deg","pitch_deg","score_mm",
        "z_rms_mm","xy_err_rms_mm","pair_closure_rms_mm",
        "pair_xy_closure_rms_mm","xy_mean_mm","final_dp_mm",
    ]
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i, r in enumerate(rows, 1):
            w.writerow({
                "rank": i,
                "tag": r["tag"],
                "roll_deg": f'{r["roll_deg"]:.1f}',
                "pitch_deg": f'{r["pitch_deg"]:.1f}',
                "score_mm": f'{r["score_mm"]:.3f}',
                "z_rms_mm": f'{r["z_rms_mm"]:.3f}',
                "xy_err_rms_mm": f'{r["xy_err_rms_mm"]:.3f}',
                "pair_closure_rms_mm": f'{r["pair_closure_rms_mm"]:.3f}',
                "pair_xy_closure_rms_mm": f'{r["pair_xy_closure_rms_mm"]:.3f}',
                "xy_mean_mm": f'{r["xy_mean_mm"]:.3f}',
                "final_dp_mm": f'{r["final_dp_mm"]:.3f}',
            })

    print("================ TBS V13 FINE SWEEP TOP 15 ================")
    print("rank  roll  pitch   score    Z_RMS   XY_RMS  pair_cl  XY_mean  final_dP")
    for i, r in enumerate(rows[:15], 1):
        print(f'{i:>2}  {r["roll_deg"]:>5.1f} {r["pitch_deg"]:>6.1f} '
              f'{r["score_mm"]:>8.2f} {r["z_rms_mm"]:>8.2f} '
              f'{r["xy_err_rms_mm"]:>8.2f} {r["pair_closure_rms_mm"]:>8.2f} '
              f'{r["xy_mean_mm"]:>8.2f} {r["final_dp_mm"]:>9.2f}')
    print(f"Saved: {OUT}")

if __name__ == "__main__":
    main()
