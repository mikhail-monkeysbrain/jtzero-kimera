#!/usr/bin/env python3
import csv
import math
import os
import sys
from collections import defaultdict

HOME = "/home/vio"
MANIFEST = os.path.join(HOME, "jtzero_tbs_cross_validation_manifest.csv")
DETAIL = os.path.join(HOME, "jtzero_tbs_cross_validation_detail.csv")
SUMMARY = os.path.join(HOME, "jtzero_tbs_cross_validation_summary.csv")
TOP = os.path.join(HOME, "jtzero_tbs_cross_validation_top.txt")


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
    if not out:
        raise RuntimeError(f"no legs in {path}")
    return out


def load_states(path):
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out.append((
                int(r["timestamp_ns"]),
                float(r["px_m"]), float(r["py_m"]), float(r["pz_m"]),
            ))
    if not out:
        raise RuntimeError(f"empty states: {path}")
    return out


def nearest(states, t):
    return min(states, key=lambda q: abs(q[0] - t))


def metrics(states, legs):
    vecs = []
    z2 = xyerr2 = xysum = 0.0
    for w in legs:
        a = nearest(states, w["start_ns"])
        b = nearest(states, w["end_ns"])
        dx, dy, dz = b[1]-a[1], b[2]-a[2], b[3]-a[3]
        xy = math.hypot(dx, dy)
        vecs.append((w["leg"], dx, dy, dz, xy))
        z2 += dz*dz
        xyerr2 += (xy-0.500)**2
        xysum += xy

    n = len(vecs)
    z_rms = math.sqrt(z2/n)*1000.0
    xy_rms = math.sqrt(xyerr2/n)*1000.0
    xy_mean = xysum/n*1000.0

    by_leg = {v[0]: v for v in vecs}
    pair2 = pairxy2 = 0.0
    npair = 0
    for la, lb in ((1,2),(3,4),(5,6)):
        if la not in by_leg or lb not in by_leg:
            continue
        a, b = by_leg[la], by_leg[lb]
        sx, sy, sz = a[1]+b[1], a[2]+b[2], a[3]+b[3]
        pair2 += sx*sx + sy*sy + sz*sz
        pairxy2 += sx*sx + sy*sy
        npair += 1
    pair_rms = math.sqrt(pair2/npair)*1000.0 if npair else float("nan")
    pair_xy = math.sqrt(pairxy2/npair)*1000.0 if npair else float("nan")

    p0, pn = states[0], states[-1]
    final_dp = math.sqrt((pn[1]-p0[1])**2 + (pn[2]-p0[2])**2 + (pn[3]-p0[3])**2)*1000.0
    score = z_rms + xy_rms + pair_rms
    return {
        "score_mm": score,
        "z_rms_mm": z_rms,
        "xy_err_rms_mm": xy_rms,
        "pair_closure_rms_mm": pair_rms,
        "pair_xy_closure_rms_mm": pair_xy,
        "xy_mean_mm": xy_mean,
        "final_dp_mm": final_dp,
    }


def main():
    if not os.path.exists(MANIFEST):
        raise SystemExit(f"missing: {MANIFEST}")

    rows = []
    with open(MANIFEST, newline="") as f:
        manifest = list(csv.DictReader(f))

    leg_cache = {}
    for m in manifest:
        ds = m["dataset"]
        backend = m["backend"]
        legs_path = m["legs"]
        key = (backend, legs_path)
        if key not in leg_cache:
            leg_cache[key] = load_leg_windows(legs_path, load_kf_times(backend))
        states_path = os.path.join(HOME, f'jtzero_extrinsics_replay_v10_{m["tag"]}.csv')
        if not os.path.exists(states_path):
            print(f"WARN missing states: {states_path}", file=sys.stderr)
            continue
        mm = metrics(load_states(states_path), leg_cache[key])
        rows.append({
            "dataset": ds,
            "candidate": m["candidate"],
            "tag": m["tag"],
            "roll_deg": float(m["roll_deg"]),
            "pitch_deg": float(m["pitch_deg"]),
            **mm,
        })

    if not rows:
        raise SystemExit("no cross-validation results")

    detail_fields = [
        "dataset","candidate","tag","roll_deg","pitch_deg","score_mm",
        "z_rms_mm","xy_err_rms_mm","pair_closure_rms_mm",
        "pair_xy_closure_rms_mm","xy_mean_mm","final_dp_mm",
    ]
    with open(DETAIL, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=detail_fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k,v in r.items()})

    grouped = defaultdict(list)
    for r in rows:
        grouped[r["candidate"]].append(r)

    agg = []
    for cand, rr in grouped.items():
        if not rr:
            continue
        agg.append({
            "candidate": cand,
            "roll_deg": rr[0]["roll_deg"],
            "pitch_deg": rr[0]["pitch_deg"],
            "datasets": len(rr),
            "mean_score_mm": sum(x["score_mm"] for x in rr)/len(rr),
            "worst_score_mm": max(x["score_mm"] for x in rr),
            "mean_z_rms_mm": sum(x["z_rms_mm"] for x in rr)/len(rr),
            "worst_z_rms_mm": max(x["z_rms_mm"] for x in rr),
            "mean_xy_rms_mm": sum(x["xy_err_rms_mm"] for x in rr)/len(rr),
            "worst_xy_rms_mm": max(x["xy_err_rms_mm"] for x in rr),
            "mean_pair_closure_mm": sum(x["pair_closure_rms_mm"] for x in rr)/len(rr),
            "worst_pair_closure_mm": max(x["pair_closure_rms_mm"] for x in rr),
            "mean_final_dp_mm": sum(x["final_dp_mm"] for x in rr)/len(rr),
            "worst_final_dp_mm": max(x["final_dp_mm"] for x in rr),
        })

    # Robust ranking: optimize the worst dataset first, then the mean.
    agg.sort(key=lambda r: (r["worst_score_mm"], r["mean_score_mm"]))
    fields = [
        "rank","candidate","roll_deg","pitch_deg","datasets",
        "mean_score_mm","worst_score_mm","mean_z_rms_mm","worst_z_rms_mm",
        "mean_xy_rms_mm","worst_xy_rms_mm","mean_pair_closure_mm",
        "worst_pair_closure_mm","mean_final_dp_mm","worst_final_dp_mm",
    ]
    with open(SUMMARY, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i,r in enumerate(agg,1):
            row = {"rank": i, **r}
            w.writerow({k: (f"{v:.3f}" if isinstance(v,float) else v) for k,v in row.items()})

    lines = []
    lines.append("================ TBS CROSS-DATASET VALIDATION ================")
    lines.append("rank candidate       roll pitch  worst_score mean_score worst_Z worst_XY worst_pair worst_final")
    for i,r in enumerate(agg,1):
        lines.append(
            f'{i:>2}  {r["candidate"]:<14} {r["roll_deg"]:>5.1f} {r["pitch_deg"]:>5.1f} '
            f'{r["worst_score_mm"]:>11.2f} {r["mean_score_mm"]:>10.2f} '
            f'{r["worst_z_rms_mm"]:>7.2f} {r["worst_xy_rms_mm"]:>8.2f} '
            f'{r["worst_pair_closure_mm"]:>10.2f} {r["worst_final_dp_mm"]:>11.2f}'
        )
    lines.append("")
    lines.append("Per-dataset detail:")
    lines.append("dataset candidate       score    Z_RMS   XY_RMS  pair_cl  XY_mean final_dP")
    for r in sorted(rows, key=lambda x:(x["candidate"],x["dataset"])):
        lines.append(
            f'{r["dataset"]:<7} {r["candidate"]:<14} {r["score_mm"]:>8.2f} '
            f'{r["z_rms_mm"]:>8.2f} {r["xy_err_rms_mm"]:>8.2f} '
            f'{r["pair_closure_rms_mm"]:>8.2f} {r["xy_mean_mm"]:>8.2f} {r["final_dp_mm"]:>8.2f}'
        )
    text = "\n".join(lines) + "\n"
    with open(TOP, "w") as f:
        f.write(text)
    print(text, end="")
    print(f"Saved: {DETAIL}")
    print(f"Saved: {SUMMARY}")
    print(f"Saved: {TOP}")


if __name__ == "__main__":
    main()
