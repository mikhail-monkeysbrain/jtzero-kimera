#!/usr/bin/env python3
import bisect
import csv
import itertools
import math
import sys
from pathlib import Path


def load(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def I(row, key):
    return int(float(row[key]))


def F(row, key):
    return float(row[key])


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def rms(xs):
    return math.sqrt(mean([x * x for x in xs])) if xs else float("nan")


def pct(xs, p):
    if not xs:
        return float("nan")
    s = sorted(xs)
    x = (len(s) - 1) * p
    i = int(math.floor(x))
    j = int(math.ceil(x))
    return s[i] if i == j else s[i] * (j - x) + s[j] * (x - i)


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def unwrap_deg(xs):
    if not xs:
        return []
    out = [xs[0]]
    for x in xs[1:]:
        prev_raw = out[-1]
        y = x
        while y - prev_raw > 180.0:
            y -= 360.0
        while y - prev_raw < -180.0:
            y += 360.0
        out.append(y)
    return out


def angular_span_deg(xs):
    u = unwrap_deg(xs)
    return max(u) - min(u) if u else float("nan")


def RzRyRx(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def mv(R, v):
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def rel_delta(a, b, floor):
    den = max((abs(a) + abs(b)) * 0.5, floor)
    return abs(a - b) / den


def load_run(root):
    root = Path(root)
    imu = [r for r in load(root / "jtzero_500mm_v25.csv") if r.get("type") == "IMU"]
    att = load(root / "jtzero_500mm_v25_attitude.csv")
    events = load(root / "jtzero_500mm_v25_events.csv")
    legs = load(root / "jtzero_500mm_v25_legs.csv")

    imu.sort(key=lambda r: I(r, "recv_ns"))
    att.sort(key=lambda r: I(r, "recv_ns"))
    atts = [I(r, "recv_ns") for r in att]

    def nearest_att(t):
        j = bisect.bisect_left(atts, t)
        cand = []
        for k in (j - 1, j, j + 1):
            if 0 <= k < len(att):
                cand.append(att[k])
        return min(cand, key=lambda r: abs(I(r, "recv_ns") - t)) if cand else None

    ev = {(I(r, "leg"), r["event"]): r for r in events}
    rows = []

    for L in legs:
        leg = I(L, "leg")
        es = ev.get((leg, "START"))
        ee = ev.get((leg, "END"))
        if not es or not ee:
            continue

        t0 = I(es, "event_wall_ns")
        t1 = I(ee, "event_wall_ns")
        seg = [r for r in imu if t0 <= I(r, "recv_ns") <= t1]
        if not seg:
            continue

        horiz = []
        totaldev = []
        gnorm = []
        roll = []
        pitch = []
        yaw = []
        match = []

        for q in seg:
            a = nearest_att(I(q, "recv_ns"))
            if not a:
                continue

            # HIGHRES_IMU is logged in FC FRD. Convert specific force to FLU,
            # then use FC attitude only to express dynamic acceleration in world axes.
            af = (F(q, "ax"), -F(q, "ay"), -F(q, "az"))
            rr = math.radians(F(a, "roll_deg"))
            pp = math.radians(-F(a, "pitch_deg"))
            yy = math.radians(-F(a, "yaw_deg"))
            aw = mv(RzRyRx(rr, pp, yy), af)
            horiz.append(math.hypot(aw[0], aw[1]))
            totaldev.append(norm((aw[0], aw[1], aw[2] - 9.81)))

            roll.append(F(a, "roll_deg"))
            pitch.append(F(a, "pitch_deg"))
            yaw.append(F(a, "yaw_deg"))
            match.append(abs(I(a, "recv_ns") - I(q, "recv_ns")) / 1e6)

            kg = None
            for keys in (("gx", "gy", "gz"), ("xgyro", "ygyro", "zgyro")):
                if all(k in q and q[k] not in ("", None) for k in keys):
                    kg = keys
                    break
            if kg:
                gnorm.append(norm((F(q, kg[0]), F(q, kg[1]), F(q, kg[2]))))

        if not horiz or not roll:
            continue

        xy = F(L, "horizontal_m") * 1000.0
        rows.append(
            dict(
                run=root.name,
                path=str(root),
                leg=leg,
                direction=L["direction"],
                scale=xy / 500.0,
                duration_s=(t1 - t0) * 1e-9,
                samples=len(horiz),
                horiz_acc_rms=rms(horiz),
                horiz_acc_p90=pct(horiz, 0.9),
                total_dyn_acc_rms=rms(totaldev),
                gyro_rms=rms(gnorm) if gnorm else float("nan"),
                gyro_p90=pct(gnorm, 0.9) if gnorm else float("nan"),
                roll_span=angular_span_deg(roll),
                pitch_span=angular_span_deg(pitch),
                yaw_span=angular_span_deg(yaw),
                att_match_mean_ms=mean(match),
            )
        )

    return rows


def pair_distance(a, b):
    parts = [
        ("duration", rel_delta(a["duration_s"], b["duration_s"], 1.0)),
        ("hAccRMS", rel_delta(a["horiz_acc_rms"], b["horiz_acc_rms"], 0.05)),
        ("hAccP90", rel_delta(a["horiz_acc_p90"], b["horiz_acc_p90"], 0.05)),
        ("dynRMS", rel_delta(a["total_dyn_acc_rms"], b["total_dyn_acc_rms"], 0.05)),
    ]

    if math.isfinite(a["gyro_rms"]) and math.isfinite(b["gyro_rms"]):
        parts += [
            ("gyroRMS", rel_delta(a["gyro_rms"], b["gyro_rms"], 0.005)),
            ("gyroP90", rel_delta(a["gyro_p90"], b["gyro_p90"], 0.005)),
        ]

    # Attitude spans are independent FC observables but can be near zero,
    # so use fixed degree scales rather than unstable relative ratios.
    parts += [
        ("rollSpan", abs(a["roll_span"] - b["roll_span"]) / 1.0),
        ("pitchSpan", abs(a["pitch_span"] - b["pitch_span"]) / 1.0),
        ("yawSpan", abs(a["yaw_span"] - b["yaw_span"]) / 1.0),
    ]
    return mean([v for _, v in parts]), parts


def tag(r):
    return f'{r["run"]}:L{r["leg"]}:{r["direction"]}'


def canonical_ba_minus_ab(a, b):
    if a["direction"] == b["direction"]:
        return float("nan")
    ba = a if a["direction"] == "B->A" else b
    ab = a if a["direction"] == "A->B" else b
    return ba["scale"] - ab["scale"]


if len(sys.argv) < 3:
    raise SystemExit(
        "usage: analyze_v25_raw_profile_crossrun_match.py <run_dir1> <run_dir2> [run_dir3 ...]"
    )

rows = []
for p in sys.argv[1:]:
    rr = load_run(p)
    if not rr:
        print(f"WARNING: no usable legs in {p}", file=sys.stderr)
    rows.extend(rr)

print("================ V25 RAW-PROFILE CROSS-RUN MATCH ================")
print(f"runs={len(sys.argv)-1} legs={len(rows)}")
print("INPUT FEATURES: operator duration + raw FC accel/gyro + unwrapped FC attitude spans")
print("NOT USED FOR MATCHING: backend velocity, backend bias, VIO scale")

for r in rows:
    print(
        f'{tag(r):52s} scale={r["scale"]:.4f} dur={r["duration_s"]:.2f}s '
        f'hAcc={r["horiz_acc_rms"]:.4f}/{r["horiz_acc_p90"]:.4f} '
        f'dyn={r["total_dyn_acc_rms"]:.4f} '
        f'gyro={r["gyro_rms"]:.5f}/{r["gyro_p90"]:.5f} '
        f'R/P/Yspan={r["roll_span"]:.3f}/{r["pitch_span"]:.3f}/{r["yaw_span"]:.3f}'
    )

pairs = []
for i in range(len(rows)):
    for j in range(i + 1, len(rows)):
        a, b = rows[i], rows[j]
        if a["run"] == b["run"]:
            continue
        d, parts = pair_distance(a, b)
        pairs.append((d, a, b, parts))

opp = [x for x in pairs if x[1]["direction"] != x[2]["direction"]]
same = [x for x in pairs if x[1]["direction"] == x[2]["direction"]]

print("\n================ NEAREST OPPOSITE-DIRECTION PAIRS ================")
for rank, (d, a, b, parts) in enumerate(sorted(opp, key=lambda x: x[0])[:12], 1):
    print(
        f'#{rank:02d} score={d:.3f}  {tag(a)}  <->  {tag(b)}  '
        f'BA-AB={canonical_ba_minus_ab(a,b):+.4f}'
    )
    print("     " + " ".join(f"{k}={v:.3f}" for k, v in parts))

print("\n================ GLOBAL ONE-TO-ONE OPPOSITE-DIRECTION MATCH ================")
ab = [r for r in rows if r["direction"] == "A->B"]
ba = [r for r in rows if r["direction"] == "B->A"]
if len(ab) == len(ba) and ab:
    cost = {}
    detail = {}
    for i, a in enumerate(ab):
        for j, b in enumerate(ba):
            if a["run"] == b["run"]:
                cost[(i, j)] = float("inf")
                detail[(i, j)] = None
            else:
                d, parts = pair_distance(a, b)
                cost[(i, j)] = d
                detail[(i, j)] = parts

    best = None
    for perm in itertools.permutations(range(len(ba))):
        vals = [cost[(i, perm[i])] for i in range(len(ab))]
        if any(not math.isfinite(v) for v in vals):
            continue
        total = sum(vals)
        if best is None or total < best[0]:
            best = (total, perm, vals)

    if best is None:
        print("No complete cross-run one-to-one matching exists.")
    else:
        total, perm, vals = best
        deltas = []
        for rank, (i, j, d) in enumerate(zip(range(len(ab)), perm, vals), 1):
            a, b = ab[i], ba[j]
            delta = b["scale"] - a["scale"]
            deltas.append(delta)
            print(
                f'#{rank:02d} score={d:.3f}  {tag(a)}  <->  {tag(b)}  BA-AB={delta:+.4f}'
            )
        sd = sorted(deltas)
        med = (sd[len(sd)//2] if len(sd)%2 else 0.5*(sd[len(sd)//2-1]+sd[len(sd)//2]))
        print(
            f"one-to-one mean score={total/len(ab):.3f} "
            f"mean BA-AB={mean(deltas):+.4f} median BA-AB={med:+.4f} "
            f"positive={sum(x>0 for x in deltas)}/{len(deltas)}"
        )
        print("NOTE: one-to-one removes leg reuse but does NOT make legs from the same physical run statistically independent.")
else:
    print(f"Cannot form balanced one-to-one match: A->B={len(ab)} B->A={len(ba)}")

print("\n================ NEAREST SAME-DIRECTION PAIRS ================")
for rank, (d, a, b, parts) in enumerate(sorted(same, key=lambda x: x[0])[:8], 1):
    print(
        f'#{rank:02d} score={d:.3f}  {tag(a)}  <->  {tag(b)}  '
        f'scale_delta={b["scale"]-a["scale"]:+.4f}'
    )

print("\nINTERPRETATION:")
print("- A low score means two legs are similar only in the recorded independent raw-input descriptors.")
print("- The score is a ranking metric, not a statistical proof that the physical trajectories were identical.")
print("- If the nearest opposite-direction pairs have small raw-profile mismatch but retain a large systematic scale delta,")
print("  pure motion-profile confounding weakens and a direction/visual-estimator asymmetry becomes more plausible.")
print("- If scale delta shrinks among the best-matched opposite-direction pairs, raw motion excitation remains a strong confounder.")
print("- Attitude spans are computed after 360-deg unwrapping; Euler wrap must not inflate the match score.")
print("- Same-direction low-score pairs are a run-to-run repeatability control.")
print("- Do not convert n_legs into n_independent_runs; causal confidence is limited by the number of physical runs.")
