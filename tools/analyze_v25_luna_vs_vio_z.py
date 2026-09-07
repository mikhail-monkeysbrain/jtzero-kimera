#!/usr/bin/env python3
import csv
import math
import statistics
import sys
from pathlib import Path

def load_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

def f(row, key):
    return float(row[key])

def i(row, key):
    return int(float(row[key]))

def median(xs):
    return statistics.median(xs) if xs else float("nan")

def mean(xs):
    return statistics.fmean(xs) if xs else float("nan")

def std(xs):
    if not xs:
        return float("nan")
    m = mean(xs)
    return math.sqrt(sum((x-m)*(x-m) for x in xs)/len(xs))

if len(sys.argv) != 2:
    raise SystemExit("usage: analyze_v25_luna_vs_vio_z.py RUN_DIR")

root = Path(sys.argv[1])
events = load_csv(root / "jtzero_500mm_v25_events.csv")
legs = load_csv(root / "jtzero_500mm_v25_legs.csv")
ranges = load_csv(root / "jtzero_500mm_v25_range.csv")

ev = {}
for r in events:
    ev[(i(r, "leg"), r["event"])] = r

# Median around operator START/END. These clocks are both RPi monotonic wall time.
WINDOW_NS = int(0.75e9)

def near(t_ns):
    return [r for r in ranges if abs(i(r, "recv_ns") - t_ns) <= WINDOW_NS]

def between(t0_ns, t1_ns):
    return [r for r in ranges if t0_ns <= i(r, "recv_ns") <= t1_ns]

print("================ V25 TF-LUNA vs VIO Z ================")
print("run:", root)
print("Assumption for this bench test: vehicle stays on its legs; true camera height above table is constant.")
print("Endpoint Luna values: median in +/-0.75 s around operator START/END.")
print()

rows = []
for L in legs:
    leg = i(L, "leg")
    start = ev.get((leg, "START"))
    end = ev.get((leg, "END"))
    if not start or not end:
        print(f"LEG {leg}: missing START/END")
        continue

    t0 = i(start, "event_wall_ns")
    t1 = i(end, "event_wall_ns")
    s = near(t0)
    e = near(t1)
    m = between(t0, t1)

    s_cm = [f(r, "current_cm") for r in s]
    e_cm = [f(r, "current_cm") for r in e]
    m_cm = [f(r, "current_cm") for r in m]

    s_v = [f(r, "vertical_m") for r in s]
    e_v = [f(r, "vertical_m") for r in e]
    m_v = [f(r, "vertical_m") for r in m]

    luna_start_m = median(s_cm) * 0.01
    luna_end_m = median(e_cm) * 0.01
    luna_delta_m = luna_end_m - luna_start_m

    vert_start_m = median(s_v)
    vert_end_m = median(e_v)
    vert_delta_m = vert_end_m - vert_start_m

    vio_dz = f(L, "dz_m")
    scale = f(L, "scale_horizontal")

    motion_span_cm = (max(m_cm) - min(m_cm)) if m_cm else float("nan")
    motion_std_cm = std(m_cm)
    signal = [f(r, "signal_quality") for r in m if r.get("signal_quality") not in ("", None)]
    signal_med = median(signal)

    rows.append({
        "leg": leg, "direction": L["direction"], "scale": scale, "vio_dz": vio_dz,
        "luna_delta": luna_delta_m, "vert_delta": vert_delta_m,
        "span_cm": motion_span_cm, "std_cm": motion_std_cm, "signal": signal_med,
        "n": len(m)
    })

    print(f"LEG {leg} {L['direction']}: scale={scale:.4f}")
    print(f"  VIO dz                 = {vio_dz*1000:+.2f} mm")
    print(f"  TF-Luna raw START/END  = {luna_start_m*1000:.1f} / {luna_end_m*1000:.1f} mm")
    print(f"  TF-Luna raw delta      = {luna_delta_m*1000:+.2f} mm")
    print(f"  TF-Luna vertical delta = {vert_delta_m*1000:+.2f} mm")
    print(f"  TF-Luna during leg     = n={len(m)} span={motion_span_cm:.2f} cm std={motion_std_cm:.3f} cm signal_med={signal_med:.1f}")
    print(f"  VIO-Luna dz mismatch   = {(vio_dz-vert_delta_m)*1000:+.2f} mm")
    print()

print("================ DIRECTION SUMMARY ================")
for direction in ("A->B", "B->A"):
    rr = [r for r in rows if r["direction"] == direction]
    if not rr:
        continue
    print(
        f"{direction}: "
        f"mean VIO dz={mean([r['vio_dz'] for r in rr])*1000:+.2f} mm, "
        f"mean Luna vertical delta={mean([r['vert_delta'] for r in rr])*1000:+.2f} mm, "
        f"mean mismatch={mean([r['vio_dz']-r['vert_delta'] for r in rr])*1000:+.2f} mm"
    )

print()
print("INTERPRETATION:")
print("- If TF-Luna delta stays near zero while VIO dz is tens/hundreds of mm, the vertical motion is estimator-created.")
print("- If the VIO dz sign repeats with A->B/B->A while Luna stays near zero, translation-to-Z coupling is strongly supported.")
print("- Do not tune T_BS from this test alone; first establish the repeatable Luna-vs-VIO mismatch.")
