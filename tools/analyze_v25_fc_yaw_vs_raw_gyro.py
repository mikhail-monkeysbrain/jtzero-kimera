#!/usr/bin/env python3
import csv
import math
import statistics
import sys
from pathlib import Path


def load(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def I(r, k):
    return int(float(r[k]))


def F(r, k):
    return float(r[k])


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def mean(xs):
    return statistics.mean(xs) if xs else float("nan")


def unwrap_deg(xs):
    if not xs:
        return []
    out = [xs[0]]
    for x in xs[1:]:
        y = x
        while y - out[-1] > 180.0:
            y -= 360.0
        while y - out[-1] < -180.0:
            y += 360.0
        out.append(y)
    return out


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def qnorm(q):
    n = math.sqrt(sum(x*x for x in q))
    return tuple(x/n for x in q)


def qexp(wdt):
    a = norm(wdt)
    if a < 1e-15:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(a/2.0) / a
    return (math.cos(a/2.0), wdt[0]*s, wdt[1]*s, wdt[2]*s)


def q_to_rotvec_deg(q):
    q = qnorm(q)
    if q[0] < 0:
        q = tuple(-x for x in q)
    w = max(-1.0, min(1.0, q[0]))
    a = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w*w))
    if s < 1e-12:
        rv = (0.0, 0.0, 0.0)
    else:
        axis = (q[1]/s, q[2]/s, q[3]/s)
        rv = tuple(axis[i]*a for i in range(3))
    return tuple(math.degrees(x) for x in rv)


def gyro_keys(r):
    for keys in (("gx","gy","gz"), ("xgyro","ygyro","zgyro")):
        if all(k in r and r[k] not in ("", None) for k in keys):
            return keys
    return None


if len(sys.argv) < 2:
    raise SystemExit("usage: analyze_v25_fc_yaw_vs_raw_gyro.py RUN_DIR [RUN_DIR ...]")

print("================ V25 FC YAW vs RAW GYRO ================")
print("Purpose: classify large FC yaw excursions as physical angular motion vs FC/reference behavior.")
print("Raw HIGHRES_IMU gyro is integrated only over operator START->END.")
print("No backend/VIO state is used to decide whether the physical angular input existed.")

for arg in sys.argv[1:]:
    root = Path(arg)
    imu = [r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type") == "IMU"]
    att = load(root/"jtzero_500mm_v25_attitude.csv")
    events = load(root/"jtzero_500mm_v25_events.csv")
    legs = load(root/"jtzero_500mm_v25_legs.csv")

    imu.sort(key=lambda r: I(r, "recv_ns"))
    att.sort(key=lambda r: I(r, "recv_ns"))
    ev = {(I(r,"leg"), r["event"]): r for r in events}

    print("\nRUN:", root)

    for L in legs:
        leg = I(L, "leg")
        es = ev.get((leg, "START"))
        ee = ev.get((leg, "END"))
        if not es or not ee:
            print(f"LEG {leg}: missing START/END")
            continue
        t0 = I(es, "event_wall_ns")
        t1 = I(ee, "event_wall_ns")

        aa = [r for r in att if t0 <= I(r, "recv_ns") <= t1]
        ii = [r for r in imu if t0 <= I(r, "recv_ns") <= t1]
        if len(aa) < 2 or len(ii) < 3:
            print(f"LEG {leg}: insufficient ATT/IMU samples")
            continue

        y = unwrap_deg([F(r, "yaw_deg") for r in aa])
        dy = y[-1] - y[0]
        span = max(y) - min(y)
        path = sum(abs(y[i]-y[i-1]) for i in range(1, len(y)))
        steps = [abs(y[i]-y[i-1]) for i in range(1, len(y))]
        max_step = max(steps) if steps else 0.0
        max_step_i = steps.index(max_step)+1 if steps else 0
        max_step_dt_ms = 0.0
        if max_step_i:
            max_step_dt_ms = (I(aa[max_step_i],"recv_ns") - I(aa[max_step_i-1],"recv_ns")) / 1e6

        keys = gyro_keys(ii[0])
        if keys is None:
            for r in ii:
                keys = gyro_keys(r)
                if keys is not None:
                    break
        if keys is None:
            print(f"LEG {leg}: no gyro columns")
            continue

        # Estimate a conservative constant gyro bias from the quietest endpoint slices.
        n = max(5, len(ii)//10)
        edge = ii[:n] + ii[-n:]
        bias = tuple(mean([F(r,k) for r in edge]) for k in keys)

        q = (1.0, 0.0, 0.0, 0.0)
        int_gz = 0.0
        abs_gz = 0.0
        total_gyro_path = 0.0
        used = 0
        prev = None
        for r in ii:
            t = I(r, "recv_ns")
            if prev is not None:
                dt = (t - prev[0]) * 1e-9
                if 0.0 < dt <= 0.03:
                    w0 = tuple(F(prev[1], keys[j]) - bias[j] for j in range(3))
                    w1 = tuple(F(r, keys[j]) - bias[j] for j in range(3))
                    w = tuple(0.5*(w0[j]+w1[j]) for j in range(3))
                    q = qnorm(qmul(q, qexp(tuple(x*dt for x in w))))
                    int_gz += w[2]*dt
                    abs_gz += abs(w[2])*dt
                    total_gyro_path += norm(w)*dt
                    used += 1
            prev = (t, r)

        rv = q_to_rotvec_deg(q)
        net_total = norm(rv)
        int_gz_deg = math.degrees(int_gz)
        abs_gz_deg = math.degrees(abs_gz)
        gyro_path_deg = math.degrees(total_gyro_path)

        ratio = abs(dy)/max(net_total, 1e-9)
        print(f"LEG {leg} {L['direction']}: dur={(t1-t0)*1e-9:.2f}s scale={F(L,'scale_horizontal'):.4f}")
        print(f"  FC yaw: start={y[0]:+.2f} end={y[-1]:+.2f} dYaw={dy:+.2f} span={span:.2f} path={path:.2f} deg")
        print(f"  FC continuity: max_step={max_step:.2f} deg over {max_step_dt_ms:.1f} ms")
        print(f"  RAW gyro bias=[{bias[0]:+.6f},{bias[1]:+.6f},{bias[2]:+.6f}] rad/s")
        print(f"  RAW gyro net rotvec FRD=[{rv[0]:+.2f},{rv[1]:+.2f},{rv[2]:+.2f}] deg total={net_total:.2f} deg")
        print(f"  RAW gyro integral z={int_gz_deg:+.2f} deg |z|path={abs_gz_deg:.2f} deg total-path={gyro_path_deg:.2f} deg")
        print(f"  |FC dYaw| / gyro-net-total = {ratio:.2f}")

print("\nDECISION GUIDE:")
print("- FC yaw tens of degrees with raw gyro net/path of only a few degrees => not compatible with real rigid yaw of that size; inspect FC attitude/reference.")
print("- FC yaw delta comparable to gyro-integrated z/rotation and temporally continuous => real angular motion is plausible.")
print("- One large FC per-sample jump with no gyro counterpart => estimator/reference discontinuity is likely.")
print("- This analyzer classifies physical angular input only; it does not claim that yaw causes VIO scale error.")
