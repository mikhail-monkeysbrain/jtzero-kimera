#!/usr/bin/env python3

import csv
import math
import statistics
from pathlib import Path

CX = 0.014570
CY = 0.082383
G = 9.81

DATASETS = [
    ("V13",
     Path("/home/vio/jtzero_yaw_only_v13.csv"),
     Path("/home/vio/jtzero_yaw_only_v13_attitude.csv")),
    ("V15.42",
     Path("/home/vio/jtzero_yaw_only_v15_42.csv"),
     Path("/home/vio/jtzero_yaw_only_v15_42_attitude.csv")),
]

def mean(x):
    return sum(x) / len(x) if x else float("nan")

def sd(x):
    return statistics.stdev(x) if len(x) > 1 else 0.0

def corr(a, b):
    if len(a) < 3 or len(a) != len(b):
        return float("nan")
    ma, mb = mean(a), mean(b)
    aa = [x-ma for x in a]
    bb = [x-mb for x in b]
    den = math.sqrt(sum(x*x for x in aa) * sum(x*x for x in bb))
    return sum(x*y for x,y in zip(aa,bb))/den if den else float("nan")

def load_imu(path):
    out = []
    with path.open(newline="", errors="replace") as f:
        r = csv.reader(f)
        next(r, None)
        for c in r:
            if len(c) < 14 or c[0] != "IMU":
                continue
            try:
                ts = int(c[3])
                acc = (float(c[8]), -float(c[9]), -float(c[10]))
                g0 = (float(c[11]), -float(c[12]), -float(c[13]))
            except Exception:
                continue

            gyro = (
                g0[0] + CX*g0[2],
                g0[1] + CY*g0[2],
                g0[2]
            )
            out.append([ts, 0.0, acc, gyro])

    if not out:
        raise RuntimeError(f"No IMU rows: {path}")

    t0 = out[0][0]
    for x in out:
        x[1] = (x[0]-t0)*1e-9
    return out

def load_att(path):
    out = []
    with path.open(newline="", errors="replace") as f:
        r = csv.DictReader(f)
        t0 = None
        for c in r:
            try:
                ts = int(c["mapped_rpi_ns"])
                if t0 is None:
                    t0 = ts
                out.append((
                    (ts-t0)*1e-9,
                    float(c["rel_roll_deg"]),
                    float(c["rel_pitch_deg"]),
                    float(c["rel_yaw_deg"])
                ))
            except Exception:
                pass
    return out

def detect_yaw(rows):
    active = [x[1] for x in rows if abs(x[3][2]) > 0.1]
    if not active:
        raise RuntimeError("Yaw not detected")
    return min(active), max(active)

def subset(rows, lo, hi):
    return [x for x in rows if lo <= x[1] <= hi]

def vecmean(rows, idx):
    return tuple(mean([x[idx][i] for x in rows]) for i in range(3))

def vecsd(rows, idx):
    return tuple(sd([x[idx][i] for x in rows]) for i in range(3))

def tilt(acc):
    return math.degrees(math.atan2(math.hypot(acc[0],acc[1]), abs(acc[2])))

def analyze(name, imu_path, att_path):
    rows = load_imu(imu_path)
    att = load_att(att_path)
    ys, ye = detect_yaw(rows)
    end = rows[-1][1]

    ranges = {
        "PRE_EARLY": (0, 5),
        "PRE_LATE": (max(0,ys-6), ys-.5),
        "POST_0_2": (ye+.5, min(end,ye+2.5)),
        "POST_2_5": (ye+2.5, min(end,ye+5.5)),
        "POST_LATE": (ye+7, min(end,ye+12)),
    }
    regions = {k:subset(rows,*v) for k,v in ranges.items()}

    print("\n"+"="*76)
    print(name)
    print("="*76)
    print(f"duration={end:.3f}s  IMU={len(rows)}")
    print(f"YAW={ys:.3f}..{ye:.3f}s duration={ye-ys:.3f}s")

    if att:
        y = [x[3] for x in att]
        r = [x[1] for x in att]
        p = [x[2] for x in att]
        print(f"ATT yaw min/max/final={min(y):+.3f}/{max(y):+.3f}/{y[-1]:+.3f} deg")
        print(f"ATT roll range={min(r):+.3f}..{max(r):+.3f} deg")
        print(f"ATT pitch range={min(p):+.3f}..{max(p):+.3f} deg")

    for k,z in regions.items():
        gm = vecmean(z,3)
        gs = vecsd(z,3)
        am = vecmean(z,2)
        ass = vecsd(z,2)
        print(
            f"{k:11s} N={len(z):4d} "
            f"G=[{gm[0]:+.7f},{gm[1]:+.7f},{gm[2]:+.7f}] "
            f"Gsd=[{gs[0]:.7f},{gs[1]:.7f},{gs[2]:.7f}] "
            f"A=[{am[0]:+.4f},{am[1]:+.4f},{am[2]:+.4f}] "
            f"Asd=[{ass[0]:.4f},{ass[1]:.4f},{ass[2]:.4f}] "
            f"tilt={tilt(am):.3f}"
        )

    pre = vecmean(regions["PRE_EARLY"],3)
    post = vecmean(regions["POST_LATE"],3)
    d = tuple(post[i]-pre[i] for i in range(3))
    dxy = math.hypot(d[0],d[1])

    rp12 = math.degrees(dxy*12)
    leak12 = G*math.sin(math.radians(rp12))

    print("\nPRE_EARLY -> POST_LATE")
    print(f"dgyro=[{d[0]:+.7f},{d[1]:+.7f},{d[2]:+.7f}]")
    print(f"|dgyro XY|={dxy:.7f} rad/s")
    print(f"12s RP error={rp12:.4f} deg")
    print(f"12s gravity leakage={leak12:.6f} m/s2")

    print("\n1 SECOND TIMELINE")
    print("mark t      gx         gy         gz       tilt    gx_sd     gy_sd")

    timeline=[]
    sec=0
    while sec <= int(end):
        z=subset(rows,sec,sec+1)
        if len(z)>=20:
            gm=vecmean(z,3)
            gs=vecsd(z,3)
            am=vecmean(z,2)
            ti=tilt(am)
            mark=" "
            if ys <= sec+.5 <= ye:
                mark="Y"
            elif ye < sec+.5 <= ye+6:
                mark="P"
            timeline.append((sec+.5,gm,gs,am,ti))
            print(
                f"{mark} {sec+.5:4.1f} "
                f"{gm[0]:+.7f} {gm[1]:+.7f} {gm[2]:+.7f} "
                f"{ti:7.3f} {gs[0]:.7f} {gs[1]:.7f}"
            )
        sec += 1

    still=[x for x in timeline if x[0] < ys-.5 or x[0] > ye+.5]
    if len(still)>=5:
        gx=[x[1][0] for x in still]
        gy=[x[1][1] for x in still]
        ax=[x[3][0] for x in still]
        ay=[x[3][1] for x in still]
        ti=[x[4] for x in still]

        print("\nSTATIONARY CORRELATIONS OF 1s MEANS")
        print(f"corr(gx,ax)={corr(gx,ax):+.4f}")
        print(f"corr(gx,ay)={corr(gx,ay):+.4f}")
        print(f"corr(gy,ax)={corr(gy,ax):+.4f}")
        print(f"corr(gy,ay)={corr(gy,ay):+.4f}")
        print(f"corr(gx,tilt)={corr(gx,ti):+.4f}")
        print(f"corr(gy,tilt)={corr(gy,ti):+.4f}")

    return {
        "name":name,
        "d":d,
        "dxy":dxy,
        "regions":regions
    }

def main():
    res=[analyze(*x) for x in DATASETS]
    a,b=res

    print("\n"+"="*76)
    print("CROSS DATASET")
    print("="*76)
    print(f"V13    |dgyro XY|={a['dxy']:.7f}")
    print(f"V15.42 |dgyro XY|={b['dxy']:.7f}")
    print(f"V15.42/V13={b['dxy']/a['dxy']:.3f}x")

    print("\nPOST-YAW EVOLUTION OF GX")
    for reg in ("PRE_LATE","POST_0_2","POST_2_5","POST_LATE"):
        ga=vecmean(a["regions"][reg],3)[0]
        gb=vecmean(b["regions"][reg],3)[0]
        sa=vecsd(a["regions"][reg],3)[0]
        sb=vecsd(b["regions"][reg],3)[0]
        print(
            f"{reg:11s} "
            f"V13 mean={ga:+.7f} sd={sa:.7f} | "
            f"V15.42 mean={gb:+.7f} sd={sb:.7f}"
        )

    print("\nFORENSIC")
    print(
        "V15_42_POST_YAW_OFFSET_GT_3X_V13="
        + ("YES" if b["dxy"] > 3*a["dxy"] else "NO")
    )
    print("TEMPERATURE_AVAILABLE=NO")
    print("VIBRATION_MESSAGE_AVAILABLE=NO")
    print("RESULT=COMPLETE")

if __name__ == "__main__":
    main()
