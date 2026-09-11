#!/usr/bin/env python3
"""Детальная временная шкала blind-move теста Ground Motion / ExternalNav.

Показывает абсолютное расхождение EKF и Ground Motion до закрытия камеры,
во время реального перемещения при visual dropout и после reacquire.
Ничего не меняет в FC или estimator.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

NS = 1_000_000_000


def med(vals):
    return statistics.median(vals) if vals else float("nan")


def read_events(path: Path):
    out = {}
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            out[r["event"]] = int(r["mono_ns"])
    return out


def read_rows(path: Path):
    out = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        need = {
            "mono_ns", "valid", "mav_sent", "x_m", "y_m",
            "ekf_local_valid", "ekf_x_ned", "ekf_y_ned",
            "ekf_vx_ned", "ekf_vy_ned",
        }
        miss = need.difference(rd.fieldnames or [])
        if miss:
            raise RuntimeError(f"{path}: нет колонок: {', '.join(sorted(miss))}")
        for r in rd:
            out.append({
                "t": int(r["mono_ns"]),
                "valid": int(r["valid"]) != 0,
                "sent": int(r["mav_sent"]) != 0,
                # estimator хранит NWU; publisher отправляет NED N=x, E=-y
                "gn": float(r["x_m"]),
                "ge": -float(r["y_m"]),
                "ekfv": int(r["ekf_local_valid"]) != 0,
                "en": float(r["ekf_x_ned"]),
                "ee": float(r["ekf_y_ned"]),
                "evn": float(r["ekf_vx_ned"]),
                "eve": float(r["ekf_vy_ned"]),
            })
    return out


def snapshot(rows, t, half_ms=150):
    half = half_ms * 1_000_000
    s = [r for r in rows if abs(r["t"] - t) <= half]
    ekf = [r for r in s if r["ekfv"]]
    if not s or not ekf:
        return None
    gn = med([r["gn"] for r in s])
    ge = med([r["ge"] for r in s])
    en = med([r["en"] for r in ekf])
    ee = med([r["ee"] for r in ekf])
    return {
        "gn": gn, "ge": ge, "en": en, "ee": ee,
        "dn": en - gn, "de": ee - ge,
        "d": math.hypot(en - gn, ee - ge),
        "valid": sum(r["valid"] for r in s),
        "sent": sum(r["sent"] for r in s),
        "rows": len(s),
    }


def main():
    ap = argparse.ArgumentParser(description="Timeline blind-move теста Ground Motion")
    ap.add_argument("csv", type=Path)
    ap.add_argument("events", type=Path)
    ap.add_argument("--bin", type=float, default=0.5, help="ширина bin, секунд")
    args = ap.parse_args()

    ev = read_events(args.events)
    rows = read_rows(args.csv)

    required = [
        "STATIC_PRE_END",
        "BLOCK_START", "BLIND_SETTLE_END",
        "BLIND_MOVE_START", "BLIND_MOVE_END",
        "BLIND_STATIC_AFTER_MOVE_END",
        "OPEN_START", "BLOCK_END", "RECOVERY_END",
    ]
    miss = [x for x in required if x not in ev]
    if miss:
        raise RuntimeError("events.csv неполный: " + ", ".join(miss))

    start = ev["STATIC_PRE_END"]
    end = ev["RECOVERY_END"]

    print("===== СОБЫТИЯ / EKF-GM OFFSET =====")
    for name in required:
        s = snapshot(rows, ev[name])
        rel = (ev[name] - start) / NS
        if s is None:
            print(f"{name:<29} t={rel:7.3f}s нет EKF данных")
            continue
        print(
            f"{name:<29} t={rel:7.3f}s "
            f"GM=({s['gn']*1000:+8.1f},{s['ge']*1000:+8.1f}) mm "
            f"EKF=({s['en']*1000:+8.1f},{s['ee']*1000:+8.1f}) mm "
            f"EKF-GM=({s['dn']*1000:+8.1f},{s['de']*1000:+8.1f}) "
            f"|D|={s['d']*1000:7.1f} mm valid={s['valid']}/{s['rows']} sent={s['sent']}"
        )

    bin_ns = max(1, int(args.bin * NS))

    print("\n===== TIMELINE =====")
    print(" t[s]   valid sent   GM_N     GM_E      EKF_N    EKF_E     D_N      D_E     |D|   EKF_v")

    k = 0
    t = start
    while t < end:
        t1 = min(t + bin_ns, end)
        b = [r for r in rows if t <= r["t"] < t1]
        if b:
            ekf = [r for r in b if r["ekfv"]]
            if ekf:
                gn = med([r["gn"] for r in b])
                ge = med([r["ge"] for r in b])
                en = med([r["en"] for r in ekf])
                ee = med([r["ee"] for r in ekf])
                dn, de = en - gn, ee - ge
                d = math.hypot(dn, de)
                vmax = max((math.hypot(r["evn"], r["eve"]) for r in ekf), default=float("nan"))
                print(
                    f"{(t-start)/NS:6.2f} "
                    f"{sum(r['valid'] for r in b):5d} {sum(r['sent'] for r in b):4d} "
                    f"{gn*1000:+8.1f} {ge*1000:+8.1f} "
                    f"{en*1000:+8.1f} {ee*1000:+8.1f} "
                    f"{dn*1000:+8.1f} {de*1000:+8.1f} {d*1000:7.1f} "
                    f"{vmax:6.3f}"
                )
        k += 1
        t = start + k * bin_ns

    tail_start = ev["RECOVERY_END"] - 2 * NS
    tail = [r for r in rows if tail_start <= r["t"] <= ev["RECOVERY_END"] and r["ekfv"]]
    if tail:
        gn = med([r["gn"] for r in tail])
        ge = med([r["ge"] for r in tail])
        en = med([r["en"] for r in tail])
        ee = med([r["ee"] for r in tail])
        dn, de = en - gn, ee - ge
        print("\n===== ФИНАЛ RECOVERY, последние 2 с =====")
        print(
            f"GM N/E=({gn*1000:+.1f},{ge*1000:+.1f}) mm; "
            f"EKF N/E=({en*1000:+.1f},{ee*1000:+.1f}) mm; "
            f"EKF-GM=({dn*1000:+.1f},{de*1000:+.1f}) mm; "
            f"|D|={math.hypot(dn,de)*1000:.1f} mm"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
