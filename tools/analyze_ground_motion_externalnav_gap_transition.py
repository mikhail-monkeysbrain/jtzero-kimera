#!/usr/bin/env python3
"""Точная временная шкала потери/возврата FC LOCAL_POSITION_NED вокруг ExternalNav GAP.

Ничего не меняет в estimator или FC. Использует уже записанные CSV + events.csv.
Показывает:
- когда ekf_local_valid исчезает во время GAP;
- последнюю свежую LOCAL_POSITION_NED перед GAP_END;
- первую свежую LOCAL_POSITION_NED после GAP_END;
- первые 1000 мс recovery с шагом 50 мс;
- EKF-GM offset на каждом шаге.
"""

from __future__ import annotations

import csv
import math
import statistics
import sys
from pathlib import Path

NS = 1_000_000_000


def med(v):
    return statistics.median(v) if v else float("nan")


def read_events(path: Path):
    out = {}
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            out[r["event"]] = int(r["mono_ns"])
    return out


def read_rows(path: Path):
    rows = []
    with path.open(newline="") as f:
        rd = csv.DictReader(f)
        need = {
            "mono_ns", "valid", "mav_sent", "x_m", "y_m",
            "ekf_local_valid", "ekf_x_ned", "ekf_y_ned",
            "ekf_local_age_ms", "ekf_local_count",
            "ekf_vx_ned", "ekf_vy_ned",
        }
        miss = need.difference(rd.fieldnames or [])
        if miss:
            raise RuntimeError(f"{path}: нет колонок: {', '.join(sorted(miss))}")
        for r in rd:
            rows.append({
                "t": int(r["mono_ns"]),
                "valid": int(r["valid"]) != 0,
                "sent": int(r["mav_sent"]) != 0,
                "gn": float(r["x_m"]),
                "ge": -float(r["y_m"]),
                "ekfv": int(r["ekf_local_valid"]) != 0,
                "en": float(r["ekf_x_ned"]),
                "ee": float(r["ekf_y_ned"]),
                "age": float(r["ekf_local_age_ms"]),
                "count": int(r["ekf_local_count"]),
                "evn": float(r["ekf_vx_ned"]),
                "eve": float(r["ekf_vy_ned"]),
            })
    return rows


def state(rows, a, b):
    s = [r for r in rows if a <= r["t"] < b]
    if not s:
        return None
    fresh = [r for r in s if r["ekfv"]]
    out = {
        "rows": len(s),
        "valid": sum(r["valid"] for r in s),
        "sent": sum(r["sent"] for r in s),
        "gn": med([r["gn"] for r in s]),
        "ge": med([r["ge"] for r in s]),
        "fresh_n": len(fresh),
        "age_med": med([r["age"] for r in s]),
        "count_min": min(r["count"] for r in s),
        "count_max": max(r["count"] for r in s),
    }
    if fresh:
        out["en"] = med([r["en"] for r in fresh])
        out["ee"] = med([r["ee"] for r in fresh])
        out["ev"] = max(math.hypot(r["evn"], r["eve"]) for r in fresh)
        out["d"] = math.hypot(out["en"] - out["gn"], out["ee"] - out["ge"])
    else:
        out["en"] = out["ee"] = out["ev"] = out["d"] = float("nan")
    return out


def main():
    if len(sys.argv) != 3:
        print(f"Использование: {sys.argv[0]} <csv> <events.csv>", file=sys.stderr)
        return 2

    rows = read_rows(Path(sys.argv[1]))
    ev = read_events(Path(sys.argv[2]))
    for name in ("GAP_START", "GAP_END", "RECOVERY_END"):
        if name not in ev:
            raise RuntimeError(f"events.csv: нет {name}")

    gap_start = ev["GAP_START"]
    gap_end = ev["GAP_END"]

    gap_rows = [r for r in rows if gap_start <= r["t"] < gap_end]
    fresh_gap = [r for r in gap_rows if r["ekfv"]]

    print("===== FC LOCAL_POSITION_NED ВО ВРЕМЯ GAP =====")
    if fresh_gap:
        last = fresh_gap[-1]
        print(
            f"последняя строка ekf_local_valid=1 перед GAP_END: "
            f"t={(last['t']-gap_end)/1e3:+.0f} us относительно GAP_END; "
            f"age={last['age']:.1f} ms count={last['count']} "
            f"EKF=({last['en']*1000:+.1f},{last['ee']*1000:+.1f}) mm"
        )
    else:
        print("внутри GAP не было ни одной свежей LOCAL_POSITION_NED строки")

    # Найти первый переход 1 -> 0, который затем держится хотя бы 0.5 с.
    sustained_loss = None
    for k, r in enumerate(gap_rows):
        if r["ekfv"]:
            continue
        t0 = r["t"]
        tail = [x for x in gap_rows if t0 <= x["t"] < t0 + 500_000_000]
        if tail and all(not x["ekfv"] for x in tail):
            sustained_loss = t0
            break
    if sustained_loss is not None:
        print(
            f"устойчивая потеря ekf_local_valid началась: "
            f"{(sustained_loss-gap_start)/1e9:.3f} s после GAP_START "
            f"({(sustained_loss-gap_end)/1e9:.3f} s относительно GAP_END)"
        )
    else:
        print("устойчивая потеря ekf_local_valid на >=0.5 s не найдена")

    after = [r for r in rows if r["t"] >= gap_end]
    first_sent = next((r for r in after if r["sent"]), None)
    first_fresh = next((r for r in after if r["ekfv"]), None)

    print("\n===== ПЕРВЫЕ СОБЫТИЯ ПОСЛЕ GAP_END =====")
    if first_sent:
        print(f"first mav_sent=1: {(first_sent['t']-gap_end)/1e6:.1f} ms")
    else:
        print("first mav_sent=1: НЕТ")
    if first_fresh:
        d = math.hypot(first_fresh["en"]-first_fresh["gn"], first_fresh["ee"]-first_fresh["ge"])
        print(
            f"first ekf_local_valid=1: {(first_fresh['t']-gap_end)/1e6:.1f} ms; "
            f"GM=({first_fresh['gn']*1000:+.1f},{first_fresh['ge']*1000:+.1f}) mm "
            f"EKF=({first_fresh['en']*1000:+.1f},{first_fresh['ee']*1000:+.1f}) mm "
            f"|D|={d*1000:.1f} mm age={first_fresh['age']:.1f} ms count={first_fresh['count']}"
        )
    else:
        print("first ekf_local_valid=1: НЕТ")

    print("\n===== ПЕРВАЯ 1 СЕКУНДА RECOVERY, BIN 50 ms =====")
    step = 50_000_000
    for i in range(20):
        a = gap_end + i*step
        b = a + step
        s = state(rows, a, b)
        if not s:
            continue
        if math.isfinite(s["d"]):
            pos = f"EKF=({s['en']*1000:+7.1f},{s['ee']*1000:+7.1f}) |D|={s['d']*1000:6.1f}"
        else:
            pos = "EKF=(нет свежей LOCAL_POSITION_NED)"
        print(
            f"{i*50:4d}..{(i+1)*50:4d} ms "
            f"sent={s['sent']:2d}/{s['rows']:2d} fresh={s['fresh_n']:2d}/{s['rows']:2d} "
            f"GM=({s['gn']*1000:+7.1f},{s['ge']*1000:+7.1f}) {pos} "
            f"age_med={s['age_med']:.1f}ms count={s['count_min']}..{s['count_max']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
