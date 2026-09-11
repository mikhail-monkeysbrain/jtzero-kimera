#!/usr/bin/env python3
"""Анализ программного разрыва ExternalNav при продолжающем работать Ground Motion.

В этом тесте камера остаётся открытой, estimator продолжает считать и
интегрировать x/y, но диагностический wrapper подавляет только пару
VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE на интервале GAP.
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
    ts, notes = {}, {}
    with path.open(newline="") as f:
        for r in csv.DictReader(f):
            ts[r["event"]] = int(r["mono_ns"])
            notes[r["event"]] = r.get("note", "")
    return ts, notes


def read_rows(path: Path):
    rows = []
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
            rows.append({
                "t": int(r["mono_ns"]),
                "valid": int(r["valid"]) != 0,
                "sent": int(r["mav_sent"]) != 0,
                "gn": float(r["x_m"]),
                "ge": -float(r["y_m"]),
                "ekfv": int(r["ekf_local_valid"]) != 0,
                "en": float(r["ekf_x_ned"]),
                "ee": float(r["ekf_y_ned"]),
                "evn": float(r["ekf_vx_ned"]),
                "eve": float(r["ekf_vy_ned"]),
            })
    return rows


def between(rows, a, b):
    return [r for r in rows if a <= r["t"] <= b]


def state(rows, a, b):
    s = between(rows, a, b)
    if not s:
        return None
    e = [r for r in s if r["ekfv"]]
    return {
        "gn": med([r["gn"] for r in s]),
        "ge": med([r["ge"] for r in s]),
        "en": med([r["en"] for r in e]) if e else float("nan"),
        "ee": med([r["ee"] for r in e]) if e else float("nan"),
        "valid": sum(r["valid"] for r in s),
        "sent": sum(r["sent"] for r in s),
        "rows": len(s),
    }


def delta(a, b, x, y):
    dx = b[x] - a[x]
    dy = b[y] - a[y]
    return dx, dy, math.hypot(dx, dy)


def mm(v):
    return v * 1000.0


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Использование: {sys.argv[0]} <csv> <events.csv>", file=sys.stderr)
        return 2

    csv_path = Path(sys.argv[1])
    events_path = Path(sys.argv[2])
    ev, notes = read_events(events_path)
    rows = read_rows(csv_path)

    required = [
        "MOVE_TARGET_MM", "STATIC_PRE_START", "STATIC_PRE_END",
        "GAP_START", "GAP_SETTLE_END", "GAP_MOVE_START", "GAP_MOVE_END",
        "GAP_STATIC_AFTER_MOVE_START", "GAP_STATIC_AFTER_MOVE_END",
        "GAP_END", "RECOVERY_START", "RECOVERY_END",
    ]
    miss = [x for x in required if x not in ev]
    if miss:
        raise RuntimeError("events.csv неполный: " + ", ".join(miss))

    try:
        target_mm = float(notes["MOVE_TARGET_MM"].replace("\\", "").strip())
    except ValueError:
        target_mm = float("nan")

    print("===== EXTERNALNAV GAP TEST =====")
    print(f"Физический target: {target_mm:.1f} мм; камера всё время открыта")

    gap = between(rows, ev["GAP_START"], ev["GAP_END"])
    move = between(rows, ev["GAP_MOVE_START"], ev["GAP_MOVE_END"])
    recovery = between(rows, ev["RECOVERY_START"], ev["RECOVERY_END"])

    print("\n===== КОНТРОЛЬ GAP =====")
    if gap:
        print(
            f"GAP dur={(gap[-1]['t']-gap[0]['t'])/NS:.2f}s rows={len(gap)} "
            f"GMvalid={sum(r['valid'] for r in gap)}/{len(gap)} "
            f"({100*sum(r['valid'] for r in gap)/len(gap):.1f}%) "
            f"MAVsent={sum(r['sent'] for r in gap)}"
        )
        if sum(r["sent"] for r in gap) == 0:
            print("PASS gate: во время GAP ExternalNav не отправлялся.")
        else:
            print("FAIL gate: во время GAP есть mav_sent=1.")
    else:
        print("FAIL: GAP пуст")

    # Односторонние окна исключают первые/последние миллисекунды ручного движения.
    pre_move = state(rows, ev["GAP_MOVE_START"] - int(0.5*NS), ev["GAP_MOVE_START"])
    post_move = state(rows, ev["GAP_MOVE_END"], ev["GAP_MOVE_END"] + int(0.5*NS))

    print("\n===== ДВИЖЕНИЕ ВО ВРЕМЯ GAP =====")
    gm_move_d = float("nan")
    if pre_move and post_move:
        gdn, gde, gm_move_d = delta(pre_move, post_move, "gn", "ge")
        edn, ede, ekf_move_d = delta(pre_move, post_move, "en", "ee")
        err = abs(mm(gm_move_d) - target_mm) if math.isfinite(target_mm) else float("nan")
        print(
            f"GM:  dN={mm(gdn):+7.1f} dE={mm(gde):+7.1f} |d|={mm(gm_move_d):7.1f} мм\n"
            f"EKF: dN={mm(edn):+7.1f} dE={mm(ede):+7.1f} |d|={mm(ekf_move_d):7.1f} мм\n"
            f"target={target_mm:.1f} мм; |GM-target|={err:.1f} мм"
        )
    else:
        print("Недостаточно данных около GAP_MOVE_START/END")

    baseline = state(rows, ev["STATIC_PRE_END"] - NS, ev["STATIC_PRE_END"])
    before_resume = state(rows, ev["GAP_END"] - int(0.5*NS), ev["GAP_END"])
    final = state(rows, ev["RECOVERY_END"] - 2*NS, ev["RECOVERY_END"])

    print("\n===== СОСТОЯНИЕ ПЕРЕД ВОЗВРАТОМ EXTERNALNAV =====")
    if before_resume:
        dn = before_resume["en"] - before_resume["gn"]
        de = before_resume["ee"] - before_resume["ge"]
        print(
            f"GM=({mm(before_resume['gn']):+7.1f},{mm(before_resume['ge']):+7.1f}) мм "
            f"EKF=({mm(before_resume['en']):+7.1f},{mm(before_resume['ee']):+7.1f}) мм "
            f"EKF-GM |D|={mm(math.hypot(dn,de)):.1f} мм"
        )

    print("\n===== ФИНАЛ RECOVERY =====")
    if baseline and final:
        gdn, gde, gd = delta(baseline, final, "gn", "ge")
        edn, ede, ed = delta(baseline, final, "en", "ee")
        dfn = final["en"] - final["gn"]
        dfe = final["ee"] - final["ge"]
        dfd = math.hypot(dfn, dfe)
        print(
            f"GM от исходной точки:  dN={mm(gdn):+7.1f} dE={mm(gde):+7.1f} |d|={mm(gd):7.1f} мм\n"
            f"EKF от исходной точки: dN={mm(edn):+7.1f} dE={mm(ede):+7.1f} |d|={mm(ed):7.1f} мм\n"
            f"EKF-GM финально:       dN={mm(dfn):+7.1f} dE={mm(dfe):+7.1f} |D|={mm(dfd):7.1f} мм"
        )

        print("\n===== ИНТЕРПРЕТАЦИЯ =====")
        if math.isfinite(target_mm) and math.isfinite(gm_move_d):
            if abs(mm(gm_move_d) - target_mm) <= max(15.0, 0.10*target_mm):
                print("PASS GM continuity: Ground Motion сохранил физическое перемещение во время разрыва публикации.")
            else:
                print("CHECK GM continuity: измеренная GM дистанция заметно отличается от физического target.")
        if mm(dfd) <= 20.0:
            print("PASS EKF reacquire: после возврата корректной ExternalNav координаты EKF снова сошёлся к GM.")
        else:
            print("FAIL/CHECK EKF reacquire: спустя recovery EKF остаётся далеко от GM.")

    # Timeline recovery относительно GAP_END.
    print("\n===== RECOVERY TIMELINE 0.5 s =====")
    if recovery:
        t = ev["GAP_END"]
        end = ev["RECOVERY_END"]
        step = int(0.5*NS)
        while t < end:
            s = state(rows, t, min(t+step, end))
            if s and math.isfinite(s["en"]):
                dn = s["en"] - s["gn"]
                de = s["ee"] - s["ge"]
                print(
                    f"{(t-ev['GAP_END'])/NS:5.1f}s "
                    f"valid={s['valid']:3d}/{s['rows']:3d} sent={s['sent']:3d} "
                    f"GM=({mm(s['gn']):+7.1f},{mm(s['ge']):+7.1f}) "
                    f"EKF=({mm(s['en']):+7.1f},{mm(s['ee']):+7.1f}) "
                    f"|D|={mm(math.hypot(dn,de)):7.1f} мм"
                )
            t += step

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
