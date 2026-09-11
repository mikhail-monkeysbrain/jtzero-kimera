#!/usr/bin/env python3
"""Анализ blind-move теста Ground Motion / ExternalNav.

Сценарий:
  1) исходная статика с открытой камерой;
  2) камера полностью закрывается;
  3) после выдержки стенд перемещается на известное расстояние при закрытой камере;
  4) после остановки камера всё ещё закрыта;
  5) камера открывается, затем наблюдается recovery.

Цель — не оценивать точность 150 мм как обычный optical-flow тест, а проверить,
теряет ли интегральная Ground Motion координата перемещение, произошедшее во
время visual dropout, и к какой координате затем сходится EKF.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Dict, List, Tuple

NS = 1_000_000_000


def med(vals: List[float]) -> float:
    return statistics.median(vals) if vals else float("nan")


def pct(n: int, d: int) -> float:
    return 100.0 * n / d if d else 0.0


def read_events(path: Path) -> Tuple[Dict[str, int], Dict[str, str]]:
    ts: Dict[str, int] = {}
    notes: Dict[str, str] = {}
    with path.open(newline="") as fh:
        for r in csv.DictReader(fh):
            ts[r["event"]] = int(r["mono_ns"])
            notes[r["event"]] = r.get("note", "")
    return ts, notes


def read_rows(path: Path):
    rows = []
    with path.open(newline="") as fh:
        rd = csv.DictReader(fh)
        need = {
            "mono_ns", "valid", "mav_sent", "x_m", "y_m",
            "quality", "inliers", "scatter_m",
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
                "quality": float(r["quality"]),
                "inliers": int(r["inliers"]),
                "scatter": float(r["scatter_m"]),
                # estimator хранит NWU; publisher отправляет NED N=x, E=-y
                "gn": float(r["x_m"]),
                "ge": -float(r["y_m"]),
                "ekfv": int(r["ekf_local_valid"]) != 0,
                "en": float(r["ekf_x_ned"]),
                "ee": float(r["ekf_y_ned"]),
                "evn": float(r["ekf_vx_ned"]),
                "eve": float(r["ekf_vy_ned"]),
            })
    return rows


def between(rows, a: int, b: int):
    return [r for r in rows if a <= r["t"] <= b]


def edge_value(seg, field: str, from_start: bool, width_s: float = 0.4) -> float:
    if not seg:
        return float("nan")
    if from_start:
        t0 = seg[0]["t"]
        sub = [r[field] for r in seg if r["t"] <= t0 + int(width_s * NS)]
    else:
        t1 = seg[-1]["t"]
        sub = [r[field] for r in seg if r["t"] >= t1 - int(width_s * NS)]
    return med(sub)


def vec_between(seg, x: str, y: str):
    if not seg:
        return float("nan"), float("nan"), float("nan")
    x0 = edge_value(seg, x, True)
    y0 = edge_value(seg, y, True)
    x1 = edge_value(seg, x, False)
    y1 = edge_value(seg, y, False)
    dx, dy = x1 - x0, y1 - y0
    return dx, dy, math.hypot(dx, dy)


def vec(a: Tuple[float, float], b: Tuple[float, float]):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return dx, dy, math.hypot(dx, dy)


def fmt_mm(v: float) -> str:
    return "nan" if not math.isfinite(v) else f"{v*1000:+7.1f}"


def segment_report(name: str, seg) -> None:
    if not seg:
        print(f"{name}: НЕТ ДАННЫХ")
        return
    valid = sum(r["valid"] for r in seg)
    sent = sum(r["sent"] for r in seg)
    ekfv = sum(r["ekfv"] for r in seg)
    gdn, gde, gd = vec_between(seg, "gn", "ge")
    edn, ede, ed = vec_between(seg, "en", "ee")
    vmax = max((math.hypot(r["evn"], r["eve"]) for r in seg if r["ekfv"]), default=0.0)
    dur = (seg[-1]["t"] - seg[0]["t"]) / NS
    print(
        f"{name:<22} dur={dur:5.2f}s rows={len(seg):4d} "
        f"GMvalid={valid:4d}/{len(seg):4d} ({pct(valid,len(seg)):5.1f}%) "
        f"MAVsent={sent:4d} EKFlocal={ekfv:4d}/{len(seg):4d}"
    )
    print(
        f"  GM delta=({fmt_mm(gdn)},{fmt_mm(gde)}) mm |d|={fmt_mm(gd)} mm; "
        f"EKF delta=({fmt_mm(edn)},{fmt_mm(ede)}) mm |d|={fmt_mm(ed)} mm; "
        f"EKF vmax={vmax:.3f} m/s"
    )


def window_state(rows, t: int, half_s: float = 0.25):
    half = int(half_s * NS)
    s = [r for r in rows if abs(r["t"] - t) <= half]
    if not s:
        return None
    ekf = [r for r in s if r["ekfv"]]
    if not ekf:
        return None
    return {
        "gn": med([r["gn"] for r in s]),
        "ge": med([r["ge"] for r in s]),
        "en": med([r["en"] for r in ekf]),
        "ee": med([r["ee"] for r in ekf]),
        "valid": sum(r["valid"] for r in s),
        "sent": sum(r["sent"] for r in s),
        "rows": len(s),
    }


def tail_state(rows, a: int, b: int):
    s = [r for r in rows if a <= r["t"] <= b]
    if not s:
        return None
    ekf = [r for r in s if r["ekfv"]]
    if not ekf:
        return None
    return {
        "gn": med([r["gn"] for r in s]),
        "ge": med([r["ge"] for r in s]),
        "en": med([r["en"] for r in ekf]),
        "ee": med([r["ee"] for r in ekf]),
    }


def valid_runs(seg, origin_ns: int):
    idx = [k for k, r in enumerate(seg) if r["valid"]]
    if not idx:
        return []
    runs = []
    a = p = idx[0]
    for k in idx[1:]:
        if k != p + 1:
            runs.append((a, p))
            a = k
        p = k
    runs.append((a, p))

    out = []
    for a, b in runs:
        rr = seg[a:b+1]
        out.append({
            "t0": (rr[0]["t"] - origin_ns) / 1e9,
            "t1": (rr[-1]["t"] - origin_ns) / 1e9,
            "n": len(rr),
            "q": max(r["quality"] for r in rr),
            "inl": max(r["inliers"] for r in rr),
            "sc": min(r["scatter"] for r in rr),
        })
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Анализ blind-move теста Ground Motion / ExternalNav")
    ap.add_argument("csv", type=Path)
    ap.add_argument("events", type=Path)
    args = ap.parse_args()

    ev, notes = read_events(args.events)
    rows = read_rows(args.csv)

    required = [
        "BLIND_MOVE_TARGET_MM",
        "STATIC_PRE_START", "STATIC_PRE_END",
        "BLOCK_START", "BLIND_SETTLE_END",
        "BLIND_MOVE_START", "BLIND_MOVE_END",
        "BLIND_STATIC_AFTER_MOVE_START", "BLIND_STATIC_AFTER_MOVE_END",
        "OPEN_START", "BLOCK_END", "RECOVERY_START", "RECOVERY_END",
    ]
    miss = [x for x in required if x not in ev]
    if miss:
        raise RuntimeError("events.csv неполный, отсутствуют: " + ", ".join(miss))

    try:
        target_mm = float(notes["BLIND_MOVE_TARGET_MM"].replace("\\", "").strip())
    except ValueError:
        target_mm = float("nan")

    segs = {
        "STATIC_PRE": between(rows, ev["STATIC_PRE_START"], ev["STATIC_PRE_END"]),
        "BLOCKED_PRE_MOVE": between(rows, ev["BLOCK_START"], ev["BLIND_MOVE_START"]),
        "BLIND_MOVE": between(rows, ev["BLIND_MOVE_START"], ev["BLIND_MOVE_END"]),
        "BLOCKED_POST_MOVE": between(rows, ev["BLIND_MOVE_END"], ev["OPEN_START"]),
        "UNBLOCKING": between(rows, ev["OPEN_START"], ev["BLOCK_END"]),
        "RECOVERY": between(rows, ev["RECOVERY_START"], ev["RECOVERY_END"]),
    }

    print("\n===== BLIND-MOVE TEST: СЕГМЕНТЫ =====")
    print(f"Физическое перемещение при закрытой камере: {target_mm:.1f} мм")
    for name, seg in segs.items():
        segment_report(name, seg)

    guaranteed_blocked = between(rows, ev["BLOCK_START"], ev["OPEN_START"])
    runs = valid_runs(guaranteed_blocked, ev["BLOCK_START"])

    print("\n===== VALID ВО ВРЕМЯ ГАРАНТИРОВАННО ЗАКРЫТОЙ КАМЕРЫ =====")
    print(
        f"rows={len(guaranteed_blocked)} valid={sum(r['valid'] for r in guaranteed_blocked)} "
        f"MAVsent={sum(r['sent'] for r in guaranteed_blocked)}"
    )
    if runs:
        for n, r in enumerate(runs, 1):
            print(
                f"  #{n}: t={r['t0']:.3f}..{r['t1']:.3f}s n={r['n']} "
                f"qmax={r['q']:.3f} inliers_max={r['inl']} scatter_min={r['sc']*1000:.3f}mm"
            )
    else:
        print("  valid-runs: НЕТ")

    before = window_state(rows, ev["BLIND_MOVE_START"])
    after = window_state(rows, ev["BLIND_MOVE_END"])
    final = tail_state(rows, ev["RECOVERY_END"] - 2 * NS, ev["RECOVERY_END"])
    baseline = tail_state(rows, ev["STATIC_PRE_END"] - NS, ev["STATIC_PRE_END"])

    print("\n===== ПОТЕРЯННОЕ ПЕРЕМЕЩЕНИЕ =====")
    if before and after:
        gdn, gde, gd = vec((before["gn"], before["ge"]), (after["gn"], after["ge"]))
        edn, ede, ed = vec((before["en"], before["ee"]), (after["en"], after["ee"]))
        print(
            f"между BLIND_MOVE_START и BLIND_MOVE_END:\n"
            f"  GM  delta=({fmt_mm(gdn)},{fmt_mm(gde)}) mm |d|={gd*1000:.1f} мм\n"
            f"  EKF delta=({fmt_mm(edn)},{fmt_mm(ede)}) mm |d|={ed*1000:.1f} мм\n"
            f"  физический target={target_mm:.1f} мм"
        )
    else:
        print("Недостаточно данных около BLIND_MOVE_START/END")

    if baseline and final:
        gdn, gde, gd = vec((baseline["gn"], baseline["ge"]), (final["gn"], final["ge"]))
        edn, ede, ed = vec((baseline["en"], baseline["ee"]), (final["en"], final["ee"]))
        dfn, dfe, dfd = vec((final["gn"], final["ge"]), (final["en"], final["ee"]))
        print("\n===== ИТОГ ПОСЛЕ 10 с RECOVERY =====")
        print(
            f"GM от исходной точки:  ({fmt_mm(gdn)},{fmt_mm(gde)}) mm |d|={gd*1000:.1f} мм\n"
            f"EKF от исходной точки: ({fmt_mm(edn)},{fmt_mm(ede)}) mm |d|={ed*1000:.1f} мм\n"
            f"EKF-GM в финале:       ({fmt_mm(dfn)},{fmt_mm(dfe)}) mm |d|={dfd*1000:.1f} мм\n"
            f"Физически стенд был смещён на {target_mm:.1f} мм при закрытой камере."
        )

        print("\n===== ИНТЕРПРЕТАЦИЯ =====")
        if math.isfinite(target_mm) and target_mm > 0:
            if gd * 1000 < 0.25 * target_mm:
                print(
                    "RESULT: после recovery Ground Motion сохранил почти старую координату и "
                    "не восстановил большую часть blind displacement."
                )
            else:
                print(
                    "CHECK: Ground Motion координата заметно изменилась; нужно проверить, "
                    "какая часть возникла из false-valid/reacquire и соответствует ли она реальному сдвигу."
                )
        if dfd * 1000 < 20.0:
            print(
                "RESULT: к концу recovery EKF снова почти совпал с координатой Ground Motion. "
                "Это доказывает reacquire, но НЕ доказывает правильность абсолютной координаты после blind move."
            )
        else:
            print(
                "CHECK: спустя 10 с recovery EKF ещё не сошёлся к Ground Motion; требуется разбор timeline."
            )
    else:
        print("\nНедостаточно baseline/final данных для итогового сравнения.")

    print(
        "\nВажно: физический target известен только как длина перемещения вдоль рельса. "
        "Его N/E компоненты здесь не предполагаются и не выдумываются."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
