#!/usr/bin/env python3
"""Анализ теста потери Ground Motion / ExternalNav.

Использует CSV production Ground Motion и отдельный events.csv от
run_ground_motion_loss_test.sh. Ничего не меняет в FC и estimator.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Dict, List


NS = 1_000_000_000


def f(v: str) -> float:
    return float(v)


def i(v: str) -> int:
    return int(v)


def med(vals: List[float]) -> float:
    return statistics.median(vals) if vals else float("nan")


def pct(n: int, d: int) -> float:
    return 100.0 * n / d if d else 0.0


def read_events(path: Path) -> Dict[str, int]:
    out: Dict[str, int] = {}
    with path.open(newline="") as fh:
        for r in csv.DictReader(fh):
            out[r["event"]] = int(r["mono_ns"])
    return out


def read_rows(path: Path):
    rows = []
    with path.open(newline="") as fh:
        rd = csv.DictReader(fh)
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
                "t": i(r["mono_ns"]),
                "valid": i(r["valid"]) != 0,
                "sent": i(r["mav_sent"]) != 0,
                # estimator хранит NWU; publisher отправляет NED: N=x, E=-y
                "gn": f(r["x_m"]),
                "ge": -f(r["y_m"]),
                "ekfv": i(r["ekf_local_valid"]) != 0,
                "en": f(r["ekf_x_ned"]),
                "ee": f(r["ekf_y_ned"]),
                "evn": f(r["ekf_vx_ned"]),
                "eve": f(r["ekf_vy_ned"]),
            })
    return rows


def between(rows, a: int, b: int):
    return [r for r in rows if a <= r["t"] <= b]


def edge_median(seg, field: str, from_start: bool, width_s: float = 0.5) -> float:
    if not seg:
        return float("nan")
    if from_start:
        t0 = seg[0]["t"]
        sub = [r[field] for r in seg if r["t"] <= t0 + int(width_s * NS)]
    else:
        t1 = seg[-1]["t"]
        sub = [r[field] for r in seg if r["t"] >= t1 - int(width_s * NS)]
    return med(sub)


def vec_drift(seg, x: str, y: str):
    if not seg:
        return float("nan"), float("nan"), float("nan")
    x0 = edge_median(seg, x, True)
    y0 = edge_median(seg, y, True)
    x1 = edge_median(seg, x, False)
    y1 = edge_median(seg, y, False)
    dx, dy = x1 - x0, y1 - y0
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
    gdn, gde, gd = vec_drift(seg, "gn", "ge")
    edn, ede, ed = vec_drift(seg, "en", "ee")
    vmax = max((math.hypot(r["evn"], r["eve"]) for r in seg if r["ekfv"]), default=0.0)
    dur = (seg[-1]["t"] - seg[0]["t"]) / NS
    print(
        f"{name:<18} dur={dur:5.2f}s rows={len(seg):4d} "
        f"GMvalid={valid:4d}/{len(seg):4d} ({pct(valid,len(seg)):5.1f}%) "
        f"MAVsent={sent:4d} EKFlocal={ekfv:4d}/{len(seg):4d}"
    )
    print(
        f"  GM drift N/E=({fmt_mm(gdn)},{fmt_mm(gde)}) mm |d|={fmt_mm(gd)} mm; "
        f"EKF drift=({fmt_mm(edn)},{fmt_mm(ede)}) mm |d|={fmt_mm(ed)} mm; "
        f"EKF vmax={vmax:.3f} m/s"
    )


def first_after(rows, t0: int, pred):
    for r in rows:
        if r["t"] >= t0 and pred(r):
            return r
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Анализ теста потери Ground Motion / ExternalNav")
    ap.add_argument("csv", type=Path)
    ap.add_argument("events", type=Path)
    args = ap.parse_args()

    ev = read_events(args.events)
    rows = read_rows(args.csv)

    required = [
        "STATIC_PRE_START", "STATIC_PRE_END", "MOVE_START", "MOVE_END",
        "STATIC_AFTER_MOVE_START", "STATIC_AFTER_MOVE_END",
        "BLOCK_START", "BLOCK_END", "RECOVERY_START", "RECOVERY_END",
    ]
    miss = [x for x in required if x not in ev]
    if miss:
        raise RuntimeError("events.csv неполный, отсутствуют: " + ", ".join(miss))

    segs = {
        "STATIC_PRE": between(rows, ev["STATIC_PRE_START"], ev["STATIC_PRE_END"]),
        "MOVE": between(rows, ev["MOVE_START"], ev["MOVE_END"]),
        "STATIC_AFTER_MOVE": between(rows, ev["STATIC_AFTER_MOVE_START"], ev["STATIC_AFTER_MOVE_END"]),
        "BLOCKED": between(rows, ev["BLOCK_START"], ev["BLOCK_END"]),
        "RECOVERY": between(rows, ev["RECOVERY_START"], ev["RECOVERY_END"]),
    }

    print("\n===== LOSS TEST: СЕГМЕНТЫ =====")
    for name, seg in segs.items():
        segment_report(name, seg)

    block_start = ev["BLOCK_START"]
    block_end = ev["BLOCK_END"]

    first_invalid = first_after(rows, block_start, lambda r: not r["valid"])
    first_valid_recovery = first_after(rows, block_end, lambda r: r["valid"])
    first_sent_recovery = first_after(rows, block_end, lambda r: r["sent"])

    core = between(rows, block_start + NS, block_end)
    core_valid = sum(r["valid"] for r in core)
    core_sent = sum(r["sent"] for r in core)

    print("\n===== ПОТЕРЯ / ВОССТАНОВЛЕНИЕ =====")
    if first_invalid:
        print(f"first invalid after BLOCK_START: {(first_invalid['t']-block_start)/1e6:.1f} ms")
    else:
        print("first invalid after BLOCK_START: НЕ ОБНАРУЖЕН")

    print(
        f"blocked core (после первой 1 с): rows={len(core)} "
        f"valid={core_valid} MAVsent={core_sent}"
    )

    if first_valid_recovery:
        print(f"first valid after BLOCK_END: {(first_valid_recovery['t']-block_end)/1e6:.1f} ms")
    else:
        print("first valid after BLOCK_END: НЕ ВОССТАНОВИЛСЯ")

    if first_sent_recovery:
        print(f"first MAV send after BLOCK_END: {(first_sent_recovery['t']-block_end)/1e6:.1f} ms")
    else:
        print("first MAV send after BLOCK_END: НЕ ВОССТАНОВИЛСЯ")

    blocked = segs["BLOCKED"]
    edn, ede, ed = vec_drift(blocked, "en", "ee")
    print(
        f"EKF drift while camera blocked: dN={fmt_mm(edn)} mm "
        f"dE={fmt_mm(ede)} mm |d|={fmt_mm(ed)} mm"
    )

    print("\n===== ИНТЕРПРЕТАЦИЯ =====")
    if core and core_valid == 0 and core_sent == 0:
        print("PASS publisher: после потери изображения valid=0 и ExternalNav сообщения не отправляются.")
    else:
        print("CHECK publisher: во время закрытой камеры остались valid/MAVsent строки; смотреть CSV/console.")

    if first_valid_recovery and first_sent_recovery:
        print("PASS recovery: после открытия камеры Ground Motion снова стал valid и публикация возобновилась.")
    else:
        print("FAIL recovery: после открытия камеры valid/publish не восстановились в пределах теста.")

    print(
        "Важно: ekf_local_valid здесь означает свежий LOCAL_POSITION_NED от FC, "
        "а не отдельный флаг принятия ExternalNav. Поведение EKF оцениваем по траектории/скорости; "
        "для точного статуса aiding нужен отдельный EKF_STATUS_REPORT лог."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
