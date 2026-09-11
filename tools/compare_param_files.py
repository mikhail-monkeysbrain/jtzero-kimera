#!/usr/bin/env python3
# JT-Zero — сравнение двух экспортов ArduPilot .param без доступа к FC.
# НИЧЕГО не записывает.
#
# usage:
#   python3 tools/compare_param_files.py old.param new.param

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

CRITICAL = [
    "AHRS_EKF_TYPE",
    "AHRS_GPS_USE",
    "FLOW_TYPE",
    "FLOW_OPTIONS",
    "FLOW_ORIENT_YAW",
    "FLOW_FXSCALER",
    "FLOW_FYSCALER",
    "EK3_FLOW_USE",
    "EK3_SRC_OPTIONS",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "EK3_PRIMARY",
    "RNGFND1_TYPE",
    "RNGFND1_ORIENT",
    "RNGFND1_MIN_CM",
    "RNGFND1_MAX_CM",
    "SERIAL0_PROTOCOL",
    "SERIAL0_BAUD",
    "SERIAL1_PROTOCOL",
    "SERIAL1_BAUD",
    "SERIAL2_PROTOCOL",
    "SERIAL2_BAUD",
    "SERIAL3_PROTOCOL",
    "SERIAL3_BAUD",
    "SERIAL4_PROTOCOL",
    "SERIAL4_BAUD",
    "SERIAL5_PROTOCOL",
    "SERIAL5_BAUD",
    "LOG_BACKEND_TYPE",
    "LOG_DISARMED",
    "LOG_REPLAY",
    "EK3_LOG_LEVEL",
]

def parse_param_file(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    for raw in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        parts = [x.strip() for x in s.split(",")] if "," in s else s.split()
        if len(parts) < 2:
            continue
        try:
            out[parts[0]] = float(parts[1])
        except ValueError:
            continue
    return out

def close(a: float, b: float) -> bool:
    tol = max(1e-6, 1e-5 * max(abs(a), abs(b), 1.0))
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)

def fmt(v):
    if v is None:
        return "—"
    return f"{v:.9g}"

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("old_param")
    ap.add_argument("new_param")
    ap.add_argument("--all-new", action="store_true")
    args = ap.parse_args()

    oldp = Path(args.old_param)
    newp = Path(args.new_param)
    if not oldp.is_file():
        print(f"ОШИБКА: old file не найден: {oldp}", file=sys.stderr)
        return 2
    if not newp.is_file():
        print(f"ОШИБКА: new file не найден: {newp}", file=sys.stderr)
        return 3

    old = parse_param_file(oldp)
    new = parse_param_file(newp)

    common = sorted(old.keys() & new.keys())
    missing = sorted(old.keys() - new.keys())
    added = sorted(new.keys() - old.keys())
    changed = [(n, old[n], new[n]) for n in common if not close(old[n], new[n])]

    print("=" * 78)
    print("JT-ZERO — OLD.PARAM vs NEW.PARAM")
    print("=" * 78)
    print(f"OLD: {oldp} params={len(old)}")
    print(f"NEW: {newp} params={len(new)}")

    print("\n===== СВОДКА =====")
    print(f"COMMON={len(common)}")
    print(f"CHANGED={len(changed)}")
    print(f"MISSING_IN_NEW={len(missing)}")
    print(f"ADDED_IN_NEW={len(added)}")

    print("\n===== КРИТИЧНЫЕ JT-ZERO =====")
    print(f"{'PARAM':24s} {'OLD':>14s} {'NEW':>14s}  STATUS")
    for n in CRITICAL:
        ov = old.get(n)
        nv = new.get(n)
        if ov is None and nv is None:
            status = "ABSENT"
        elif ov is None:
            status = "ADDED"
        elif nv is None:
            status = "MISSING"
        elif close(ov, nv):
            status = "SAME"
        else:
            status = "DIFF"
        print(f"{n:24s} {fmt(ov):>14s} {fmt(nv):>14s}  {status}")

    print("\n===== ИЗМЕНЁННЫЕ ЗНАЧЕНИЯ =====")
    if not changed:
        print("нет")
    else:
        for n, ov, nv in changed:
            print(f"{n:28s} old={fmt(ov):>12s} new={fmt(nv):>12s}")

    print("\n===== БЫЛИ В OLD, НО НЕТ В NEW =====")
    if not missing:
        print("нет")
    else:
        for n in missing:
            print(f"{n}={fmt(old[n])}")

    if args.all_new:
        print("\n===== ПОЯВИЛИСЬ ТОЛЬКО В NEW =====")
        for n in added:
            print(f"{n}={fmt(new[n])}")

    print("\n===== VERDICT =====")
    print("FILE_TO_FILE_ONLY=YES")
    print("FC_ACCESS_REQUIRED=NO")
    print("FC_PARAMS_WRITTEN=0")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
