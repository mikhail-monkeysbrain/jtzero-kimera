#!/usr/bin/env python3
# JT-Zero — безопасный аудит старого .param против текущей прошивки FC.
# НИЧЕГО не записывает в FC.
#
# usage:
#   python3 tools/audit_saved_params_vs_fc.py old.param --device /dev/ttyAMA0
#
# Вывод:
#   - параметры, отсутствующие в новой прошивке;
#   - новые параметры, которых не было в старом файле;
#   - отличающиеся значения;
#   - отдельный блок критичных JT-Zero параметров.

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

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
    "SERIAL1_PROTOCOL",
    "SERIAL1_BAUD",
    "SERIAL2_PROTOCOL",
    "SERIAL2_BAUD",
    "LOG_BACKEND_TYPE",
    "LOG_DISARMED",
    "LOG_REPLAY",
    "EK3_LOG_LEVEL",
]

def parse_param_file(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    for no, raw in enumerate(path.read_text(encoding="utf-8-sig", errors="replace").splitlines(), 1):
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        if "," in s:
            parts = [x.strip() for x in s.split(",")]
        else:
            parts = s.split()
        if len(parts) < 2:
            continue
        name = parts[0]
        try:
            value = float(parts[1])
        except ValueError:
            continue
        out[name] = value
    return out

def wait_fc(master, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if m is None:
            continue
        if int(getattr(m, "autopilot", -1)) == int(mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            return int(m.get_srcSystem()), int(m.get_srcComponent())
    return None

def read_all_params(master, sysid: int, compid: int, timeout=12.0) -> dict[str, float]:
    master.mav.param_request_list_send(sysid, compid)
    out: dict[str, float] = {}
    expected = None
    last_new = time.monotonic()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if m is None:
            if expected is not None and len(out) >= expected:
                break
            if time.monotonic() - last_new > 2.0 and out:
                # повторный запрос списка помогает при потерях
                master.mav.param_request_list_send(sysid, compid)
                last_new = time.monotonic()
            continue
        if int(m.get_srcSystem()) != sysid:
            continue
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("ascii", errors="ignore")
        name = str(pid).rstrip("\x00")
        if name not in out:
            last_new = time.monotonic()
        out[name] = float(m.param_value)
        try:
            expected = int(m.param_count)
        except Exception:
            pass
        if expected is not None and len(out) >= expected:
            break
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
    ap.add_argument("param_file")
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--show-all-new", action="store_true")
    args = ap.parse_args()

    p = Path(args.param_file)
    if not p.is_file():
        print(f"ОШИБКА: файл не найден: {p}", file=sys.stderr)
        return 2

    saved = parse_param_file(p)
    print(f"Сохранённый файл: {p}")
    print(f"Параметров в файле: {len(saved)}")

    master = mavutil.mavlink_connection(
        args.device, baud=args.baud,
        source_system=191, source_component=199,
        autoreconnect=False,
    )
    fc = wait_fc(master)
    if fc is None:
        print("ОШИБКА: ArduPilot HEARTBEAT не найден", file=sys.stderr)
        return 3
    sysid, compid = fc
    print(f"FC heartbeat sys={sysid} comp={compid}")

    current = read_all_params(master, sysid, compid)
    print(f"Параметров прочитано с FC: {len(current)}")
    if len(current) < 500:
        print("ОШИБКА: получен явно неполный список параметров; ничего не анализирую.", file=sys.stderr)
        return 4

    common = sorted(saved.keys() & current.keys())
    missing = sorted(saved.keys() - current.keys())
    new = sorted(current.keys() - saved.keys())
    changed = [(n, saved[n], current[n]) for n in common if not close(saved[n], current[n])]

    print("\n===== СВОДКА =====")
    print(f"COMMON={len(common)}")
    print(f"CHANGED={len(changed)}")
    print(f"MISSING_IN_NEW_FW={len(missing)}")
    print(f"NEW_IN_4_7_1={len(new)}")

    print("\n===== КРИТИЧНЫЕ JT-ZERO =====")
    print(f"{'PARAM':24s} {'OLD':>14s} {'CURRENT':>14s}  STATUS")
    for n in CRITICAL:
        ov = saved.get(n)
        cv = current.get(n)
        if ov is None and cv is None:
            status = "ABSENT"
        elif ov is None:
            status = "NEW"
        elif cv is None:
            status = "MISSING"
        elif close(ov, cv):
            status = "SAME"
        else:
            status = "DIFF"
        print(f"{n:24s} {fmt(ov):>14s} {fmt(cv):>14s}  {status}")

    print("\n===== ИЗМЕНЁННЫЕ ЗНАЧЕНИЯ =====")
    if not changed:
        print("нет")
    else:
        for n, ov, cv in changed:
            print(f"{n:28s} old={fmt(ov):>12s} current={fmt(cv):>12s}")

    print("\n===== ПАРАМЕТРЫ СТАРОГО ФАЙЛА, КОТОРЫХ НЕТ В НОВОЙ ПРОШИВКЕ =====")
    if not missing:
        print("нет")
    else:
        for n in missing:
            print(f"{n}={fmt(saved[n])}")

    if args.show_all_new:
        print("\n===== НОВЫЕ ПАРАМЕТРЫ НОВОЙ ПРОШИВКИ =====")
        for n in new:
            print(f"{n}={fmt(current[n])}")

    print("\n===== VERDICT =====")
    print("AUDIT_ONLY=YES")
    print("FC_PARAMS_WRITTEN=0")
    print("Следующий шаг: по этому отчёту сформировать whitelist восстановления, а не заливать старый .param вслепую.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
