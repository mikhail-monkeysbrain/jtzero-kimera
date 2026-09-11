#!/usr/bin/env python3
# JT-Zero — безопасное чтение/запись одного параметра ArduPilot через MAVLink.
# Ищет именно HEARTBEAT MAV_AUTOPILOT_ARDUPILOTMEGA, после PARAM_SET делает read-back.

from __future__ import annotations

import argparse
import math
import sys
import time

from pymavlink import mavutil


def wait_fc(master, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if m is None:
            continue
        if int(getattr(m, "autopilot", -1)) == int(mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            return int(m.get_srcSystem()), int(m.get_srcComponent())
    return None


def param_name(m) -> str:
    p = m.param_id
    if isinstance(p, bytes):
        p = p.decode("ascii", errors="ignore")
    return str(p).rstrip("\x00")


def read_param(master, sysid: int, compid: int, name: str, timeout: float = 3.0):
    master.mav.param_request_read_send(sysid, compid, name.encode("ascii"), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if m is None:
            continue
        if int(m.get_srcSystem()) != sysid:
            continue
        if param_name(m) == name:
            return float(m.param_value), int(m.param_type)
    return None


def set_param(master, sysid: int, compid: int, name: str, value: float, ptype: int):
    master.mav.param_set_send(sysid, compid, name.encode("ascii"), float(value), ptype)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("value", nargs="?", type=float)
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--value-only", action="store_true")
    args = ap.parse_args()

    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=199,
        autoreconnect=False,
    )
    fc = wait_fc(master)
    if fc is None:
        print("ОШИБКА: HEARTBEAT ArduPilot не найден", file=sys.stderr)
        return 2
    sysid, compid = fc

    cur = read_param(master, sysid, compid, args.name)
    if cur is None:
        print(f"ОШИБКА: параметр {args.name} не прочитан", file=sys.stderr)
        return 3
    old, ptype = cur

    if args.value is None:
        if args.value_only:
            print(f"{old:.9g}")
        else:
            print(f"FC sys={sysid} comp={compid}")
            print(f"{args.name}={old:.9g}")
        return 0

    set_param(master, sysid, compid, args.name, args.value, ptype)
    deadline = time.monotonic() + 4.0
    got = None
    while time.monotonic() < deadline:
        r = read_param(master, sysid, compid, args.name, timeout=0.8)
        if r is not None:
            got = r[0]
            if math.isclose(got, args.value, rel_tol=0.0, abs_tol=max(1e-6, abs(args.value)*1e-5)):
                break

    if got is None or not math.isclose(got, args.value, rel_tol=0.0, abs_tol=max(1e-6, abs(args.value)*1e-5)):
        print(f"ОШИБКА: read-back {args.name}={got}, ожидалось {args.value}", file=sys.stderr)
        return 4

    if args.value_only:
        print(f"{got:.9g}")
    else:
        print(f"FC sys={sysid} comp={compid}")
        print(f"{args.name}: {old:.9g} -> {got:.9g} VERIFIED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
