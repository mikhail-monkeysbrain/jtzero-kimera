#!/usr/bin/env python3
# JT-Zero — проверка наличия EKF3 optical-flow fusion в реально прошитой FC.
#
# Через MAVFTP читает виртуальный @SYS/flash.bin из ArduPilot и ищет
# уникальные строковые литералы из NavEKF3 optical-flow path.
# SD-карта не используется. Параметры FC не изменяются.

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from pymavlink import mavutil, mavftp


def request_autopilot_version(master):
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0,
        )
        msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3.0)
        if msg is None:
            return None
        return msg
    except Exception:
        return None


def fmt_custom_version(v) -> str:
    try:
        b = bytes(v)
    except Exception:
        return "UNKNOWN"
    return b.hex()


def ftp_read_flash(master, max_bytes: int) -> bytes | None:
    ftp = mavftp.MAVFTP(
        master,
        target_system=master.target_system,
        target_component=master.target_component,
    )
    ftp.ftp_settings.debug = 0
    ftp.ftp_settings.burst_read_size = 239

    candidates = ["@SYS/flash.bin", "/@SYS/flash.bin"]
    for remote in candidates:
        print(f"MAVFTP: пробую {remote}, max_bytes={max_bytes}")
        try:
            data = ftp.read(remote, max_bytes, 0)
        except Exception as exc:
            print(f"MAVFTP: {remote}: exception: {exc}")
            data = None
        if data:
            print(f"MAVFTP: {remote}: получено {len(data)} bytes")
            return bytes(data)
        print(f"MAVFTP: {remote}: данных нет")
    return None


def contains(data: bytes, needle: bytes) -> bool:
    return data.find(needle) >= 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Проверка EKF3 OpticalFlow feature в прошивке FC")
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--max-bytes", type=int, default=2 * 1024 * 1024)
    ap.add_argument("--save", default="")
    args = ap.parse_args()

    print("=" * 70)
    print("JT-ZERO — FC FLASH EKF3 OPTFLOW FEATURE PROBE")
    print("=" * 70)
    print("SD-карта не используется. Параметры FC не изменяются.")
    print(f"FC: {args.device} @ {args.baud}")

    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=197,
        autoreconnect=False,
    )

    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        print("ОШИБКА: HEARTBEAT не получен")
        return 2
    print(f"FC heartbeat sys={master.target_system} comp={master.target_component}")

    ver = request_autopilot_version(master)
    if ver is not None:
        print("\n===== AUTOPILOT_VERSION =====")
        print(f"flight_sw_version={getattr(ver, 'flight_sw_version', 'UNKNOWN')}")
        print(f"middleware_sw_version={getattr(ver, 'middleware_sw_version', 'UNKNOWN')}")
        print(f"board_version={getattr(ver, 'board_version', 'UNKNOWN')}")
        print(f"flight_custom_version={fmt_custom_version(getattr(ver, 'flight_custom_version', b''))}")
    else:
        print("\nAUTOPILOT_VERSION: NO_DATA")

    print("\n===== MAVFTP FLASH READ =====")
    data = ftp_read_flash(master, args.max_bytes)
    if not data:
        print("ОШИБКА: не удалось прочитать @SYS/flash.bin")
        print("FLASH_READ=NO")
        return 3

    if args.save:
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(data)
        print(f"flash saved: {out}")

    probes = [
        ("EKF3_OPTFLOW_FUSION_TEXT", b"fusing optical flow"),
        ("EKF3_TILT_ALIGNMENT_TEXT", b"tilt alignment complete"),
        ("OPTICALFLOW_FRONTEND_TEXT", b"FlowCal:"),
        ("EKF3_PREFIX_TEXT", b"EKF3 IMU"),
    ]

    print("\n===== STRING PROBES =====")
    results = {}
    for name, needle in probes:
        yes = contains(data, needle)
        results[name] = yes
        off = data.find(needle)
        print(f"{name}={'YES' if yes else 'NO'}" + (f" offset=0x{off:x}" if yes else ""))

    print("\n===== VERDICT =====")
    if results["EKF3_OPTFLOW_FUSION_TEXT"]:
        print("EKF3_OPTFLOW_FUSION_COMPILED=YES")
        print("В реально прошитой FC присутствует строка из NavEKF3_core::FuseOptFlow().")
        print("Следовательно compile-time отключение EKF3 optical-flow fusion исключается.")
        print("Следующий подозреваемый — доставка measurement внутрь EKF3 core / flowMeaTime_ms.")
        return 0

    if results["EKF3_TILT_ALIGNMENT_TEXT"] and results["EKF3_PREFIX_TEXT"]:
        print("EKF3_OPTFLOW_FUSION_COMPILED=NO_OR_DIFFERENT_FIRMWARE")
        print("EKF3 строки в firmware есть, но уникальная строка FuseOptFlow отсутствует.")
        print("Это сильное указание, что EK3_FEATURE_OPTFLOW_FUSION выключен в этой сборке")
        print("или фактическая прошивка отличается от проверяемого исходника Copter-4.7.0.")
        return 4

    print("EKF3_OPTFLOW_FUSION_COMPILED=UNKNOWN")
    print("Контрольные EKF3 строки тоже не найдены; flash dump недостаточен или формат firmware отличается.")
    return 5


if __name__ == "__main__":
    raise SystemExit(main())
