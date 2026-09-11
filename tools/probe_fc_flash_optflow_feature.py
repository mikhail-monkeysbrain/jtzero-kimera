#!/usr/bin/env python3
# JT-Zero — проверка наличия EKF3 optical-flow fusion в реально прошитой FC.
#
# Через MAVFTP читает виртуальный @SYS/flash.bin из ArduPilot и ищет
# уникальные строковые литералы из NavEKF3 optical-flow path.
# SD-карта не используется. Параметры FC не изменяются.

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

from pymavlink import mavutil, mavftp


def request_autopilot_version(master, target_system: int, target_component: int):
    try:
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0,
        )
        return master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3.0)
    except Exception:
        return None


def custom_version_bytes(v) -> bytes:
    try:
        return bytes(v)
    except Exception:
        return b""


def fmt_custom_version(v) -> str:
    return custom_version_bytes(v).hex() or "UNKNOWN"


def ascii_custom_version(v) -> str:
    b = custom_version_bytes(v)
    if not b:
        return "UNKNOWN"
    try:
        s = b.decode("ascii", errors="strict").rstrip("\x00")
    except Exception:
        return "NON_ASCII"
    return s if s else "EMPTY"


def _ret_code(ret):
    if ret is None:
        return None
    if hasattr(ret, "return_code"):
        try:
            return int(ret.return_code)
        except Exception:
            pass
    if hasattr(ret, "error_code"):
        try:
            return int(ret.error_code)
        except Exception:
            pass
    return None


def ftp_read_flash(master, target_system: int, target_component: int, max_bytes: int) -> bytes | None:
    # Не используем MAVFTP.read(): в старых версиях pymavlink эта функция
    # имеет несовместимую семантику и может пытаться открыть remote path
    # как локальный файл. cmd_get() поддерживается существенно дольше.
    candidates = ["@SYS/flash.bin", "/@SYS/flash.bin"]

    for remote in candidates:
        fd, local = tempfile.mkstemp(prefix="jtzero_fc_flash_", suffix=".bin")
        os.close(fd)
        try:
            os.unlink(local)
        except FileNotFoundError:
            pass

        print(f"MAVFTP: пробую {remote}")
        try:
            ftp = mavftp.MAVFTP(
                master,
                target_system=target_system,
                target_component=target_component,
            )
            ftp.ftp_settings.debug = 0
            try:
                ftp.ftp_settings.burst_read_size = 239
            except Exception:
                pass

            start_ret = ftp.cmd_get([remote, local])
            start_code = _ret_code(start_ret)
            if start_code not in (None, 0):
                print(f"MAVFTP: {remote}: cmd_get error={start_code}")
                continue

            # Современный pymavlink называет ожидание полной операции Get.
            # Для старых версий fallback на OpenFileRO безопасен: если Get уже
            # завершился, файл просто будет существовать и будет прочитан ниже.
            try:
                ret = ftp.process_ftp_reply("Get", timeout=120)
                code = _ret_code(ret)
                if code not in (None, 0):
                    print(f"MAVFTP: {remote}: Get error={code}")
            except Exception as exc_get:
                print(f"MAVFTP: {remote}: Get wait exception: {exc_get}")
                try:
                    ret = ftp.process_ftp_reply("OpenFileRO", timeout=120)
                    code = _ret_code(ret)
                    if code not in (None, 0):
                        print(f"MAVFTP: {remote}: OpenFileRO error={code}")
                except Exception as exc_old:
                    print(f"MAVFTP: {remote}: fallback exception: {exc_old}")

            if os.path.exists(local):
                size = os.path.getsize(local)
                if size > 0:
                    with open(local, "rb") as f:
                        data = f.read(max_bytes)
                    print(f"MAVFTP: {remote}: получено {len(data)} bytes (remote/local size >= {size})")
                    return data

            print(f"MAVFTP: {remote}: данных нет")
        except Exception as exc:
            print(f"MAVFTP: {remote}: exception: {exc}")
        finally:
            try:
                os.unlink(local)
            except FileNotFoundError:
                pass

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

    # В pymavlink target_component может оставаться 0 даже после wait_heartbeat().
    # Для MAVFTP берём реальные source ids из самого HEARTBEAT.
    target_system = int(hb.get_srcSystem())
    target_component = int(hb.get_srcComponent())
    master.target_system = target_system
    master.target_component = target_component
    print(f"FC heartbeat sys={target_system} comp={target_component}")

    ver = request_autopilot_version(master, target_system, target_component)
    if ver is not None:
        custom = getattr(ver, "flight_custom_version", b"")
        print("\n===== AUTOPILOT_VERSION =====")
        print(f"flight_sw_version={getattr(ver, 'flight_sw_version', 'UNKNOWN')}")
        print(f"middleware_sw_version={getattr(ver, 'middleware_sw_version', 'UNKNOWN')}")
        print(f"board_version={getattr(ver, 'board_version', 'UNKNOWN')}")
        print(f"flight_custom_version={fmt_custom_version(custom)}")
        print(f"flight_git_hash_ascii={ascii_custom_version(custom)}")
    else:
        print("\nAUTOPILOT_VERSION: NO_DATA")

    print("\n===== MAVFTP FLASH READ =====")
    data = ftp_read_flash(master, target_system, target_component, args.max_bytes)
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
