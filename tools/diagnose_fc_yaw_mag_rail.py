#!/usr/bin/env python3
"""Пассивная диагностика yaw/компаса FC при движении стенда по рельсе.

Ground Motion должен быть остановлен: скрипт сам открывает /dev/ttyAMA0.
Он ничего не меняет в параметрах FC и не переключает EKF source set.

Записывает ATTITUDE + RAW_IMU + AHRS + EKF_STATUS_REPORT в один CSV и
помечает этапы STATIC_A, MOVE_AB, STATIC_B, MOVE_BA, STATIC_A2.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import threading
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError as e:
    raise SystemExit(
        "ОШИБКА: pymavlink не установлен в этом Python. "
        "Запустите через ~/venv-jtzero-mav/bin/python или активируйте venv-jtzero-mav."
    ) from e


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stage = "BOOT"
        self.running = True
        self.mag = None
        self.ahrs = None
        self.ekf = None
        self.rows = []


def request_rate(master, target_sys: int, target_comp: int, msgid: int, hz: float) -> None:
    interval_us = int(round(1_000_000.0 / hz))
    master.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msgid,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def safe(v, default=float("nan")):
    try:
        return float(v)
    except Exception:
        return default


def receiver(master, st: State, writer, csv_file) -> None:
    while st.running:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        typ = msg.get_type()
        now_ns = time.monotonic_ns()

        if typ == "RAW_IMU":
            with st.lock:
                st.mag = {
                    "xmag": safe(getattr(msg, "xmag", 0.0)),
                    "ymag": safe(getattr(msg, "ymag", 0.0)),
                    "zmag": safe(getattr(msg, "zmag", 0.0)),
                    "xacc": safe(getattr(msg, "xacc", 0.0)),
                    "yacc": safe(getattr(msg, "yacc", 0.0)),
                    "zacc": safe(getattr(msg, "zacc", 0.0)),
                    "xgyro": safe(getattr(msg, "xgyro", 0.0)),
                    "ygyro": safe(getattr(msg, "ygyro", 0.0)),
                    "zgyro": safe(getattr(msg, "zgyro", 0.0)),
                    "recv_ns": now_ns,
                }
            continue

        if typ == "AHRS":
            with st.lock:
                st.ahrs = {
                    "error_rp": safe(getattr(msg, "error_rp", float("nan"))),
                    "error_yaw": safe(getattr(msg, "error_yaw", float("nan"))),
                    "recv_ns": now_ns,
                }
            continue

        if typ == "EKF_STATUS_REPORT":
            with st.lock:
                st.ekf = {
                    "velocity_variance": safe(getattr(msg, "velocity_variance", float("nan"))),
                    "pos_horiz_variance": safe(getattr(msg, "pos_horiz_variance", float("nan"))),
                    "pos_vert_variance": safe(getattr(msg, "pos_vert_variance", float("nan"))),
                    "compass_variance": safe(getattr(msg, "compass_variance", float("nan"))),
                    "terrain_alt_variance": safe(getattr(msg, "terrain_alt_variance", float("nan"))),
                    "flags": int(getattr(msg, "flags", 0)),
                    "recv_ns": now_ns,
                }
            continue

        if typ != "ATTITUDE":
            continue

        with st.lock:
            stage = st.stage
            mag = dict(st.mag) if st.mag else {}
            ahrs = dict(st.ahrs) if st.ahrs else {}
            ekf = dict(st.ekf) if st.ekf else {}

        mx = safe(mag.get("xmag"))
        my = safe(mag.get("ymag"))
        mz = safe(mag.get("zmag"))
        mag_norm = math.sqrt(mx * mx + my * my + mz * mz) if all(math.isfinite(v) for v in (mx, my, mz)) else float("nan")
        mag_age_ms = (now_ns - int(mag.get("recv_ns", now_ns))) * 1e-6 if mag else float("nan")
        ahrs_age_ms = (now_ns - int(ahrs.get("recv_ns", now_ns))) * 1e-6 if ahrs else float("nan")
        ekf_age_ms = (now_ns - int(ekf.get("recv_ns", now_ns))) * 1e-6 if ekf else float("nan")

        row = {
            "mono_ns": now_ns,
            "stage": stage,
            "roll_deg": math.degrees(safe(getattr(msg, "roll", 0.0))),
            "pitch_deg": math.degrees(safe(getattr(msg, "pitch", 0.0))),
            "yaw_deg": math.degrees(safe(getattr(msg, "yaw", 0.0))),
            "rollspeed_dps": math.degrees(safe(getattr(msg, "rollspeed", 0.0))),
            "pitchspeed_dps": math.degrees(safe(getattr(msg, "pitchspeed", 0.0))),
            "yawspeed_dps": math.degrees(safe(getattr(msg, "yawspeed", 0.0))),
            "xmag": mx,
            "ymag": my,
            "zmag": mz,
            "mag_norm": mag_norm,
            "mag_age_ms": mag_age_ms,
            "xacc": safe(mag.get("xacc")),
            "yacc": safe(mag.get("yacc")),
            "zacc": safe(mag.get("zacc")),
            "xgyro": safe(mag.get("xgyro")),
            "ygyro": safe(mag.get("ygyro")),
            "zgyro": safe(mag.get("zgyro")),
            "ahrs_error_rp": safe(ahrs.get("error_rp")),
            "ahrs_error_yaw": safe(ahrs.get("error_yaw")),
            "ahrs_age_ms": ahrs_age_ms,
            "ekf_velocity_variance": safe(ekf.get("velocity_variance")),
            "ekf_pos_horiz_variance": safe(ekf.get("pos_horiz_variance")),
            "ekf_pos_vert_variance": safe(ekf.get("pos_vert_variance")),
            "ekf_compass_variance": safe(ekf.get("compass_variance")),
            "ekf_terrain_alt_variance": safe(ekf.get("terrain_alt_variance")),
            "ekf_flags": int(ekf.get("flags", 0)) if ekf else 0,
            "ekf_age_ms": ekf_age_ms,
        }
        writer.writerow(row)
        csv_file.flush()
        with st.lock:
            st.rows.append(row)


def unwrap_deg(vals):
    if not vals:
        return []
    out = [vals[0]]
    for v in vals[1:]:
        while v - out[-1] > 180.0:
            v -= 360.0
        while v - out[-1] < -180.0:
            v += 360.0
        out.append(v)
    return out


def finite_vals(rows, key):
    out = []
    for r in rows:
        v = safe(r.get(key))
        if math.isfinite(v):
            out.append(v)
    return out


def mean_std(vals):
    if not vals:
        return float("nan"), float("nan")
    return statistics.fmean(vals), statistics.stdev(vals) if len(vals) > 1 else 0.0


def print_summary(rows) -> None:
    print("\n===== YAW/MAG СВОДКА =====")
    stages = ["STATIC_A", "MOVE_AB", "STATIC_B", "MOVE_BA", "STATIC_A2"]
    for stage in stages:
        rr = [r for r in rows if r["stage"] == stage]
        if not rr:
            continue
        yaw = unwrap_deg(finite_vals(rr, "yaw_deg"))
        roll = finite_vals(rr, "roll_deg")
        pitch = finite_vals(rr, "pitch_deg")
        mn = finite_vals(rr, "mag_norm")
        mx = finite_vals(rr, "xmag")
        my = finite_vals(rr, "ymag")
        mz = finite_vals(rr, "zmag")
        cv = finite_vals(rr, "ekf_compass_variance")
        ey = finite_vals(rr, "ahrs_error_yaw")
        ym, ys = mean_std(yaw)
        rm, rs = mean_std(roll)
        pm, ps = mean_std(pitch)
        bm, bs = mean_std(mn)
        xm, _ = mean_std(mx)
        yym, _ = mean_std(my)
        zm, _ = mean_std(mz)
        cvm, cvs = mean_std(cv)
        eym, eys = mean_std(ey)
        print(
            f"{stage:9s} n={len(rr):4d} "
            f"yaw={ym:+8.3f}±{ys:.3f} deg  roll={rm:+7.3f}±{rs:.3f}  pitch={pm:+7.3f}±{ps:.3f}"
        )
        print(
            f"           MAG=({xm:+8.1f},{yym:+8.1f},{zm:+8.1f}) |B|={bm:8.2f}±{bs:.2f}  "
            f"EKF compass_var={cvm:.4f}±{cvs:.4f}  AHRS err_yaw={eym:.4f}±{eys:.4f}"
        )

    a = [r for r in rows if r["stage"] == "STATIC_A"]
    b = [r for r in rows if r["stage"] == "STATIC_B"]
    a2 = [r for r in rows if r["stage"] == "STATIC_A2"]
    if a and b:
        ba, _ = mean_std(finite_vals(a, "mag_norm"))
        bb, _ = mean_std(finite_vals(b, "mag_norm"))
        print(f"\nSTATIC B - STATIC A: delta |B|={bb-ba:+.2f} raw-mag units")
    if a and a2:
        ba, _ = mean_std(finite_vals(a, "mag_norm"))
        ba2, _ = mean_std(finite_vals(a2, "mag_norm"))
        print(f"STATIC A2 - STATIC A: delta |B|={ba2-ba:+.2f} raw-mag units")


def set_stage(st: State, stage: str) -> None:
    with st.lock:
        st.stage = stage
    print(f"[STAGE] {stage}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Диагностика FC yaw + RM3100 на рельсе")
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    if args.csv is None:
        outdir = Path("/home/vio/jtzero_runs")
        outdir.mkdir(parents=True, exist_ok=True)
        args.csv = outdir / (time.strftime("%Y%m%d_%H%M%S") + "_FC_YAW_MAG_RAIL.csv")

    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=197,
        autoreconnect=False,
    )
    print(f"Жду HEARTBEAT FC на {args.device} @ {args.baud} ...")
    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit("ОШИБКА: HEARTBEAT timeout")
    target_sys = master.target_system
    target_comp = master.target_component
    print(f"FC heartbeat sys={target_sys} comp={target_comp}")

    request_rate(master, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
    request_rate(master, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 20)
    request_rate(master, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_AHRS, 10)
    request_rate(master, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 10)

    fields = [
        "mono_ns", "stage", "roll_deg", "pitch_deg", "yaw_deg",
        "rollspeed_dps", "pitchspeed_dps", "yawspeed_dps",
        "xmag", "ymag", "zmag", "mag_norm", "mag_age_ms",
        "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro",
        "ahrs_error_rp", "ahrs_error_yaw", "ahrs_age_ms",
        "ekf_velocity_variance", "ekf_pos_horiz_variance", "ekf_pos_vert_variance",
        "ekf_compass_variance", "ekf_terrain_alt_variance", "ekf_flags", "ekf_age_ms",
    ]

    st = State()
    with args.csv.open("w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields)
        wr.writeheader()
        th = threading.Thread(target=receiver, args=(master, st, wr, f), daemon=True)
        th.start()

        print(f"CSV: {args.csv}")
        print("Ground Motion НЕ запускать параллельно с этой диагностикой.")
        time.sleep(2.0)

        input("\nУстановите стенд в ограничитель A и нажмите Enter: ")
        set_stage(st, "STATIC_A")
        print("Статика A: 8 секунд, не двигать...")
        time.sleep(8.0)

        input("Нажмите Enter и сразу начинайте движение A -> B: ")
        set_stage(st, "MOVE_AB")
        input("Когда стенд упрётся в ограничитель B, нажмите Enter: ")
        set_stage(st, "STATIC_B")
        print("Статика B: 8 секунд, не двигать...")
        time.sleep(8.0)

        input("Нажмите Enter и сразу начинайте движение B -> A: ")
        set_stage(st, "MOVE_BA")
        input("Когда стенд упрётся в ограничитель A, нажмите Enter: ")
        set_stage(st, "STATIC_A2")
        print("Статика A после возврата: 8 секунд, не двигать...")
        time.sleep(8.0)

        set_stage(st, "DONE")
        time.sleep(0.5)
        st.running = False
        th.join(timeout=1.0)

    print_summary(st.rows)
    print(f"\nГОТОВО. CSV: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
