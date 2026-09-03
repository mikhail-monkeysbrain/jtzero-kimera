#!/usr/bin/env python3
import argparse
import csv
import math
import queue
import statistics
import threading
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import messagebox
from pymavlink import mavutil

PRE_S = 15.0
YAW_S = 10.0
POST_S = 15.0
TOTAL_S = PRE_S + YAW_S + POST_S
BASELINE_DXY = 0.00202

WANTED = [
    "RAW_IMU",
    "SCALED_IMU",
    "SCALED_IMU2",
    "SCALED_IMU3",
    "HIGHRES_IMU",
    "ATTITUDE",
    "ATTITUDE_QUATERNION",
    "VIBRATION",
]

MSG_IDS = {
    "RAW_IMU": 27,
    "SCALED_IMU": 26,
    "SCALED_IMU2": 116,
    "SCALED_IMU3": 129,
    "HIGHRES_IMU": 105,
    "ATTITUDE": 30,
    "ATTITUDE_QUATERNION": 31,
    "VIBRATION": 241,
}

GYRO_STREAMS = {
    "SCALED_IMU",
    "SCALED_IMU2",
    "SCALED_IMU3",
    "HIGHRES_IMU",
    "ATTITUDE",
}


def phase_for(t):
    if t < PRE_S:
        return "PRE"
    if t < PRE_S + YAW_S:
        return "YAW"
    return "POST"


def block_for(t):
    if 0 <= t < 5:
        return "PRE_EARLY"
    if 5 <= t < 10:
        return "PRE_MID"
    if 10 <= t < 15:
        return "PRE_LATE"
    if 25 <= t < 30:
        return "POST_EARLY"
    if 30 <= t < 35:
        return "POST_MID"
    if 35 <= t <= 40.5:
        return "POST_LATE"
    return ""


def gyro_rad(msg):
    typ = msg.get_type()
    if typ == "HIGHRES_IMU":
        return float(msg.xgyro), float(msg.ygyro), float(msg.zgyro)
    if typ in ("SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
        return (
            float(msg.xgyro) * 1e-3,
            float(msg.ygyro) * 1e-3,
            float(msg.zgyro) * 1e-3,
        )
    if typ == "ATTITUDE":
        return float(msg.rollspeed), float(msg.pitchspeed), float(msg.yawspeed)
    return None


def mean(values):
    return statistics.fmean(values) if values else float("nan")


def sd(values):
    return statistics.stdev(values) if len(values) >= 2 else float("nan")


def request_interval(mav, msgid, hz):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msgid,
        1e6 / hz,
        0,
        0,
        0,
        0,
        0,
    )


def analyze(rows):
    by_type = defaultdict(list)
    for row in rows:
        if row["gx"] is not None:
            by_type[row["type"]].append(row)

    result = {}
    for typ, stream_rows in by_type.items():
        pre = [r for r in stream_rows if r["phase"] == "PRE"]
        post = [r for r in stream_rows if r["phase"] == "POST"]
        if not pre or not post:
            continue

        d = {
            "n_pre": len(pre),
            "n_post": len(post),
            "n_total": len(stream_rows),
        }

        for axis in "xyz":
            key = "g" + axis
            pre_values = [r[key] for r in pre]
            post_values = [r[key] for r in post]
            pre_mean = mean(pre_values)
            post_mean = mean(post_values)
            d["pre_" + axis] = pre_mean
            d["post_" + axis] = post_mean
            d["delta_" + axis] = post_mean - pre_mean
            d["pre_sd_" + axis] = sd(pre_values)
            d["post_sd_" + axis] = sd(post_values)

        d["delta_xy"] = math.hypot(d["delta_x"], d["delta_y"])
        d["delta_xyz"] = math.sqrt(
            d["delta_x"] ** 2 + d["delta_y"] ** 2 + d["delta_z"] ** 2
        )
        d["ratio"] = d["delta_xy"] / BASELINE_DXY

        blocks = {}
        for block_name in (
            "PRE_EARLY",
            "PRE_MID",
            "PRE_LATE",
            "POST_EARLY",
            "POST_MID",
            "POST_LATE",
        ):
            block_rows = [r for r in stream_rows if r["block"] == block_name]
            if block_rows:
                blocks[block_name] = tuple(
                    mean([r["g" + axis] for r in block_rows]) for axis in "xyz"
                )
        d["blocks"] = blocks
        result[typ] = d

    return result


def save(rows, result, out_dir, port):
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"matek_h743_ardupilot_imu_path_{stamp}.csv"
    txt_path = out_dir / f"matek_h743_ardupilot_imu_path_{stamp}.txt"

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "host_t",
                "t_rel",
                "phase",
                "block",
                "type",
                "fc_time",
                "gx_rad_s",
                "gy_rad_s",
                "gz_rad_s",
                "raw_xgyro",
                "raw_ygyro",
                "raw_zgyro",
                "roll",
                "pitch",
                "yaw",
                "q1",
                "q2",
                "q3",
                "q4",
                "vibration_x",
                "vibration_y",
                "vibration_z",
                "clipping_0",
                "clipping_1",
                "clipping_2",
            ]
        )
        for r in rows:
            writer.writerow(
                [
                    f"{r['host_t']:.9f}",
                    f"{r['t']:.6f}",
                    r["phase"],
                    r["block"],
                    r["type"],
                    r["fc_time"],
                    "" if r["gx"] is None else f"{r['gx']:.12f}",
                    "" if r["gy"] is None else f"{r['gy']:.12f}",
                    "" if r["gz"] is None else f"{r['gz']:.12f}",
                    r["raw_x"],
                    r["raw_y"],
                    r["raw_z"],
                    r["roll"],
                    r["pitch"],
                    r["yaw"],
                    r["q1"],
                    r["q2"],
                    r["q3"],
                    r["q4"],
                    r["vibration_x"],
                    r["vibration_y"],
                    r["vibration_z"],
                    r["clipping_0"],
                    r["clipping_1"],
                    r["clipping_2"],
                ]
            )

    counts = defaultdict(int)
    for row in rows:
        counts[row["type"]] += 1

    with txt_path.open("w") as f:
        f.write("JT-Zero P11/B11 ArduPilot IMU path test\n")
        f.write("FC: Matek H743-SLIM V3 / STM32H743 / ArduPilot\n")
        f.write(f"MAVLink USB port: {port}\n")
        f.write("Sequence: 15 s STILL PRE -> ~90 deg YAW -> 15 s STILL POST\n")
        f.write(
            "Streams requested: RAW_IMU, SCALED_IMU, SCALED_IMU2, "
            "SCALED_IMU3, HIGHRES_IMU, ATTITUDE, ATTITUDE_QUATERNION, VIBRATION\n"
        )
        f.write(
            "Gyro units used for comparison: HIGHRES_IMU/ATTITUDE rad/s; "
            "SCALED_IMU* mrad/s converted to rad/s. RAW_IMU is saved separately "
            "without assuming a conversion in this diagnostic.\n"
        )
        f.write(f"Reference problematic baseline |DELTA gyro XY| ~= {BASELINE_DXY:.9f} rad/s\n\n")

        f.write("Message counts:\n")
        for name in WANTED:
            f.write(f"  {name}: {counts[name]}\n")
        f.write("\n")

        for typ in sorted(result):
            d = result[typ]
            f.write(
                f"[{typ}] PRE={d['n_pre']} POST={d['n_post']} TOTAL={d['n_total']}\n"
            )
            for axis in "xyz":
                f.write(
                    f"  {axis.upper()}: PRE={d['pre_' + axis]:+.9f} "
                    f"POST={d['post_' + axis]:+.9f} "
                    f"DELTA={d['delta_' + axis]:+.9f} rad/s "
                    f"SDpre={d['pre_sd_' + axis]:.9f} "
                    f"SDpost={d['post_sd_' + axis]:.9f}\n"
                )
            f.write(
                f"  |DELTA XY|={d['delta_xy']:.9f} rad/s  "
                f"|DELTA XYZ|={d['delta_xyz']:.9f} rad/s  "
                f"ratio_to_problem_baseline={d['ratio']:.3f}\n"
            )
            for block_name, vals in d["blocks"].items():
                f.write(
                    f"  {block_name}: X={vals[0]:+.9f} "
                    f"Y={vals[1]:+.9f} Z={vals[2]:+.9f}\n"
                )
            f.write("\n")

    return csv_path, txt_path


class App:
    def __init__(self, root, port, baud):
        self.root = root
        self.port = port
        self.baud = baud
        self.mav = None
        self.rows = []
        self.running = False
        self.start_t = None
        self.last_att = None
        self.yaw_reference = None
        self.events = queue.Queue()
        self.seen = defaultdict(int)

        root.title("JT-Zero — Matek H743 IMU path test")
        root.geometry("1080x760")

        tk.Label(
            root,
            text="Matek H743-SLIM V3 / ArduPilot — IMU A/B",
            font=("DejaVu Sans", 20, "bold"),
        ).pack(pady=(18, 8))

        self.status = tk.Label(root, text="Готово", font=("DejaVu Sans", 26, "bold"))
        self.status.pack(pady=8)

        self.instr = tk.Label(
            root,
            text="Положите полётник неподвижно. Поворот будет только по YAW.",
            font=("DejaVu Sans", 16),
        )
        self.instr.pack(pady=4)

        self.timer = tk.Label(root, text="00.0 с", font=("DejaVu Sans Mono", 44, "bold"))
        self.timer.pack(pady=10)

        self.att = tk.Label(root, text="ATTITUDE: ---", font=("DejaVu Sans Mono", 16))
        self.att.pack(pady=6)

        self.yaw_target = tk.Label(
            root,
            text="YAW относительно старта: ---   Цель: 90° ±15°",
            font=("DejaVu Sans Mono", 16, "bold"),
        )
        self.yaw_target.pack(pady=6)

        self.streams = tk.Label(
            root,
            text="Потоки: ---",
            font=("DejaVu Sans Mono", 12),
            justify="left",
            wraplength=1000,
        )
        self.streams.pack(pady=8)

        self.btn = tk.Button(
            root,
            text="НАЧАТЬ ТЕСТ",
            font=("DejaVu Sans", 16, "bold"),
            width=22,
            command=self.start,
        )
        self.btn.pack(pady=14)

        self.result = tk.Label(
            root,
            text="",
            font=("DejaVu Sans Mono", 12),
            justify="left",
            anchor="w",
        )
        self.result.pack(fill="x", padx=45, pady=8)

        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.tick)

    def connect(self):
        self.mav = mavutil.mavlink_connection(
            self.port,
            baud=self.baud,
            autoreconnect=False,
        )
        hb = self.mav.wait_heartbeat(timeout=5)
        if hb is None:
            raise RuntimeError("HEARTBEAT timeout")

        for name in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "HIGHRES_IMU"):
            request_interval(self.mav, MSG_IDS[name], 50)
        for name in ("ATTITUDE", "ATTITUDE_QUATERNION", "VIBRATION"):
            request_interval(self.mav, MSG_IDS[name], 20)

        return hb.get_srcSystem(), hb.get_srcComponent()

    def start(self):
        if self.running:
            return

        try:
            sysid, compid = self.connect()
        except Exception as exc:
            messagebox.showerror("Ошибка MAVLink", str(exc))
            return

        self.rows = []
        self.seen = defaultdict(int)
        self.start_t = time.monotonic()
        self.running = True
        self.last_att = None
        self.yaw_reference = None
        self.btn.config(state="disabled")
        self.result.config(text=f"MAVLink подключён: SYS={sysid} COMP={compid} PORT={self.port}")
        threading.Thread(target=self.collect, daemon=True).start()

    def collect(self):
        try:
            while self.running:
                now = time.monotonic()
                t = now - self.start_t
                if t >= TOTAL_S:
                    break

                msg = self.mav.recv_match(type=WANTED, blocking=True, timeout=0.25)
                if msg is None:
                    continue

                typ = msg.get_type()
                self.seen[typ] += 1
                gyro = gyro_rad(msg)
                gx = gy = gz = None
                if gyro:
                    gx, gy, gz = gyro

                raw_x = getattr(msg, "xgyro", "") if typ == "RAW_IMU" else ""
                raw_y = getattr(msg, "ygyro", "") if typ == "RAW_IMU" else ""
                raw_z = getattr(msg, "zgyro", "") if typ == "RAW_IMU" else ""
                fc_time = getattr(msg, "time_usec", getattr(msg, "time_boot_ms", ""))

                roll = pitch = yaw = ""
                q1 = q2 = q3 = q4 = ""
                vibration_x = vibration_y = vibration_z = ""
                clipping_0 = clipping_1 = clipping_2 = ""

                if typ == "ATTITUDE":
                    roll = float(msg.roll)
                    pitch = float(msg.pitch)
                    yaw = float(msg.yaw)
                    self.last_att = (roll, pitch, yaw)
                    if self.yaw_reference is None and t >= 2.0:
                        self.yaw_reference = yaw

                elif typ == "ATTITUDE_QUATERNION":
                    q1 = float(msg.q1)
                    q2 = float(msg.q2)
                    q3 = float(msg.q3)
                    q4 = float(msg.q4)

                elif typ == "VIBRATION":
                    vibration_x = float(msg.vibration_x)
                    vibration_y = float(msg.vibration_y)
                    vibration_z = float(msg.vibration_z)
                    clipping_0 = int(msg.clipping_0)
                    clipping_1 = int(msg.clipping_1)
                    clipping_2 = int(msg.clipping_2)

                self.rows.append(
                    dict(
                        host_t=now,
                        t=t,
                        phase=phase_for(t),
                        block=block_for(t),
                        type=typ,
                        fc_time=fc_time,
                        gx=gx,
                        gy=gy,
                        gz=gz,
                        raw_x=raw_x,
                        raw_y=raw_y,
                        raw_z=raw_z,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        q1=q1,
                        q2=q2,
                        q3=q3,
                        q4=q4,
                        vibration_x=vibration_x,
                        vibration_y=vibration_y,
                        vibration_z=vibration_z,
                        clipping_0=clipping_0,
                        clipping_1=clipping_1,
                        clipping_2=clipping_2,
                    )
                )
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.running = False
            try:
                if self.mav:
                    self.mav.close()
            except Exception:
                pass
            self.events.put(("done", None))

    @staticmethod
    def wrap_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def tick(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "error":
                    messagebox.showerror("Ошибка", data)
                elif kind == "done" and self.start_t is not None:
                    self.finish()
        except queue.Empty:
            pass

        if self.running:
            t = min(time.monotonic() - self.start_t, TOTAL_S)
            phase = phase_for(min(t, TOTAL_S - 1e-6))

            if phase == "PRE":
                self.status.config(text="ПОКОЙ PRE")
                self.instr.config(text="НЕ ДВИГАТЬ полётник")
                remaining = PRE_S - t
            elif phase == "YAW":
                self.status.config(text="ПОВОРОТ YAW ~90°")
                self.instr.config(text="Плавно поверните только по YAW примерно на 90° и оставьте в новом положении")
                remaining = PRE_S + YAW_S - t
            else:
                self.status.config(text="ПОКОЙ POST")
                self.instr.config(text="НЕ ДВИГАТЬ полётник")
                remaining = TOTAL_S - t

            self.timer.config(text=f"{max(0, remaining):04.1f} с")

            if self.last_att:
                roll, pitch, yaw = self.last_att
                self.att.config(
                    text=(
                        f"ATTITUDE deg: R {math.degrees(roll):+.2f}  "
                        f"P {math.degrees(pitch):+.2f}  Y {math.degrees(yaw):+.2f}"
                    )
                )
                if self.yaw_reference is not None:
                    dyaw = math.degrees(self.wrap_pi(yaw - self.yaw_reference))
                    abs_dyaw = abs(dyaw)
                    if 75.0 <= abs_dyaw <= 105.0:
                        target_text = "В ЦЕЛЕВОЙ ЗОНЕ"
                    elif abs_dyaw < 75.0:
                        target_text = "ДОВЕРНУТЬ"
                    else:
                        target_text = "СЛИШКОМ ДАЛЕКО"
                    self.yaw_target.config(
                        text=f"YAW относительно старта: {dyaw:+.1f}°   Цель: ±90° (75…105°)   {target_text}"
                    )

            self.streams.config(
                text="Потоки: " + "  ".join(f"{name}:{self.seen[name]}" for name in WANTED)
            )

        self.root.after(50, self.tick)

    def finish(self):
        if not self.rows:
            self.btn.config(state="normal")
            return

        result = analyze(self.rows)
        out_dir = Path(__file__).resolve().parent / "results"
        csv_path, txt_path = save(self.rows, result, out_dir, self.port)

        lines = ["ТЕСТ ЗАВЕРШЁН", f"TXT: {txt_path.name}"]
        for typ in sorted(result):
            d = result[typ]
            lines.append(
                f"{typ}: |ΔXY|={d['delta_xy']:.7f} rad/s ({d['ratio']:.3f}x problem baseline)"
            )

        self.status.config(text="ТЕСТ ЗАВЕРШЁН")
        self.instr.config(text="Результат сохранён. Полётник можно двигать.")
        self.timer.config(text="00.0 с")
        self.result.config(text="\n".join(lines))
        self.btn.config(state="normal")
        self.start_t = None

    def close(self):
        self.running = False
        try:
            if self.mav:
                self.mav.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    root = tk.Tk()
    App(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
