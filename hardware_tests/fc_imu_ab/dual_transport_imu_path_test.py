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


def phase_for(t):
    if t < PRE_S:
        return "PRE"
    if t < PRE_S + YAW_S:
        return "YAW"
    return "POST"


def block_for(t):
    if 0.0 <= t < 5.0:
        return "PRE_EARLY"
    if 5.0 <= t < 10.0:
        return "PRE_MID"
    if 10.0 <= t < 15.0:
        return "PRE_LATE"
    if 25.0 <= t < 30.0:
        return "POST_EARLY"
    if 30.0 <= t < 35.0:
        return "POST_MID"
    if 35.0 <= t <= 40.5:
        return "POST_LATE"
    return ""


def mean(v):
    return statistics.fmean(v) if v else float("nan")


def sd(v):
    return statistics.stdev(v) if len(v) >= 2 else float("nan")


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


def request_interval(mav, name, hz):
    try:
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            MSG_IDS[name],
            1e6 / hz,
            0,
            0,
            0,
            0,
            0,
        )
    except Exception:
        pass


def analyze(rows):
    grouped = defaultdict(list)
    for row in rows:
        if row["gx"] is not None:
            grouped[(row["transport"], row["type"])].append(row)

    result = {}
    for key, rr in grouped.items():
        pre = [r for r in rr if r["phase"] == "PRE"]
        post = [r for r in rr if r["phase"] == "POST"]
        if not pre or not post:
            continue
        d = {"n_pre": len(pre), "n_post": len(post), "n_total": len(rr)}
        for axis in "xyz":
            k = "g" + axis
            pm = mean([r[k] for r in pre])
            qm = mean([r[k] for r in post])
            d["pre_" + axis] = pm
            d["post_" + axis] = qm
            d["delta_" + axis] = qm - pm
            d["pre_sd_" + axis] = sd([r[k] for r in pre])
            d["post_sd_" + axis] = sd([r[k] for r in post])
        d["delta_xy"] = math.hypot(d["delta_x"], d["delta_y"])
        d["delta_xyz"] = math.sqrt(
            d["delta_x"] ** 2 + d["delta_y"] ** 2 + d["delta_z"] ** 2
        )
        d["ratio"] = d["delta_xy"] / BASELINE_DXY
        d["blocks"] = {}
        for bn in (
            "PRE_EARLY",
            "PRE_MID",
            "PRE_LATE",
            "POST_EARLY",
            "POST_MID",
            "POST_LATE",
        ):
            q = [r for r in rr if r["block"] == bn]
            if q:
                d["blocks"][bn] = tuple(mean([r["g" + a] for r in q]) for a in "xyz")
        result[key] = d
    return result


def cross_transport_highres(rows, max_dt_us=5000):
    usb = [
        r for r in rows
        if r["transport"] == "USB" and r["type"] == "HIGHRES_IMU" and isinstance(r["fc_time"], int)
    ]
    uart = [
        r for r in rows
        if r["transport"] == "UART" and r["type"] == "HIGHRES_IMU" and isinstance(r["fc_time"], int)
    ]
    if not usb or not uart:
        return {"pairs": 0}

    usb.sort(key=lambda r: r["fc_time"])
    uart.sort(key=lambda r: r["fc_time"])
    j = 0
    pairs = []
    for u in usb:
        while j + 1 < len(uart) and uart[j + 1]["fc_time"] <= u["fc_time"]:
            j += 1
        candidates = [uart[j]]
        if j + 1 < len(uart):
            candidates.append(uart[j + 1])
        v = min(candidates, key=lambda r: abs(r["fc_time"] - u["fc_time"]))
        dt = int(v["fc_time"] - u["fc_time"])
        if abs(dt) > max_dt_us:
            continue
        dx = v["gx"] - u["gx"]
        dy = v["gy"] - u["gy"]
        dz = v["gz"] - u["gz"]
        pairs.append((u["phase"], dt, dx, dy, dz))

    if not pairs:
        return {"pairs": 0}

    def stats(subset):
        if not subset:
            return None
        mx = mean([p[2] for p in subset])
        my = mean([p[3] for p in subset])
        mz = mean([p[4] for p in subset])
        rms = math.sqrt(mean([p[2] ** 2 + p[3] ** 2 + p[4] ** 2 for p in subset]))
        return {
            "n": len(subset),
            "mean_x": mx,
            "mean_y": my,
            "mean_z": mz,
            "mean_xy": math.hypot(mx, my),
            "rms_vec": rms,
            "mean_abs_dt_us": mean([abs(p[1]) for p in subset]),
        }

    return {
        "pairs": len(pairs),
        "all": stats(pairs),
        "pre": stats([p for p in pairs if p[0] == "PRE"]),
        "post": stats([p for p in pairs if p[0] == "POST"]),
    }


def save(rows, result, cross, out_dir, uart_port, usb_port):
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"matek_h743_dual_transport_yaw_{stamp}.csv"
    txt_path = out_dir / f"matek_h743_dual_transport_yaw_{stamp}.txt"

    headers = [
        "host_t",
        "t_rel",
        "phase",
        "block",
        "transport",
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
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        for r in rows:
            w.writerow({
                "host_t": f"{r['host_t']:.9f}",
                "t_rel": f"{r['t']:.6f}",
                "phase": r["phase"],
                "block": r["block"],
                "transport": r["transport"],
                "type": r["type"],
                "fc_time": r["fc_time"],
                "gx_rad_s": "" if r["gx"] is None else f"{r['gx']:.12f}",
                "gy_rad_s": "" if r["gy"] is None else f"{r['gy']:.12f}",
                "gz_rad_s": "" if r["gz"] is None else f"{r['gz']:.12f}",
                "raw_xgyro": r["raw_x"],
                "raw_ygyro": r["raw_y"],
                "raw_zgyro": r["raw_z"],
                "roll": r["roll"],
                "pitch": r["pitch"],
                "yaw": r["yaw"],
                "q1": r["q1"],
                "q2": r["q2"],
                "q3": r["q3"],
                "q4": r["q4"],
                "vibration_x": r["vibration_x"],
                "vibration_y": r["vibration_y"],
                "vibration_z": r["vibration_z"],
                "clipping_0": r["clipping_0"],
                "clipping_1": r["clipping_1"],
                "clipping_2": r["clipping_2"],
            })

    counts = defaultdict(int)
    for r in rows:
        counts[(r["transport"], r["type"])] += 1

    with txt_path.open("w") as f:
        f.write("JT-Zero P11/B11 simultaneous USB vs UART IMU transport A/B\n")
        f.write("FC: Matek H743-SLIM V3 / ArduPilot\n")
        f.write(f"UART production path: {uart_port} @ 460800\n")
        f.write(f"USB diagnostic path: {usb_port} @ 115200\n")
        f.write("Sequence: 15 s STILL PRE -> ~90 deg YAW -> 15 s STILL POST\n")
        f.write(f"Problematic historical baseline |DELTA gyro XY| ~= {BASELINE_DXY:.9f} rad/s\n\n")

        f.write("MESSAGE COUNTS\n")
        for transport in ("UART", "USB"):
            f.write(f"[{transport}]\n")
            for name in WANTED:
                f.write(f"  {name}: {counts[(transport, name)]}\n")
        f.write("\n")

        for transport in ("UART", "USB"):
            f.write(f"================ {transport} ================\n")
            keys = [k for k in result if k[0] == transport]
            for _, typ in sorted(keys):
                d = result[(transport, typ)]
                f.write(f"[{typ}] PRE={d['n_pre']} POST={d['n_post']} TOTAL={d['n_total']}\n")
                for a in "xyz":
                    f.write(
                        f"  {a.upper()}: PRE={d['pre_' + a]:+.9f} "
                        f"POST={d['post_' + a]:+.9f} "
                        f"DELTA={d['delta_' + a]:+.9f} rad/s "
                        f"SDpre={d['pre_sd_' + a]:.9f} SDpost={d['post_sd_' + a]:.9f}\n"
                    )
                f.write(
                    f"  |DELTA XY|={d['delta_xy']:.9f} rad/s  "
                    f"|DELTA XYZ|={d['delta_xyz']:.9f} rad/s  "
                    f"ratio_to_problem_baseline={d['ratio']:.3f}\n"
                )
                for bn, vals in d["blocks"].items():
                    f.write(
                        f"  {bn}: X={vals[0]:+.9f} Y={vals[1]:+.9f} Z={vals[2]:+.9f}\n"
                    )
                f.write("\n")

        f.write("================ DIRECT HIGHRES UART-USB COMPARISON ================\n")
        f.write(f"Matched pairs: {cross.get('pairs', 0)}\n")
        for name in ("all", "pre", "post"):
            s = cross.get(name)
            if not s:
                continue
            f.write(
                f"{name.upper()}: N={s['n']} mean(UART-USB)="
                f"[{s['mean_x']:+.9f},{s['mean_y']:+.9f},{s['mean_z']:+.9f}] rad/s "
                f"meanXY={s['mean_xy']:.9f} rmsVec={s['rms_vec']:.9f} "
                f"mean|dt_fc|={s['mean_abs_dt_us']:.1f} us\n"
            )

        f.write("\nINTERPRETATION GUIDE\n")
        f.write("- USB stable, UART near 0.00202 rad/s: transport/UART path implicated.\n")
        f.write("- USB and UART both stable: historical v15.42 offset is not currently reproduced.\n")
        f.write("- USB and UART both show similar large shift: FC/state/motion-dependent source upstream of transport.\n")
        f.write("- Direct HIGHRES UART-USB difference should be near zero when matched by FC timestamp.\n")

    return csv_path, txt_path


class App:
    def __init__(self, root, uart_port, usb_port):
        self.root = root
        self.uart_port = uart_port
        self.usb_port = usb_port
        self.connections = {}
        self.rows = []
        self.rows_lock = threading.Lock()
        self.running = False
        self.start_t = None
        self.done_readers = 0
        self.events = queue.Queue()
        self.seen = defaultdict(int)
        self.last_att = None
        self.yaw_reference = None

        root.title("JT-Zero — USB vs UART IMU A/B")
        root.geometry("1120x780")
        tk.Label(
            root,
            text="Matek H743 — одновременный USB ↔ UART IMU A/B",
            font=("DejaVu Sans", 20, "bold"),
        ).pack(pady=(18, 8))
        self.status = tk.Label(root, text="Готово", font=("DejaVu Sans", 26, "bold"))
        self.status.pack(pady=8)
        self.instr = tk.Label(
            root,
            text="Положите полётник неподвижно. USB и UART должны оставаться подключены.",
            font=("DejaVu Sans", 15),
        )
        self.instr.pack(pady=4)
        self.timer = tk.Label(root, text="00.0 с", font=("DejaVu Sans Mono", 42, "bold"))
        self.timer.pack(pady=8)
        self.att = tk.Label(root, text="ATTITUDE USB: ---", font=("DejaVu Sans Mono", 15))
        self.att.pack(pady=4)
        self.yaw_target = tk.Label(
            root,
            text="YAW относительно старта: ---   Цель: ±90° (75…105°)",
            font=("DejaVu Sans", 15, "bold"),
        )
        self.yaw_target.pack(pady=4)
        self.streams = tk.Label(
            root,
            text="Потоки: ---",
            font=("DejaVu Sans Mono", 11),
            justify="left",
        )
        self.streams.pack(pady=8)
        self.btn = tk.Button(
            root,
            text="НАЧАТЬ ОДИН A/B ПРОГОН",
            font=("DejaVu Sans", 16, "bold"),
            width=28,
            command=self.start,
        )
        self.btn.pack(pady=12)
        self.result = tk.Label(
            root,
            text="",
            font=("DejaVu Sans Mono", 11),
            justify="left",
            anchor="w",
        )
        self.result.pack(fill="x", padx=35, pady=8)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.tick)

    @staticmethod
    def wrap_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def connect_one(self, label, port, baud):
        m = mavutil.mavlink_connection(
            port,
            baud=baud,
            autoreconnect=False,
            source_system=255,
            source_component=190,
        )
        hb = m.wait_heartbeat(timeout=5)
        if hb is None:
            m.close()
            raise RuntimeError(f"{label}: HEARTBEAT timeout on {port}")
        for name in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "HIGHRES_IMU"):
            request_interval(m, name, 50)
        for name in ("ATTITUDE", "ATTITUDE_QUATERNION", "VIBRATION"):
            request_interval(m, name, 20)
        return m

    def start(self):
        if self.running:
            return
        try:
            uart = self.connect_one("UART", self.uart_port, 460800)
            usb = self.connect_one("USB", self.usb_port, 115200)
        except Exception as exc:
            try:
                if "uart" in locals():
                    uart.close()
            except Exception:
                pass
            messagebox.showerror("Ошибка MAVLink", str(exc))
            return

        self.connections = {"UART": uart, "USB": usb}
        self.rows = []
        self.seen = defaultdict(int)
        self.last_att = None
        self.yaw_reference = None
        self.done_readers = 0
        self.start_t = time.monotonic()
        self.running = True
        self.btn.config(state="disabled")
        self.result.config(text="Оба канала подключены. Запись идёт одновременно.")
        threading.Thread(target=self.collect, args=("UART", uart), daemon=True).start()
        threading.Thread(target=self.collect, args=("USB", usb), daemon=True).start()

    def collect(self, transport, mav):
        try:
            while self.running:
                now = time.monotonic()
                t = now - self.start_t
                if t >= TOTAL_S:
                    break
                msg = mav.recv_match(type=WANTED, blocking=True, timeout=0.20)
                if msg is None:
                    continue
                typ = msg.get_type()
                self.seen[(transport, typ)] += 1
                g = gyro_rad(msg)
                gx = gy = gz = None
                if g:
                    gx, gy, gz = g

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
                    if transport == "USB":
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

                row = dict(
                    host_t=now,
                    t=t,
                    phase=phase_for(t),
                    block=block_for(t),
                    transport=transport,
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
                with self.rows_lock:
                    self.rows.append(row)
        except Exception as exc:
            self.events.put(("error", f"{transport}: {exc}"))
        finally:
            try:
                mav.close()
            except Exception:
                pass
            self.events.put(("reader_done", transport))

    def tick(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "error":
                    messagebox.showerror("Ошибка", data)
                elif kind == "reader_done":
                    self.done_readers += 1
                    if self.done_readers >= 2 and self.start_t is not None:
                        self.running = False
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

            self.timer.config(text=f"{max(0.0, remaining):04.1f} с")
            if self.last_att:
                roll, pitch, yaw = self.last_att
                self.att.config(
                    text=(
                        f"ATTITUDE USB deg: R {math.degrees(roll):+.2f}  "
                        f"P {math.degrees(pitch):+.2f}  Y {math.degrees(yaw):+.2f}"
                    )
                )
                if self.yaw_reference is not None:
                    dyaw = math.degrees(self.wrap_pi(yaw - self.yaw_reference))
                    ad = abs(dyaw)
                    if 75.0 <= ad <= 105.0:
                        verdict = "В ЦЕЛЕВОЙ ЗОНЕ"
                    elif ad < 75.0:
                        verdict = "ДОВЕРНУТЬ"
                    else:
                        verdict = "СЛИШКОМ ДАЛЕКО"
                    self.yaw_target.config(
                        text=f"YAW относительно старта: {dyaw:+.1f}°   Цель: ±90° (75…105°)   {verdict}"
                    )

            lines = []
            for transport in ("UART", "USB"):
                lines.append(
                    transport + ": " + "  ".join(
                        f"{name}:{self.seen[(transport, name)]}" for name in WANTED
                    )
                )
            self.streams.config(text="\n".join(lines))

        self.root.after(50, self.tick)

    def finish(self):
        with self.rows_lock:
            rows = list(self.rows)
        if not rows:
            self.btn.config(state="normal")
            self.start_t = None
            return
        result = analyze(rows)
        cross = cross_transport_highres(rows)
        out_dir = Path(__file__).resolve().parent / "results"
        csv_path, txt_path = save(
            rows,
            result,
            cross,
            out_dir,
            self.uart_port,
            self.usb_port,
        )
        lines = ["ТЕСТ ЗАВЕРШЁН", f"TXT: {txt_path.name}"]
        for transport in ("UART", "USB"):
            d = result.get((transport, "HIGHRES_IMU"))
            if d:
                lines.append(
                    f"{transport} HIGHRES: |ΔXY|={d['delta_xy']:.7f} rad/s ({d['ratio']:.3f}x baseline)"
                )
        if cross.get("all"):
            lines.append(
                f"UART-USB HIGHRES direct RMS={cross['all']['rms_vec']:.7f} rad/s, pairs={cross['pairs']}"
            )
        self.status.config(text="ТЕСТ ЗАВЕРШЁН")
        self.instr.config(text="Результат сохранён. Полётник можно двигать.")
        self.timer.config(text="00.0 с")
        self.result.config(text="\n".join(lines))
        self.btn.config(state="normal")
        self.start_t = None

    def close(self):
        self.running = False
        for mav in self.connections.values():
            try:
                mav.close()
            except Exception:
                pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uart", default="/dev/ttyAMA0")
    parser.add_argument("--usb", default="/dev/ttyACM0")
    args = parser.parse_args()
    root = tk.Tk()
    App(root, args.uart, args.usb)
    root.mainloop()


if __name__ == "__main__":
    main()
