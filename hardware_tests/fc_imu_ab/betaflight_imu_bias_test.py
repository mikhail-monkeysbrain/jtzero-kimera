#!/usr/bin/env python3
import argparse
import csv
import math
import queue
import statistics
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import serial
import tkinter as tk
from tkinter import messagebox

MSP_API_VERSION = 1
MSP_RAW_IMU = 102

BASELINE_DXY_RAD_S = 0.00202
BASELINE_DX_RAD_S = 0.00192

PRE_S = 15.0
YAW_S = 10.0
POST_S = 15.0
TOTAL_S = PRE_S + YAW_S + POST_S
REQUEST_PERIOD_S = 0.02  # ~50 Hz host polling

# MSP_RAW_IMU in Betaflight 4.4/4.5 serializes gyroRateDps(), which returns
# gyro.gyroADCf / rawSensorDev->scale, i.e. sensor-count-equivalent values.
# Therefore conversion back to deg/s must use the actual driver scale.
PROFILES = {
    "karma-lsm6dsv16x": {
        "fc": "KARMAF435V1G / AT32F435G / Betaflight 4.5.4",
        "imu": "LSM6DSV16X",
        "alignment": "CW0FLIP",
        "gyro_deg_per_count": 0.0700000000,
        "slug": "karma_lsm6dsv16x",
    },
    "hk4530-icm42688p": {
        "fc": "HK4530V2.1 / HAKRCF405V2 / STM32F405 / Betaflight 4.4.3",
        "imu": "ICM42688P",
        "alignment": "CW90",
        "gyro_deg_per_count": 2000.0 / 32768.0,
        "slug": "hk4530_icm42688p",
    },
}


@dataclass
class Sample:
    t_monotonic: float
    t_rel: float
    phase: str
    acc_x: int
    acc_y: int
    acc_z: int
    gyro_raw_x: int
    gyro_raw_y: int
    gyro_raw_z: int
    gyro_dps_x: float
    gyro_dps_y: float
    gyro_dps_z: float
    gyro_rad_x: float
    gyro_rad_y: float
    gyro_rad_z: float


def msp_v1_packet(cmd: int, payload: bytes = b"") -> bytes:
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes((size, cmd)) + payload + bytes((checksum,))


def read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    out = bytearray()
    while len(out) < n and time.monotonic() < deadline:
        chunk = ser.read(n - len(out))
        if chunk:
            out.extend(chunk)
    return bytes(out)


def read_msp_v1(ser: serial.Serial, expected_cmd: int, timeout_s: float = 0.25) -> bytes:
    deadline = time.monotonic() + timeout_s
    sync = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        sync += b
        if len(sync) > 3:
            sync = sync[-3:]
        if bytes(sync) in (b"$M>", b"$M!"):
            if bytes(sync) == b"$M!":
                raise RuntimeError(f"MSP command {expected_cmd} rejected by FC")
            break
    else:
        raise TimeoutError("MSP header timeout")

    hdr = read_exact(ser, 2, deadline)
    if len(hdr) != 2:
        raise TimeoutError("MSP header incomplete")
    size, cmd = hdr
    payload = read_exact(ser, size, deadline)
    cks = read_exact(ser, 1, deadline)
    if len(payload) != size or len(cks) != 1:
        raise TimeoutError("MSP payload incomplete")

    calc = size ^ cmd
    for b in payload:
        calc ^= b
    if calc != cks[0]:
        raise ValueError(f"MSP checksum error: got={cks[0]:02x} expected={calc:02x}")
    if cmd != expected_cmd:
        raise ValueError(f"Unexpected MSP command: {cmd}, expected {expected_cmd}")
    return payload


def request(ser: serial.Serial, cmd: int) -> bytes:
    ser.write(msp_v1_packet(cmd))
    ser.flush()
    return read_msp_v1(ser, cmd)


def parse_raw_imu(payload: bytes):
    if len(payload) < 18:
        raise ValueError(f"MSP_RAW_IMU payload too short: {len(payload)}")
    return struct.unpack_from("<9h", payload, 0)


def mean(values):
    return statistics.fmean(values) if values else float("nan")


def sd(values):
    return statistics.stdev(values) if len(values) >= 2 else float("nan")


def sem(values):
    return sd(values) / math.sqrt(len(values)) if len(values) >= 2 else float("nan")


def phase_for(t_rel: float) -> str:
    if t_rel < PRE_S:
        return "PRE"
    if t_rel < PRE_S + YAW_S:
        return "YAW"
    return "POST"


def analyze(samples):
    pre = [s for s in samples if s.phase == "PRE"]
    post = [s for s in samples if s.phase == "POST"]

    def axis_stats(group, attr):
        vals = [getattr(s, attr) for s in group]
        return mean(vals), sd(vals), sem(vals)

    result = {"n_pre": len(pre), "n_post": len(post)}
    for axis in "xyz":
        pre_m, pre_sd, pre_sem = axis_stats(pre, f"gyro_rad_{axis}")
        post_m, post_sd, post_sem = axis_stats(post, f"gyro_rad_{axis}")
        result[f"pre_{axis}"] = pre_m
        result[f"post_{axis}"] = post_m
        result[f"delta_{axis}"] = post_m - pre_m
        result[f"pre_sd_{axis}"] = pre_sd
        result[f"post_sd_{axis}"] = post_sd
        result[f"pre_sem_{axis}"] = pre_sem
        result[f"post_sem_{axis}"] = post_sem

    dx, dy, dz = result["delta_x"], result["delta_y"], result["delta_z"]
    result["delta_xy"] = math.hypot(dx, dy)
    result["delta_xyz"] = math.sqrt(dx * dx + dy * dy + dz * dz)
    result["ratio_to_baseline_xy"] = result["delta_xy"] / BASELINE_DXY_RAD_S
    return result


def save_results(samples, result, out_dir: Path, profile, msp_api: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"{profile['slug']}_yaw_bias_{stamp}.csv"
    txt_path = out_dir / f"{profile['slug']}_yaw_bias_{stamp}.txt"

    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t_monotonic_s", "t_rel_s", "phase",
            "acc_raw_x", "acc_raw_y", "acc_raw_z",
            "gyro_msp_count_x", "gyro_msp_count_y", "gyro_msp_count_z",
            "gyro_dps_x", "gyro_dps_y", "gyro_dps_z",
            "gyro_rad_s_x", "gyro_rad_s_y", "gyro_rad_s_z",
        ])
        for s in samples:
            w.writerow([
                f"{s.t_monotonic:.9f}", f"{s.t_rel:.6f}", s.phase,
                s.acc_x, s.acc_y, s.acc_z,
                s.gyro_raw_x, s.gyro_raw_y, s.gyro_raw_z,
                f"{s.gyro_dps_x:.9f}", f"{s.gyro_dps_y:.9f}", f"{s.gyro_dps_z:.9f}",
                f"{s.gyro_rad_x:.12f}", f"{s.gyro_rad_y:.12f}", f"{s.gyro_rad_z:.12f}",
            ])

    with txt_path.open("w") as f:
        f.write("JT-Zero P11/B11 FC IMU A/B test\n")
        f.write(f"FC: {profile['fc']}\n")
        f.write(f"IMU: {profile['imu']}\n")
        f.write(f"Sensor alignment: {profile['alignment']}\n")
        f.write(f"MSP API: {msp_api}\n")
        f.write("Protocol: MSP_RAW_IMU over USB ACM\n")
        f.write("Sequence: 15 s STILL -> ~90 deg YAW -> 15 s STILL\n")
        f.write("Coordinate handling: native Betaflight MSP values; no FRD->FLU, no ZXY correction\n")
        f.write(f"Gyro sensor scale used: {profile['gyro_deg_per_count']:.12f} deg/s per count\n")
        f.write(f"Samples: PRE={result['n_pre']} POST={result['n_post']} TOTAL={len(samples)}\n\n")

        for axis in "xyz":
            f.write(
                f"{axis.upper()}: PRE={result[f'pre_{axis}']:+.9f} rad/s "
                f"POST={result[f'post_{axis}']:+.9f} rad/s "
                f"DELTA={result[f'delta_{axis}']:+.9f} rad/s "
                f"PRE_SD={result[f'pre_sd_{axis}']:.9f} "
                f"POST_SD={result[f'post_sd_{axis}']:.9f} "
                f"PRE_SEM={result[f'pre_sem_{axis}']:.9f} "
                f"POST_SEM={result[f'post_sem_{axis}']:.9f}\n"
            )

        f.write("\n")
        f.write(f"|DELTA gyro XY|  = {result['delta_xy']:.9f} rad/s\n")
        f.write(f"|DELTA gyro XYZ| = {result['delta_xyz']:.9f} rad/s\n")
        f.write(f"Baseline ICM42688 |DELTA gyro XY| ~= {BASELINE_DXY_RAD_S:.9f} rad/s\n")
        f.write(f"Baseline ICM42688 DELTA X ~= +{BASELINE_DX_RAD_S:.9f} rad/s\n")
        f.write(f"Ratio test/baseline XY = {result['ratio_to_baseline_xy']:.3f}\n")

    return csv_path, txt_path


class App:
    def __init__(self, root, port, profile):
        self.root = root
        self.port = port
        self.profile = profile
        self.gyro_deg_per_count = profile["gyro_deg_per_count"]
        self.gyro_rad_per_count = math.radians(self.gyro_deg_per_count)
        self.ser = None
        self.samples = []
        self.events = queue.Queue()
        self.running = False
        self.start_time = None
        self.last_sample = None
        self.worker = None
        self.msp_api = "unknown"

        root.title("JT-Zero — тест смещения IMU после YAW")
        root.geometry("940x650")
        root.minsize(840, 580)

        self.title = tk.Label(root, text=f"{profile['fc']} / {profile['imu']} — B11 A/B", font=("DejaVu Sans", 18, "bold"), wraplength=900)
        self.title.pack(pady=(18, 8))

        self.status = tk.Label(root, text="Готово к запуску", font=("DejaVu Sans", 24, "bold"))
        self.status.pack(pady=8)
        self.instruction = tk.Label(root, text="Положите полётник неподвижно на стол", font=("DejaVu Sans", 16))
        self.instruction.pack(pady=4)
        self.timer = tk.Label(root, text="00.0 с", font=("DejaVu Sans Mono", 42, "bold"))
        self.timer.pack(pady=12)
        self.gyro = tk.Label(root, text="GYRO rad/s: X ---   Y ---   Z ---", font=("DejaVu Sans Mono", 16))
        self.gyro.pack(pady=8)
        self.samples_label = tk.Label(root, text="Сэмплы: 0   Частота: --- Hz", font=("DejaVu Sans", 13))
        self.samples_label.pack(pady=4)

        self.progress = tk.Canvas(root, width=760, height=42, highlightthickness=0)
        self.progress.pack(pady=18)
        self.progress.create_rectangle(0, 0, 285, 42, fill="#bfe8bf", outline="")
        self.progress.create_rectangle(285, 0, 475, 42, fill="#f6dda0", outline="")
        self.progress.create_rectangle(475, 0, 760, 42, fill="#bfe8bf", outline="")
        self.progress.create_text(142, 21, text="ПОКОЙ 15 с", font=("DejaVu Sans", 12, "bold"))
        self.progress.create_text(380, 21, text="YAW ~90°", font=("DejaVu Sans", 12, "bold"))
        self.progress.create_text(617, 21, text="ПОКОЙ 15 с", font=("DejaVu Sans", 12, "bold"))
        self.marker = self.progress.create_line(0, 0, 0, 42, width=4)

        self.start_btn = tk.Button(root, text="НАЧАТЬ ТЕСТ", font=("DejaVu Sans", 16, "bold"), width=22, command=self.start)
        self.start_btn.pack(pady=12)
        self.result = tk.Label(root, text="", justify="left", anchor="w", font=("DejaVu Sans Mono", 13))
        self.result.pack(fill="x", padx=55, pady=8)

        self.root.after(50, self.ui_tick)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def connect(self):
        self.ser = serial.Serial(self.port, 115200, timeout=0.05, write_timeout=0.2)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        payload = request(self.ser, MSP_API_VERSION)
        if len(payload) < 3:
            raise RuntimeError("MSP_API_VERSION returned short payload")
        return f"{payload[0]}.{payload[1]}.{payload[2]}"

    def start(self):
        if self.running:
            return
        try:
            self.msp_api = self.connect()
        except Exception as e:
            messagebox.showerror("Ошибка подключения", f"Не удалось открыть {self.port}:\n{e}")
            return

        self.samples = []
        self.start_time = time.monotonic()
        self.running = True
        self.start_btn.config(state="disabled")
        self.result.config(text=f"MSP API {self.msp_api}. Сбор данных запущен.")
        self.worker = threading.Thread(target=self.collect, daemon=True)
        self.worker.start()

    def collect(self):
        next_t = time.monotonic()
        try:
            while self.running:
                now = time.monotonic()
                t_rel = now - self.start_time
                if t_rel >= TOTAL_S:
                    break

                values = parse_raw_imu(request(self.ser, MSP_RAW_IMU))
                ax, ay, az, gx, gy, gz, _mx, _my, _mz = values
                phase = phase_for(t_rel)
                s = Sample(
                    t_monotonic=now, t_rel=t_rel, phase=phase,
                    acc_x=ax, acc_y=ay, acc_z=az,
                    gyro_raw_x=gx, gyro_raw_y=gy, gyro_raw_z=gz,
                    gyro_dps_x=gx * self.gyro_deg_per_count,
                    gyro_dps_y=gy * self.gyro_deg_per_count,
                    gyro_dps_z=gz * self.gyro_deg_per_count,
                    gyro_rad_x=gx * self.gyro_rad_per_count,
                    gyro_rad_y=gy * self.gyro_rad_per_count,
                    gyro_rad_z=gz * self.gyro_rad_per_count,
                )
                self.samples.append(s)
                self.last_sample = s

                next_t += REQUEST_PERIOD_S
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()
        except Exception as e:
            self.events.put(("error", str(e)))
        finally:
            self.running = False
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.events.put(("done", None))

    def ui_tick(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "error":
                    messagebox.showerror("Ошибка MSP", data)
                elif kind == "done" and self.start_time is not None:
                    self.finish()
        except queue.Empty:
            pass

        if self.running and self.start_time is not None:
            t = min(time.monotonic() - self.start_time, TOTAL_S)
            p = phase_for(min(t, TOTAL_S - 1e-6))
            remain = PRE_S - t if p == "PRE" else (PRE_S + YAW_S - t if p == "YAW" else TOTAL_S - t)

            if p == "PRE":
                self.status.config(text="ПОКОЙ")
                self.instruction.config(text="НЕ ДВИГАТЬ полётник")
            elif p == "YAW":
                self.status.config(text="ПОВОРОТ YAW ~90°")
                self.instruction.config(text="Плавно поверните только по YAW примерно на 90°, затем положите неподвижно")
            else:
                self.status.config(text="ПОКОЙ ПОСЛЕ YAW")
                self.instruction.config(text="НЕ ДВИГАТЬ полётник")

            self.timer.config(text=f"{max(remain, 0):04.1f} с")
            x = 760.0 * t / TOTAL_S
            self.progress.coords(self.marker, x, 0, x, 42)

            if self.last_sample:
                s = self.last_sample
                self.gyro.config(text=f"GYRO rad/s: X {s.gyro_rad_x:+.5f}   Y {s.gyro_rad_y:+.5f}   Z {s.gyro_rad_z:+.5f}")
                hz = len(self.samples) / t if t > 0 else 0
                self.samples_label.config(text=f"Сэмплы: {len(self.samples)}   Средняя частота: {hz:.1f} Hz")

        self.root.after(50, self.ui_tick)

    def finish(self):
        if not self.samples:
            self.start_btn.config(state="normal")
            return
        result = analyze(self.samples)
        out_dir = Path(__file__).resolve().parent / "results"
        csv_path, txt_path = save_results(self.samples, result, out_dir, self.profile, self.msp_api)

        text = (
            f"Δgyro X  = {result['delta_x']:+.7f} rad/s\n"
            f"Δgyro Y  = {result['delta_y']:+.7f} rad/s\n"
            f"Δgyro Z  = {result['delta_z']:+.7f} rad/s\n"
            f"|Δgyro XY| = {result['delta_xy']:.7f} rad/s\n"
            f"baseline ICM42688 = {BASELINE_DXY_RAD_S:.7f} rad/s\n"
            f"отношение = {result['ratio_to_baseline_xy']:.3f}x\n\n"
            f"CSV: {csv_path.name}\nTXT: {txt_path.name}"
        )
        self.status.config(text="ТЕСТ ЗАВЕРШЁН")
        self.instruction.config(text="Результат сохранён. Полётник можно двигать.")
        self.timer.config(text="00.0 с")
        self.result.config(text=text)
        self.start_btn.config(state="normal")
        self.start_time = None

    def close(self):
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="JT-Zero Betaflight IMU stationary/yaw/stationary bias test")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--profile", choices=sorted(PROFILES), default="hk4530-icm42688p")
    args = ap.parse_args()

    profile = PROFILES[args.profile]
    print(f"Profile: {args.profile}")
    print(f"FC: {profile['fc']}")
    print(f"IMU: {profile['imu']}")
    print(f"Gyro scale: {profile['gyro_deg_per_count']:.12f} deg/s/count")
    print(f"Port: {args.port}")

    root = tk.Tk()
    App(root, args.port, profile)
    root.mainloop()


if __name__ == "__main__":
    main()
