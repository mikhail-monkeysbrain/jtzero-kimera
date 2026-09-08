#!/usr/bin/env python3
"""JT-Zero v50 GUI A→B→A POSTCAL with settle phases.

Protocol:
  WAIT
  -> A_START_SETTLE 5 s
  -> A_START_STILL 10 s
  -> READY_A_TO_B
  -> A_TO_B
  -> B_SETTLE 5 s
  -> B_STILL 10 s
  -> READY_B_TO_A
  -> B_TO_A
  -> A_END_SETTLE 5 s
  -> A_END_STILL 10 s
  -> FINISHED

The airframe orientation must remain fixed for the entire run.
On B→A move backwards; do not yaw/turn the stand around.
"""

from __future__ import annotations

import csv
import os
import threading
import time
import tkinter as tk
from tkinter import messagebox
from typing import Dict, Optional

from pymavlink import mavutil

PORT = "/dev/ttyAMA0"
BAUD = 460800
OUT = "/home/vio/jtzero_ab_gui_v50.csv"

SETTLE_SECONDS = 5.0
STILL_SECONDS = 10.0

WANTED = (
    "I0SX", "I0SY", "I0SZ", "I0CX", "I0CY", "I0CZ",
    "I1SX", "I1SY", "I1SZ", "I1CX", "I1CY", "I1CZ",
)

WAIT = "WAIT"
A_START_SETTLE = "A_START_SETTLE"
A_START_STILL = "A_START_STILL"
READY_A_TO_B = "READY_A_TO_B"
A_TO_B = "A_TO_B"
B_SETTLE = "B_SETTLE"
B_STILL = "B_STILL"
READY_B_TO_A = "READY_B_TO_A"
B_TO_A = "B_TO_A"
A_END_SETTLE = "A_END_SETTLE"
A_END_STILL = "A_END_STILL"
FINISHED = "FINISHED"

TEXT = {
    WAIT: (
        "Поставь стенд в A.\n"
        "Нос направлен A → B.\n"
        "На обратном ходе корпус НЕ разворачивать."
    ),
    A_START_SETTLE: "A — ОТПУСТИ СТЕНД\nОЖИДАНИЕ УСПОКОЕНИЯ",
    A_START_STILL: "A — ЗАПИСЬ STILL\nНЕ ТРОГАТЬ СТЕНД",
    READY_A_TO_B: "A STILL ГОТОВ\nМожно начинать движение A → B",
    A_TO_B: "ДВИГАЙ A → B\nКорпус не разворачивать",
    B_SETTLE: "B — ОТПУСТИ СТЕНД\nОЖИДАНИЕ УСПОКОЕНИЯ",
    B_STILL: "B — ЗАПИСЬ STILL\nНЕ ТРОГАТЬ СТЕНД",
    READY_B_TO_A: "B STILL ГОТОВ\nМожно начинать движение B → A",
    B_TO_A: "ДВИГАЙ B → A НАЗАД\nКорпус НЕ разворачивать",
    A_END_SETTLE: "A — ОТПУСТИ СТЕНД\nОЖИДАНИЕ УСПОКОЕНИЯ",
    A_END_STILL: "A — ФИНАЛЬНАЯ ЗАПИСЬ STILL\nНЕ ТРОГАТЬ СТЕНД",
    FINISHED: "ТЕСТ ЗАВЕРШЁН",
}


class App:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.running = True
        self.phase = WAIT
        self.phase_started = time.monotonic()
        self.test_started = time.monotonic()
        self.rows = 0
        self.counts: Dict[str, int] = {name: 0 for name in WANTED}

        self.mav = None
        self.csv_file = None
        self.writer = None
        self.worker: Optional[threading.Thread] = None

        self.root = tk.Tk()
        self.root.title("JT-Zero v50 — A/B/A POSTCAL CLEAN")
        self.root.geometry("940x700")
        self.root.minsize(840, 620)

        self.phase_label = tk.Label(
            self.root, text=WAIT, font=("DejaVu Sans", 32, "bold")
        )
        self.phase_label.pack(pady=(26, 8))

        self.instruction_label = tk.Label(
            self.root,
            text=TEXT[WAIT],
            font=("DejaVu Sans", 22),
            justify="center",
        )
        self.instruction_label.pack(pady=14)

        self.timer_label = tk.Label(
            self.root,
            text="--",
            font=("DejaVu Sans Mono", 54, "bold"),
        )
        self.timer_label.pack(pady=18)

        self.status_label = tk.Label(
            self.root,
            text="MAVLink: подключение...",
            font=("DejaVu Sans Mono", 14),
        )
        self.status_label.pack(pady=8)

        buttons = tk.Frame(self.root)
        buttons.pack(pady=18)

        self.start_button = tk.Button(
            buttons,
            text="START",
            font=("DejaVu Sans", 17),
            width=22,
            command=self.start_test,
        )
        self.start_button.grid(row=0, column=0, padx=8, pady=8)

        self.a_to_b_button = tk.Button(
            buttons,
            text="НАЧАТЬ A → B",
            font=("DejaVu Sans", 17),
            width=22,
            state="disabled",
            command=self.start_a_to_b,
        )
        self.a_to_b_button.grid(row=0, column=1, padx=8, pady=8)

        self.arrived_b_button = tk.Button(
            buttons,
            text="Я В B",
            font=("DejaVu Sans", 17),
            width=22,
            state="disabled",
            command=self.arrived_b,
        )
        self.arrived_b_button.grid(row=1, column=0, padx=8, pady=8)

        self.b_to_a_button = tk.Button(
            buttons,
            text="НАЧАТЬ B → A",
            font=("DejaVu Sans", 17),
            width=22,
            state="disabled",
            command=self.start_b_to_a,
        )
        self.b_to_a_button.grid(row=1, column=1, padx=8, pady=8)

        self.arrived_a_button = tk.Button(
            buttons,
            text="Я В A",
            font=("DejaVu Sans", 17),
            width=22,
            state="disabled",
            command=self.arrived_a,
        )
        self.arrived_a_button.grid(
            row=2, column=0, columnspan=2, padx=8, pady=8
        )

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def get_phase(self) -> str:
        with self.lock:
            return self.phase

    def phase_age(self) -> float:
        with self.lock:
            return time.monotonic() - self.phase_started

    def set_phase(self, phase: str) -> None:
        with self.lock:
            self.phase = phase
            self.phase_started = time.monotonic()
        self.update_controls()

    def connect(self) -> None:
        print(f"[MAV] opening {PORT} @ {BAUD}")
        self.mav = mavutil.mavlink_connection(
            PORT,
            baud=BAUD,
            source_system=255,
            source_component=190,
        )

        print("[MAV] waiting HEARTBEAT...")
        hb = self.mav.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError("HEARTBEAT timeout")

        print(
            f"[MAV] FC sysid={hb.get_srcSystem()} "
            f"compid={hb.get_srcComponent()}"
        )

        try:
            if hasattr(self.mav, "port") and hasattr(
                self.mav.port, "reset_input_buffer"
            ):
                self.mav.port.reset_input_buffer()
        except Exception as exc:
            print(f"[MAV] input-buffer reset warning: {exc}")

        self.csv_file = open(OUT, "w", newline="", buffering=1)
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ["host_time_s", "fc_time_ms", "phase", "name", "value"]
        )

        self.worker = threading.Thread(target=self.mav_loop, daemon=True)
        self.worker.start()
        self.status_label.config(
            text=f"MAVLink: OK    CSV: {os.path.basename(OUT)}"
        )

    def mav_loop(self) -> None:
        last_hb = 0.0

        while self.running:
            now = time.monotonic()
            if now - last_hb >= 1.0:
                try:
                    self.mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        0,
                    )
                except Exception:
                    pass
                last_hb = now

            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.05)
            except Exception:
                continue

            if msg is None or msg.get_type() != "NAMED_VALUE_FLOAT":
                continue

            name = msg.name
            if isinstance(name, bytes):
                name = name.decode("ascii", errors="ignore")
            name = name.rstrip("\x00")

            if name not in self.counts:
                continue

            phase = self.get_phase()
            host_time_s = time.monotonic() - self.test_started

            try:
                self.writer.writerow(
                    [
                        f"{host_time_s:.6f}",
                        int(msg.time_boot_ms),
                        phase,
                        name,
                        float(msg.value),
                    ]
                )
            except Exception:
                continue

            with self.lock:
                self.rows += 1
                self.counts[name] += 1

    def start_test(self) -> None:
        if self.get_phase() == WAIT:
            self.set_phase(A_START_SETTLE)

    def start_a_to_b(self) -> None:
        if self.get_phase() == READY_A_TO_B:
            self.set_phase(A_TO_B)

    def arrived_b(self) -> None:
        if self.get_phase() == A_TO_B:
            self.set_phase(B_SETTLE)

    def start_b_to_a(self) -> None:
        if self.get_phase() == READY_B_TO_A:
            self.set_phase(B_TO_A)

    def arrived_a(self) -> None:
        if self.get_phase() == B_TO_A:
            self.set_phase(A_END_SETTLE)

    def update_controls(self) -> None:
        phase = self.get_phase()

        self.phase_label.config(text=phase)
        self.instruction_label.config(text=TEXT[phase])

        self.start_button.config(
            state="normal" if phase == WAIT else "disabled"
        )
        self.a_to_b_button.config(
            state="normal" if phase == READY_A_TO_B else "disabled"
        )
        self.arrived_b_button.config(
            state="normal" if phase == A_TO_B else "disabled"
        )
        self.b_to_a_button.config(
            state="normal" if phase == READY_B_TO_A else "disabled"
        )
        self.arrived_a_button.config(
            state="normal" if phase == B_TO_A else "disabled"
        )

    def tick(self) -> None:
        if not self.running:
            return

        phase = self.get_phase()
        age = self.phase_age()

        settle_phases = {
            A_START_SETTLE: A_START_STILL,
            B_SETTLE: B_STILL,
            A_END_SETTLE: A_END_STILL,
        }
        still_phases = {
            A_START_STILL: READY_A_TO_B,
            B_STILL: READY_B_TO_A,
            A_END_STILL: FINISHED,
        }

        if phase in settle_phases:
            remain = max(0.0, SETTLE_SECONDS - age)
            self.timer_label.config(text=f"SETTLE {remain:04.1f}s")
            if remain <= 0.0:
                self.set_phase(settle_phases[phase])

        elif phase in still_phases:
            remain = max(0.0, STILL_SECONDS - age)
            self.timer_label.config(text=f"REC {remain:04.1f}s")
            if remain <= 0.0:
                self.set_phase(still_phases[phase])

        elif phase in (A_TO_B, B_TO_A):
            self.timer_label.config(text=f"MOVE {age:04.1f}s")

        elif phase in (READY_A_TO_B, READY_B_TO_A):
            self.timer_label.config(text="READY")

        elif phase == FINISHED:
            self.timer_label.config(text="DONE")

        else:
            self.timer_label.config(text="--")

        with self.lock:
            rows = self.rows
            i0 = self.counts["I0CX"]
            i1 = self.counts["I1CX"]

        self.status_label.config(
            text=f"MAVLink: OK    rows={rows}    I0CX={i0}    I1CX={i1}"
        )
        self.root.after(100, self.tick)

    def on_close(self) -> None:
        if self.get_phase() != FINISHED:
            if not messagebox.askyesno(
                "Завершить?",
                "Тест ещё не завершён. Закрыть программу?",
            ):
                return

        self.running = False
        time.sleep(0.1)

        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass

        self.root.destroy()

    def run(self) -> None:
        self.connect()
        self.update_controls()
        self.root.after(100, self.tick)
        self.root.mainloop()

        self.running = False
        print()
        print("===== TEST COMPLETE =====")
        print(f"CSV: {OUT}")
        print(f"rows: {self.rows}")


if __name__ == "__main__":
    app = App()
    try:
        app.run()
    except Exception as exc:
        try:
            messagebox.showerror("JT-Zero v50", str(exc))
        except Exception:
            pass
        raise
