#!/usr/bin/env python3
from __future__ import annotations

import os
import queue
import subprocess
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext

ROOT = Path(__file__).resolve().parents[1]
KIMERA_ROOT = Path(os.environ.get("KIMERA_ROOT", "/home/vio/Kimera-VIO"))
CAMERA = os.environ.get(
    "JTZERO_FLOW_CAMERA",
    "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0",
)
STAMP = time.strftime("%Y%m%d_%H%M%S")
OUT = Path(f"/home/vio/jtzero_runs/{STAMP}_OF_CAP_DECISION")
REC = "/tmp/jtzero_of_cap_decision_record"
BUILD_LOG = Path("/tmp/jtzero_of_cap_decision_record.build.log")

events: queue.Queue[tuple[str, str]] = queue.Queue()
stop_requested = False
active_proc: subprocess.Popen | None = None


def put(kind: str, text: str = ""):
    events.put((kind, text))


def find_mavlink_inc() -> str:
    candidates = [
        KIMERA_ROOT / "third_party/mavlink",
        KIMERA_ROOT / "third_party/mavlink/include/mavlink/v2.0",
        Path("/usr/local/include/mavlink/v2.0"),
    ]
    for d in candidates:
        if (d / "common/mavlink.h").is_file() and (d / "ardupilotmega/mavlink.h").is_file():
            return f"-I{d}"
    raise RuntimeError("MAVLink headers не найдены")


def run_cmd(cmd: list[str], *, capture=True) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=False,
    )


def build_recorder():
    try:
        put("state", "ПОДГОТОВКА")
        put("headline", "Подготовка теста…")
        put("instruction", "Собираю recorder. Дрон пока НЕ ДВИГАТЬ.")
        mav = find_mavlink_inc()
        cflags = subprocess.check_output(["pkg-config", "--cflags", "opencv4"], text=True).strip().split()
        libs = subprocess.check_output(["pkg-config", "--libs", "opencv4"], text=True).strip().split()
        cmd = [
            "g++", "-std=c++17", "-O2", "-DNDEBUG", "-pthread",
            "-Wno-address-of-packed-member",
            *cflags, mav,
            str(ROOT / "tools/optical_flow_feature_replay_record.cpp"),
            "-o", REC, *libs, "-lpthread",
        ]
        with BUILD_LOG.open("w") as log:
            p = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, text=True)
        if p.returncode != 0:
            tail = "\n".join(BUILD_LOG.read_text(errors="replace").splitlines()[-40:])
            raise RuntimeError("Ошибка сборки recorder:\n" + tail)
        if not Path(CAMERA).exists():
            raise RuntimeError(f"Камера не найдена: {CAMERA}")
        OUT.mkdir(parents=True, exist_ok=True)
        put("ready")
    except Exception as e:
        put("error", str(e))


def stream_process(cmd: list[str], tag: str):
    global active_proc
    active_proc = subprocess.Popen(
        cmd,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert active_proc.stdout is not None
    for line in active_proc.stdout:
        put(tag, line.rstrip())
    rc = active_proc.wait()
    active_proc = None
    return rc


def recording_worker():
    try:
        put("state", "ЗАПИСЬ")
        put("phase", "СПОКОЙНО")
        put("headline", "ДЕРЖИТЕ ДРОН НЕПОДВИЖНО")
        put("instruction", "Идёт запись. Первые 2 секунды — аппарат спокойно.")
        put("countdown", "2")
        global active_proc
        active_proc = subprocess.Popen(
            [REC, CAMERA, str(OUT), "15"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        t0 = time.monotonic()
        last = None
        while True:
            elapsed = time.monotonic() - t0
            if elapsed < 2:
                remain = max(0, 2 - int(elapsed))
                phase = ("СПОКОЙНО", "ДЕРЖИТЕ ДРОН НЕПОДВИЖНО",
                         "Запись уже идёт. Пока не двигать.", str(remain))
            elif elapsed < 12:
                remain = max(0, 12 - int(elapsed))
                phase = ("ДВИЖЕНИЕ", "ДВИГАЙТЕ ДРОН СЕЙЧАС",
                         "Переносы, изменение высоты, roll/pitch/yaw.\n"
                         "Сделайте несколько быстрых, но контролируемых движений.\n"
                         "Камеру рукой не закрывать.", str(remain))
            elif elapsed < 15:
                remain = max(0, 15 - int(elapsed))
                phase = ("ФИНИШ", "ОСТАНОВИТЕ ДРОН",
                         "Последние 3 секунды держите аппарат неподвижно.", str(remain))
            else:
                break

            if phase != last:
                put("phase", phase[0])
                put("headline", phase[1])
                put("instruction", phase[2])
                put("countdown", phase[3])
                last = phase
            time.sleep(0.08)

        rc = active_proc.wait()
        active_proc = None
        if rc != 0:
            raise RuntimeError(f"Recorder завершился с кодом {rc}")

        put("state", "АНАЛИЗ")
        put("phase", "ГОТОВО")
        put("headline", "ЗАПИСЬ ЗАВЕРШЕНА")
        put("instruction", "Дрон больше двигать НЕ НУЖНО.\nАнализирую одинаковые тяжёлые пары для 500 / 300 / 200.")
        put("countdown", "")

        rc = stream_process(
            ["bash", str(ROOT / "tools/run_optical_flow_heavy_pair_feature_replay.sh"), str(OUT), "25"],
            "log",
        )
        if rc != 0:
            raise RuntimeError(f"Heavy-pair replay завершился с кодом {rc}")
        put("done", str(OUT))
    except Exception as e:
        put("error", str(e))


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("JT-ZERO — Решающий тест Feature Cap")
        root.geometry("1050x760")
        root.minsize(900, 650)

        self.state_var = tk.StringVar(value="ПОДГОТОВКА")
        self.phase_var = tk.StringVar(value="")
        self.headline_var = tk.StringVar(value="Подготовка теста…")
        self.instruction_var = tk.StringVar(value="Дрон пока НЕ ДВИГАТЬ.")
        self.countdown_var = tk.StringVar(value="")

        top = tk.Frame(root, padx=24, pady=18)
        top.pack(fill="x")

        tk.Label(top, text="JT-ZERO — РЕШАЮЩИЙ ТЕСТ FEATURE CAP 500 / 300 / 200",
                 font=("DejaVu Sans", 20, "bold")).pack(anchor="w")
        tk.Label(top, textvariable=self.state_var, font=("DejaVu Sans", 12, "bold")).pack(anchor="w", pady=(8, 0))

        card = tk.Frame(root, padx=28, pady=24, relief="groove", bd=2)
        card.pack(fill="x", padx=24, pady=8)
        tk.Label(card, textvariable=self.headline_var,
                 font=("DejaVu Sans", 30, "bold"), wraplength=900, justify="left").pack(anchor="w")
        tk.Label(card, textvariable=self.instruction_var,
                 font=("DejaVu Sans", 16), wraplength=920, justify="left").pack(anchor="w", pady=(12, 4))
        tk.Label(card, textvariable=self.countdown_var,
                 font=("DejaVu Sans", 44, "bold")).pack(anchor="center", pady=(8, 0))

        info = (
            "Протокол: 2 с спокойно → 10 с движение → 3 с спокойно.\n"
            "FC и TF-Luna не нужны. Маршрут повторять не нужно.\n"
            "После записи GUI сам запустит анализ. Во время анализа дрон не двигать."
        )
        tk.Label(root, text=info, font=("DejaVu Sans", 13), justify="left",
                 wraplength=980).pack(anchor="w", padx=28, pady=(8, 10))

        buttons = tk.Frame(root)
        buttons.pack(fill="x", padx=24)
        self.start_btn = tk.Button(
            buttons, text="НАЧАТЬ ТЕСТ", font=("DejaVu Sans", 18, "bold"),
            state="disabled", command=self.start_test, padx=28, pady=10,
        )
        self.start_btn.pack(side="left")
        tk.Button(buttons, text="ЗАКРЫТЬ", font=("DejaVu Sans", 14),
                  command=self.close, padx=20, pady=10).pack(side="right")

        self.log = scrolledtext.ScrolledText(root, height=13, font=("DejaVu Sans Mono", 10))
        self.log.pack(fill="both", expand=True, padx=24, pady=16)
        self.log.insert("end", "Подготовка…\n")
        self.log.configure(state="disabled")

        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(80, self.poll_events)
        threading.Thread(target=build_recorder, daemon=True).start()

    def append_log(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def start_test(self):
        self.start_btn.configure(state="disabled")
        threading.Thread(target=recording_worker, daemon=True).start()

    def poll_events(self):
        try:
            while True:
                kind, text = events.get_nowait()
                if kind == "state":
                    self.state_var.set(text)
                elif kind == "phase":
                    self.phase_var.set(text)
                elif kind == "headline":
                    self.headline_var.set(text)
                elif kind == "instruction":
                    self.instruction_var.set(text)
                elif kind == "countdown":
                    self.countdown_var.set(text)
                elif kind == "ready":
                    self.state_var.set("ГОТОВО К ЗАПУСКУ")
                    self.headline_var.set("НАЖМИТЕ «НАЧАТЬ ТЕСТ»")
                    self.instruction_var.set("После нажатия запись начнётся сразу.\n"
                                             "Первые 2 секунды дрон держать неподвижно.")
                    self.countdown_var.set("")
                    self.start_btn.configure(state="normal")
                    self.append_log("Подготовка завершена. Нажмите «НАЧАТЬ ТЕСТ».")
                elif kind == "log":
                    self.append_log(text)
                elif kind == "done":
                    self.state_var.set("ЗАВЕРШЕНО")
                    self.headline_var.set("ТЕСТ ЗАВЕРШЁН")
                    self.instruction_var.set("Анализ завершён. Дрон можно поставить.")
                    self.countdown_var.set("")
                    self.append_log(f"Dataset: {text}")
                    messagebox.showinfo("JT-ZERO", "Тест и анализ завершены.")
                elif kind == "error":
                    self.state_var.set("ОШИБКА")
                    self.headline_var.set("ТЕСТ ОСТАНОВЛЕН")
                    self.instruction_var.set(text)
                    self.countdown_var.set("")
                    self.append_log("ОШИБКА: " + text)
                    messagebox.showerror("JT-ZERO — ошибка", text)
        except queue.Empty:
            pass
        self.root.after(80, self.poll_events)

    def close(self):
        global stop_requested, active_proc
        stop_requested = True
        if active_proc is not None and active_proc.poll() is None:
            try:
                active_proc.terminate()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
