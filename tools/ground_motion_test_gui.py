#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import queue
import time
import tkinter as tk
from tkinter import scrolledtext
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "tools" / "run_ground_motion_mvp.sh"
PREVIEW = Path("/dev/shm/jtzero_ground_motion_preview.pgm")

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("JT-ZERO Ground Motion MVP — тест 500 мм")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#111111")
        self.proc = None
        self.log_q = queue.Queue()
        self.csv_path = ""
        self.stage = "READY"
        self.countdown_end = None
        self.preview_img = None

        self.title = tk.Label(root, text="JT-ZERO GROUND MOTION MVP", font=("DejaVu Sans", 28, "bold"), fg="white", bg="#111111")
        self.title.pack(pady=(18, 10))

        body = tk.Frame(root, bg="#111111")
        body.pack(fill="both", expand=True, padx=20, pady=5)

        left = tk.Frame(body, bg="#111111")
        left.pack(side="left", fill="both", expand=True, padx=(0, 12))
        right = tk.Frame(body, bg="#111111", width=560)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)

        self.video = tk.Label(left, text="ВИДЕО ПОЯВИТСЯ ПОСЛЕ ЗАПУСКА", font=("DejaVu Sans", 20, "bold"), fg="white", bg="black")
        self.video.pack(fill="both", expand=True)

        self.current_hdr = tk.Label(right, text="ТЕКУЩЕЕ ДЕЙСТВИЕ", font=("DejaVu Sans", 16, "bold"), fg="#aaaaaa", bg="#111111")
        self.current_hdr.pack(pady=(5, 3))
        self.current = tk.Label(right, text="Подготовьте стенд и нажмите СТАРТ", font=("DejaVu Sans", 25, "bold"), fg="white", bg="#111111", wraplength=520, justify="center")
        self.current.pack(pady=10)

        self.next_hdr = tk.Label(right, text="СЛЕДУЮЩЕЕ ДЕЙСТВИЕ", font=("DejaVu Sans", 14, "bold"), fg="#aaaaaa", bg="#111111")
        self.next_hdr.pack(pady=(10, 2))
        self.next = tk.Label(right, text="5 с статика A → движение 500 мм → 5 с статика B", font=("DejaVu Sans", 18), fg="white", bg="#111111", wraplength=520, justify="center")
        self.next.pack(pady=8)

        self.status = tk.Label(right, text="Estimator: остановлен", font=("DejaVu Sans", 15), fg="white", bg="#111111", wraplength=520)
        self.status.pack(pady=8)

        self.main_btn = tk.Button(right, text="СТАРТ ТЕСТА", font=("DejaVu Sans", 20, "bold"), height=2, command=self.main_action)
        self.main_btn.pack(fill="x", padx=20, pady=(10, 8))
        self.stop_btn = tk.Button(right, text="АВАРИЙНЫЙ СТОП", font=("DejaVu Sans", 17, "bold"), height=2, command=self.abort)
        self.stop_btn.pack(fill="x", padx=20, pady=8)

        self.log = scrolledtext.ScrolledText(right, height=11, font=("DejaVu Sans Mono", 9), bg="#1b1b1b", fg="white", insertbackground="white")
        self.log.pack(fill="both", expand=True, padx=10, pady=(8, 8))

        self.hint = tk.Label(root, text="Esc — НЕМЕДЛЕННО ОСТАНОВИТЬ И ЗАКРЫТЬ   F11 — полноэкранный режим", font=("DejaVu Sans", 12, "bold"), fg="#bbbbbb", bg="#111111")
        self.hint.pack(pady=(4, 10))

        self.root.bind_all("<Escape>", self.escape_now)
        self.root.bind_all("<F11>", self.toggle_fullscreen)
        self.root.protocol("WM_DELETE_WINDOW", self.safe_quit)
        self.root.focus_force()
        self.root.after(50, self.poll)
        self.root.after(100, self.poll_preview)

    def toggle_fullscreen(self, _=None):
        self.root.attributes("-fullscreen", not bool(self.root.attributes("-fullscreen")))

    def set_text(self, current, next_text, button=None, enabled=True):
        self.current.config(text=current)
        self.next.config(text=next_text)
        if button is not None:
            self.main_btn.config(text=button)
        self.main_btn.config(state=("normal" if enabled else "disabled"))

    def start_estimator(self):
        try:
            PREVIEW.unlink()
        except FileNotFoundError:
            pass
        env = os.environ.copy()
        env["JTZERO_GM_PREVIEW"] = str(PREVIEW)
        self.proc = subprocess.Popen(
            ["bash", str(RUNNER)], cwd=str(ROOT), env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, preexec_fn=os.setsid
        )
        self.status.config(text=f"Estimator: запущен, PID {self.proc.pid}")
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            self.log_q.put(line)
        self.log_q.put("__PROC_DONE__")

    def main_action(self):
        if self.stage == "READY":
            self.start_estimator()
            self.stage = "STILL_A"
            self.countdown_end = time.monotonic() + 5.0
            self.set_text("СТАТИКА A — НЕ ДВИГАТЬ БПЛА", "После 5 секунд нажмите НАЧАТЬ ДВИЖЕНИЕ", "ПОДОЖДИТЕ...", False)
        elif self.stage == "READY_MOVE":
            self.stage = "MOVE"
            self.set_text("ДВИЖЕНИЕ A → B: РОВНО 500 ММ", "Не вращать и не приподнимать. В точке B нажмите ДОСТИГ ТОЧКИ B", "ДОСТИГ ТОЧКИ B", True)
        elif self.stage == "MOVE":
            self.stage = "STILL_B"
            self.countdown_end = time.monotonic() + 5.0
            self.set_text("СТАТИКА B — НЕ ДВИГАТЬ БПЛА", "Через 5 секунд тест завершится автоматически", "ПОДОЖДИТЕ...", False)
        elif self.stage in ("DONE", "ERROR"):
            self.safe_quit()

    def stop_estimator(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass

    def abort(self):
        self.stop_estimator()
        self.stage = "ERROR"
        self.set_text("ТЕСТ ОСТАНОВЛЕН", "Данные до остановки сохранены в CSV", "ЗАКРЫТЬ", True)
        self.status.config(text="Estimator: остановка")

    def escape_now(self, _=None):
        self.safe_quit()

    def safe_quit(self):
        self.stop_estimator()
        try:
            PREVIEW.unlink()
        except FileNotFoundError:
            pass
        self.root.after(150, self.root.destroy)

    def poll_preview(self):
        if PREVIEW.exists():
            try:
                img = tk.PhotoImage(file=str(PREVIEW))
                self.preview_img = img
                self.video.config(image=img, text="")
            except tk.TclError:
                pass
        self.root.after(100, self.poll_preview)

    def poll(self):
        try:
            while True:
                line = self.log_q.get_nowait()
                if line == "__PROC_DONE__":
                    rc = self.proc.poll() if self.proc else None
                    self.status.config(text=f"Estimator: завершён, код {rc}")
                    continue
                self.log.insert("end", line)
                self.log.see("end")
                if line.startswith("csv="):
                    self.csv_path = line.strip()[4:]
        except queue.Empty:
            pass

        now = time.monotonic()
        if self.stage == "STILL_A" and self.countdown_end is not None:
            left = max(0.0, self.countdown_end - now)
            self.current.config(text=f"СТАТИКА A — НЕ ДВИГАТЬ   {left:0.1f} с")
            if left <= 0:
                self.stage = "READY_MOVE"
                self.countdown_end = None
                self.set_text("СТАТИКА A ЗАПИСАНА", "Нажмите НАЧАТЬ ДВИЖЕНИЕ и переместите стенд A → B ровно на 500 мм", "НАЧАТЬ ДВИЖЕНИЕ", True)
        elif self.stage == "STILL_B" and self.countdown_end is not None:
            left = max(0.0, self.countdown_end - now)
            self.current.config(text=f"СТАТИКА B — НЕ ДВИГАТЬ   {left:0.1f} с")
            if left <= 0:
                self.countdown_end = None
                self.stop_estimator()
                self.stage = "DONE"
                extra = f"CSV: {self.csv_path}" if self.csv_path else "CSV указан в журнале"
                self.set_text("ТЕСТ ЗАВЕРШЁН", extra, "ЗАКРЫТЬ", True)
                self.status.config(text="Estimator: отправлен SIGINT")

        self.root.after(50, self.poll)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
