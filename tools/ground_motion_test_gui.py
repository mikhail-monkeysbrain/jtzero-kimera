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

        self.title = tk.Label(root, text="JT-ZERO GROUND MOTION MVP", font=("DejaVu Sans", 30, "bold"), fg="white", bg="#111111")
        self.title.pack(pady=(30, 15))

        self.current_hdr = tk.Label(root, text="ТЕКУЩЕЕ ДЕЙСТВИЕ", font=("DejaVu Sans", 18, "bold"), fg="#aaaaaa", bg="#111111")
        self.current_hdr.pack()
        self.current = tk.Label(root, text="Подготовьте стенд и нажмите СТАРТ", font=("DejaVu Sans", 34, "bold"), fg="white", bg="#111111", wraplength=1500, justify="center")
        self.current.pack(pady=15)

        self.next_hdr = tk.Label(root, text="СЛЕДУЮЩЕЕ ДЕЙСТВИЕ", font=("DejaVu Sans", 16, "bold"), fg="#aaaaaa", bg="#111111")
        self.next_hdr.pack(pady=(15, 0))
        self.next = tk.Label(root, text="5 с статика A → движение 500 мм → 5 с статика B", font=("DejaVu Sans", 24), fg="white", bg="#111111", wraplength=1500, justify="center")
        self.next.pack(pady=10)

        self.status = tk.Label(root, text="Estimator: остановлен", font=("DejaVu Sans", 18), fg="white", bg="#111111")
        self.status.pack(pady=10)

        btn_frame = tk.Frame(root, bg="#111111")
        btn_frame.pack(pady=15)
        self.main_btn = tk.Button(btn_frame, text="СТАРТ ТЕСТА", font=("DejaVu Sans", 24, "bold"), width=24, height=2, command=self.main_action)
        self.main_btn.pack(side="left", padx=15)
        self.stop_btn = tk.Button(btn_frame, text="АВАРИЙНЫЙ СТОП", font=("DejaVu Sans", 20, "bold"), width=20, height=2, command=self.abort)
        self.stop_btn.pack(side="left", padx=15)

        self.log = scrolledtext.ScrolledText(root, height=12, font=("DejaVu Sans Mono", 12), bg="#1b1b1b", fg="white", insertbackground="white")
        self.log.pack(fill="both", expand=True, padx=30, pady=(10, 20))

        self.hint = tk.Label(root, text="F11 — полноэкранный режим   Esc — аварийный стоп", font=("DejaVu Sans", 13), fg="#888888", bg="#111111")
        self.hint.pack(pady=(0, 15))

        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", lambda e: self.abort())
        self.root.protocol("WM_DELETE_WINDOW", self.abort)
        self.root.after(50, self.poll)

    def toggle_fullscreen(self, _=None):
        self.root.attributes("-fullscreen", not bool(self.root.attributes("-fullscreen")))

    def set_text(self, current, next_text, button=None, enabled=True):
        self.current.config(text=current)
        self.next.config(text=next_text)
        if button is not None:
            self.main_btn.config(text=button)
        self.main_btn.config(state=("normal" if enabled else "disabled"))

    def start_estimator(self):
        self.proc = subprocess.Popen(
            ["bash", str(RUNNER)], cwd=str(ROOT),
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
            self.set_text("ДВИЖЕНИЕ A → B: ПЕРЕМЕСТИТЕ СТЕНД РОВНО НА 500 ММ", "Не вращать и не приподнимать. В точке B нажмите ДОСТИГ ТОЧКИ B", "ДОСТИГ ТОЧКИ B", True)
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

    def safe_quit(self):
        self.stop_estimator()
        self.root.after(300, self.root.destroy)

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
            self.current.config(text=f"СТАТИКА A — НЕ ДВИГАТЬ БПЛА   {left:0.1f} с")
            if left <= 0:
                self.stage = "READY_MOVE"
                self.countdown_end = None
                self.set_text("СТАТИКА A ЗАПИСАНА", "Нажмите НАЧАТЬ ДВИЖЕНИЕ и переместите стенд A → B ровно на 500 мм", "НАЧАТЬ ДВИЖЕНИЕ", True)
        elif self.stage == "STILL_B" and self.countdown_end is not None:
            left = max(0.0, self.countdown_end - now)
            self.current.config(text=f"СТАТИКА B — НЕ ДВИГАТЬ БПЛА   {left:0.1f} с")
            if left <= 0:
                self.countdown_end = None
                self.stop_estimator()
                self.stage = "DONE"
                extra = f"CSV: {self.csv_path}" if self.csv_path else "CSV указан в журнале ниже"
                self.set_text("ТЕСТ ЗАВЕРШЁН", extra, "ЗАКРЫТЬ", True)
                self.status.config(text="Estimator: отправлен SIGINT")

        self.root.after(50, self.poll)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
