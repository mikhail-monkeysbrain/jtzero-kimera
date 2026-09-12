#!/usr/bin/env python3
# JT-Zero OpticalFlow bench GUI.
# Управляет существующим guided-тестом через PTY, показывает инструкции крупно,
# после прогона просит ввести ФАКТИЧЕСКИ измеренный сдвиг и сохраняет его рядом с CSV.

from __future__ import annotations
import csv
import json
import math
import os
import pty
import queue
import re
import signal
import statistics
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import messagebox
except Exception as e:
    raise SystemExit(
        "ОШИБКА: Python tkinter недоступен. Установите пакет python3-tk и повторите запуск. "
        f"Подробности: {e}"
    )

ROOT = Path(__file__).resolve().parent.parent
RUNS_ROOT = Path(os.environ.get("JTZERO_RUNS_ROOT", "/home/vio/jtzero_runs"))
COUNT = int(os.environ.get("JTZERO_GUI_RUNS", "5"))
FOCAL_SCALE = os.environ.get("JTZERO_FLOW_FOCAL_SCALE", "1.0000")
NOMINAL_MM = os.environ.get("JTZERO_FLOW_TARGET_MM", "300")
FLOW_THRESHOLD = 0.03
RECIPROCAL = os.environ.get("JTZERO_GUI_RECIPROCAL", "0").lower() in ("1","true","yes","on")


def fv(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default


def analyze_csv(path: Path):
    rows = list(csv.DictReader(path.open(newline="")))
    if not rows:
        raise RuntimeError("CSV пуст")

    mags = [math.hypot(fv(r, "flow_body_x"), fv(r, "flow_body_y")) for r in rows]
    idx = [i for i, m in enumerate(mags) if m >= FLOW_THRESHOLD and int(fv(rows[i], "valid")) == 1]
    if not idx:
        raise RuntimeError("Не найдено движение по raw flow")

    i0 = max(1, min(idx) - 5)
    i1 = min(len(rows) - 1, max(idx) + 5)

    dx = dy = 0.0
    fdx = fdy = 0.0
    have_fc = all(k in rows[0] for k in ("range_to_fc_m", "flow_send_x", "flow_send_y"))

    for i in range(i0, i1 + 1):
        r = rows[i]
        dt = fv(r, "dt_s")
        if not (0 < dt < 0.2 and int(fv(r, "valid")) == 1):
            continue
        h = fv(r, "luna_m")
        if 0.05 < h < 20:
            dx += h * fv(r, "flow_body_x") * dt
            dy += h * fv(r, "flow_body_y") * dt
        if have_fc:
            hf = fv(r, "range_to_fc_m")
            if 0.05 < hf < 20:
                fdx += hf * fv(r, "flow_send_x") * dt
                fdy += hf * fv(r, "flow_send_y") * dt

    raw_mm = math.hypot(dx, dy) * 1000.0
    fc_mm = math.hypot(fdx, fdy) * 1000.0 if have_fc else float("nan")

    fresh = [i for i in range(i0, i1 + 1) if int(fv(rows[i], "ekf_local_valid")) == 1]
    ekf_mm = float("nan")
    if fresh:
        a, b = rows[fresh[0]], rows[fresh[-1]]
        dn = fv(b, "ekf_x_ned") - fv(a, "ekf_x_ned")
        de = fv(b, "ekf_y_ned") - fv(a, "ekf_y_ned")
        ekf_mm = math.hypot(dn, de) * 1000.0

    return {
        "raw_mm": raw_mm,
        "fc_presented_mm": fc_mm,
        "ekf_mm": ekf_mm,
        "movement_row_start": i0,
        "movement_row_end": i1,
    }


class BenchGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("JT-Zero — OpticalFlow bench")
        self.root.geometry("1050x720")
        self.root.minsize(900, 620)

        self.proc = None
        self.master_fd = None
        self.reader_thread = None
        self.q = queue.Queue()
        self.run_index = 0
        self.results = []
        self.current_csv = None
        self.current_ekf_console_mm = None
        self.sent_start_enter = False
        self.sent_move_enter = False
        self.current_direction = None

        gui_title = "JT-ZERO — RECIPROCAL OPTICAL FLOW" if RECIPROCAL else "JT-ZERO — СТЕНДОВЫЙ ТЕСТ OPTICAL FLOW"
        self.title = tk.Label(root, text=gui_title,
                              font=("DejaVu Sans", 22, "bold"))
        self.title.pack(pady=(18, 8))

        self.stage = tk.Label(root, text="ПОДГОТОВКА", font=("DejaVu Sans", 28, "bold"),
                              fg="#174a7e")
        self.stage.pack(pady=6)

        self.instruction = tk.Label(
            root,
            text=(
                "Пропеллеры сняты. FC должен быть ARMED.\n"
                "Точность попадания в заданное расстояние НЕ нужна.\n"
                + ("После каждого прохода измерьте фактический сдвиг линейкой.\n"
                   "Направления будут чередоваться A→B / B→A."
                   if RECIPROCAL
                   else "После каждого прохода измерьте фактический сдвиг линейкой.")
            ),
            font=("DejaVu Sans", 16),
            justify="center",
            wraplength=940,
        )
        self.instruction.pack(pady=12)

        self.progress = tk.Label(root, text=f"Прогон 0 / {COUNT}", font=("DejaVu Sans", 14))
        self.progress.pack(pady=3)

        controls = tk.Frame(root)
        controls.pack(pady=12)

        self.start_btn = tk.Button(controls, text="НАЧАТЬ ПРОГОН", font=("DejaVu Sans", 16, "bold"),
                                   width=20, height=2, command=self.start_run)
        self.start_btn.grid(row=0, column=0, padx=8)

        self.move_done_btn = tk.Button(controls, text="СДВИГ ЗАВЕРШЁН", font=("DejaVu Sans", 16, "bold"),
                                       width=20, height=2, state="disabled", command=self.finish_move)
        self.move_done_btn.grid(row=0, column=1, padx=8)

        self.stop_btn = tk.Button(controls, text="ОСТАНОВИТЬ", font=("DejaVu Sans", 13),
                                  width=14, command=self.stop_process)
        self.stop_btn.grid(row=0, column=2, padx=8)

        measure = tk.Frame(root)
        measure.pack(pady=8)
        tk.Label(measure, text="Фактический сдвиг, мм:", font=("DejaVu Sans", 15)).grid(row=0, column=0, padx=6)
        self.actual_entry = tk.Entry(measure, font=("DejaVu Sans", 18), width=10, justify="center", state="disabled")
        self.actual_entry.grid(row=0, column=1, padx=6)
        self.save_btn = tk.Button(measure, text="СОХРАНИТЬ ИЗМЕРЕНИЕ", font=("DejaVu Sans", 13, "bold"),
                                  state="disabled", command=self.save_measurement)
        self.save_btn.grid(row=0, column=2, padx=8)

        self.result_label = tk.Label(root, text="", font=("DejaVu Sans", 15, "bold"), justify="center")
        self.result_label.pack(pady=8)

        log_frame = tk.Frame(root)
        log_frame.pack(fill="both", expand=True, padx=18, pady=(8, 14))
        tk.Label(log_frame, text="Технический лог", font=("DejaVu Sans", 11)).pack(anchor="w")
        self.log = tk.Text(log_frame, height=12, font=("DejaVu Sans Mono", 9), wrap="none")
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.poll_queue)

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_stage(self, title, text, color="#174a7e"):
        self.stage.config(text=title, fg=color)
        self.instruction.config(text=text)

    def start_run(self):
        if self.proc and self.proc.poll() is None:
            return
        if self.run_index >= COUNT:
            return

        self.run_index += 1
        self.current_direction = "A->B" if (self.run_index % 2 == 1) else "B->A"
        dir_text = f"  {self.current_direction}" if RECIPROCAL else ""
        self.progress.config(text=f"Прогон {self.run_index} / {COUNT}{dir_text}")
        self.current_csv = None
        self.current_ekf_console_mm = None
        self.sent_start_enter = False
        self.sent_move_enter = False
        self.actual_entry.config(state="disabled")
        self.actual_entry.delete(0, "end")
        self.save_btn.config(state="disabled")
        self.start_btn.config(state="disabled")
        self.move_done_btn.config(state="disabled")
        self.result_label.config(text="")
        launch_text = "Аппарат НЕ ДВИГАТЬ.\nПрограмма запускает камеру, TF-Luna, MAVLink и EKF logging."
        if RECIPROCAL:
            launch_text += f"\nТекущий измеряемый проход: {self.current_direction}."
        self.set_stage("ЗАПУСК", launch_text)

        env = os.environ.copy()
        env["JTZERO_FLOW_FOCAL_SCALE"] = FOCAL_SCALE
        env["JTZERO_FLOW_TARGET_MM"] = NOMINAL_MM
        env["JTZERO_FLOW_GUIDED_MODE"] = "armed-gate-open"

        master, slave = pty.openpty()
        self.master_fd = master
        self.proc = subprocess.Popen(
            ["bash", "tools/run_optical_flow_mavlink_bench.sh"],
            cwd=str(ROOT),
            env=env,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            start_new_session=True,
            close_fds=True,
        )
        os.close(slave)
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def reader_loop(self):
        buf = ""
        while self.proc and self.proc.poll() is None:
            try:
                b = os.read(self.master_fd, 2048)
                if not b:
                    break
                s = b.decode("utf-8", errors="replace")
                self.q.put(("log", s))
                buf += s
                if len(buf) > 12000:
                    buf = buf[-12000:]

                if ("Запустить? [Enter]" in buf) and not self.sent_start_enter:
                    self.sent_start_enter = True
                    os.write(self.master_fd, b"\n")
                    buf = ""

                if "СТАТИКА 5 секунд. НЕ ДВИГАТЬ." in buf:
                    self.q.put(("stage_static", None))
                    buf = ""

                if ">>> ДВИГАЙТЕ:" in buf:
                    self.q.put(("stage_move", None))
                    buf = ""

                if ">>> СТОП. НЕ ТРОГАТЬ" in buf:
                    self.q.put(("stage_post", None))
                    buf = ""

                m = re.findall(r"CSV:\s*(/[^\r\n ]*optical_flow_mavlink\.csv)", buf)
                if m:
                    self.q.put(("csv", m[-1]))

                e = re.findall(r"EKF horizontal displacement\s*=\s*([0-9.+-]+)\s*mm", buf)
                if e:
                    try:
                        self.q.put(("ekf_console", float(e[-1])))
                    except ValueError:
                        pass
            except OSError:
                break

        rc = self.proc.wait() if self.proc else -1
        self.q.put(("exit", rc))

    def finish_move(self):
        if not self.proc or self.proc.poll() is not None or self.sent_move_enter:
            return
        self.sent_move_enter = True
        try:
            os.write(self.master_fd, b"\n")
        except OSError:
            return
        self.move_done_btn.config(state="disabled")
        self.set_stage(
            "НЕ ТРОГАТЬ",
            "Сдвиг завершён.\nАппарат полностью неподвижен — идёт финальная статика 5 секунд.",
            "#9a5a00",
        )

    def stop_process(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def poll_queue(self):
        while True:
            try:
                kind, value = self.q.get_nowait()
            except queue.Empty:
                break

            if kind == "log":
                self.append_log(value)
            elif kind == "stage_static":
                self.set_stage("НЕ ДВИГАТЬ", "Стартовая статика 5 секунд.\nВообще не трогайте аппарат.", "#9a5a00")
            elif kind == "stage_move":
                move_prefix = f"Направление {self.current_direction}.\n" if RECIPROCAL else ""
                self.set_stage(
                    "ДВИГАЙТЕ",
                    move_prefix + "Сдвиньте ВЕСЬ аппарат строго по столу.\n"
                    "Точное расстояние сейчас НЕ важно. Не вращать, не наклонять, не приподнимать.\n"
                    "После полной остановки нажмите «СДВИГ ЗАВЕРШЁН».",
                    "#0b7a28",
                )
                self.move_done_btn.config(state="normal")
            elif kind == "stage_post":
                self.set_stage("НЕ ТРОГАТЬ", "Финальная статика 5 секунд. Аппарат полностью неподвижен.", "#9a5a00")
                self.move_done_btn.config(state="disabled")
            elif kind == "csv":
                self.current_csv = Path(value)
            elif kind == "ekf_console":
                self.current_ekf_console_mm = value
            elif kind == "exit":
                self.move_done_btn.config(state="disabled")
                if value == 0 and self.current_csv and self.current_csv.exists():
                    self.set_stage(
                        "ИЗМЕРЬТЕ СДВИГ",
                        "Теперь линейкой/рулеткой измерьте ФАКТИЧЕСКОЕ перемещение стенда.\n"
                        "Введите число в миллиметрах. Именно оно будет эталоном, а не приблизительная команда теста.",
                        "#6b2b8c",
                    )
                    self.actual_entry.config(state="normal")
                    self.save_btn.config(state="normal")
                    self.actual_entry.focus_set()
                else:
                    self.set_stage("ОШИБКА ТЕСТА", f"Процесс завершился с кодом {value}. Смотрите технический лог.", "#a00000")
                    self.start_btn.config(state="normal")

        self.root.after(50, self.poll_queue)

    def save_measurement(self):
        s = self.actual_entry.get().strip().replace(",", ".")
        try:
            actual = float(s)
        except ValueError:
            messagebox.showerror("JT-Zero", "Введите фактический сдвиг числом в миллиметрах.")
            return
        if not (20.0 <= actual <= 1000.0):
            messagebox.showerror("JT-Zero", "Допустимый стендовый диапазон: 20…1000 мм.")
            return
        if not self.current_csv:
            messagebox.showerror("JT-Zero", "CSV текущего прогона не найден.")
            return

        try:
            a = analyze_csv(self.current_csv)
        except Exception as e:
            messagebox.showerror("JT-Zero", f"Не удалось проанализировать CSV: {e}")
            return

        rec = {
            "run": self.run_index,
            "direction": self.current_direction if RECIPROCAL else None,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "csv": str(self.current_csv),
            "focal_scale": float(FOCAL_SCALE),
            "nominal_mm": float(NOMINAL_MM),
            "physical_measured_mm": actual,
            **a,
            "raw_error_pct": (a["raw_mm"] / actual - 1.0) * 100.0,
            "ekf_error_pct": (a["ekf_mm"] / actual - 1.0) * 100.0 if math.isfinite(a["ekf_mm"]) else float("nan"),
            "ekf_console_mm": self.current_ekf_console_mm,
        }

        sidecar = Path(str(self.current_csv) + ".measurement.json")
        sidecar.write_text(json.dumps(rec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.results.append(rec)

        self.result_label.config(
            text=(
                f"Физически: {actual:.1f} мм    RAW: {a['raw_mm']:.1f} мм ({rec['raw_error_pct']:+.2f}%)\n"
                f"EKF: {a['ekf_mm']:.1f} мм ({rec['ekf_error_pct']:+.2f}%)"
            )
        )
        self.actual_entry.config(state="disabled")
        self.save_btn.config(state="disabled")

        if self.run_index < COUNT:
            if RECIPROCAL:
                next_dir = "A->B" if ((self.run_index + 1) % 2 == 1) else "B->A"
                self.set_stage(
                    "СЛЕДУЮЩИЙ ПРОГОН",
                    f"НЕ возвращайте аппарат отдельно. Следующий ход {next_dir} сам является измеряемым возвратом.\n"
                    "Аппарат полностью остановить и нажать «НАЧАТЬ СЛЕДУЮЩИЙ ПРОГОН».",
                    "#174a7e",
                )
            else:
                self.set_stage(
                    "ВОЗВРАТ В ИСХОДНУЮ ТОЧКУ",
                    "Верните аппарат назад. Этот возврат НЕ измеряется.\n"
                    "Полностью остановите аппарат и нажмите «НАЧАТЬ ПРОГОН».",
                    "#174a7e",
                )
            self.start_btn.config(text="НАЧАТЬ СЛЕДУЮЩИЙ ПРОГОН", state="normal")
        else:
            self.finish_session()

    def finish_session(self):
        self.start_btn.config(state="disabled")
        self.set_stage("СЕРИЯ ЗАВЕРШЕНА", "Все измерения сохранены. Ниже — сводка серии.", "#174a7e")

        if not self.results:
            return

        raw_ratios = [r["raw_mm"] / r["physical_measured_mm"] for r in self.results]
        ekf_ratios = [r["ekf_mm"] / r["physical_measured_mm"] for r in self.results if math.isfinite(r["ekf_mm"])]
        raw_mean = statistics.mean(raw_ratios)
        raw_sd = statistics.pstdev(raw_ratios) if len(raw_ratios) > 1 else 0.0
        ekf_mean = statistics.mean(ekf_ratios) if ekf_ratios else float("nan")
        ekf_sd = statistics.pstdev(ekf_ratios) if len(ekf_ratios) > 1 else 0.0

        summary_text = (
            f"RAW/physical: mean={raw_mean:.4f}, SD={raw_sd:.4f}  "
            f"({(raw_mean-1)*100:+.2f}%)\n"
            f"EKF/physical: mean={ekf_mean:.4f}, SD={ekf_sd:.4f}  "
            f"({(ekf_mean-1)*100:+.2f}%)"
        )

        if RECIPROCAL:
            ab = [r for r in self.results if r.get("direction") == "A->B"]
            ba = [r for r in self.results if r.get("direction") == "B->A"]
            def ratio_mean(group, key):
                vals = [r[key] / r["physical_measured_mm"] for r in group if math.isfinite(r[key])]
                return statistics.mean(vals) if vals else float("nan")
            raw_ab = ratio_mean(ab, "raw_mm")
            raw_ba = ratio_mean(ba, "raw_mm")
            ekf_ab = ratio_mean(ab, "ekf_mm")
            ekf_ba = ratio_mean(ba, "ekf_mm")
            raw_bias = (raw_ab / raw_ba - 1.0) * 100.0 if raw_ba else float("nan")
            ekf_bias = (ekf_ab / ekf_ba - 1.0) * 100.0 if ekf_ba else float("nan")
            summary_text += (
                f"\nA→B RAW={raw_ab:.4f}  B→A RAW={raw_ba:.4f}  bias={raw_bias:+.2f}%"
                f"\nA→B EKF={ekf_ab:.4f}  B→A EKF={ekf_ba:.4f}  bias={ekf_bias:+.2f}%"
            )

        self.result_label.config(text=summary_text)

        RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        out = RUNS_ROOT / f"{datetime.now():%Y%m%d_%H%M%S}_OPTICAL_FLOW_GUI_SERIES.json"
        out.write_text(json.dumps(self.results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.append_log(f"\nSESSION={out}\n")

    def on_close(self):
        self.stop_process()
        self.root.after(150, self.root.destroy)


def main():
    if COUNT < 1 or COUNT > 30:
        raise SystemExit("JTZERO_GUI_RUNS должен быть 1..30")
    root = tk.Tk()
    BenchGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
