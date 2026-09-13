#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext, messagebox
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KIMERA_ROOT = Path(os.environ.get("KIMERA_ROOT", "/home/vio/Kimera-VIO"))
DEVICE = os.environ.get("JTZERO_FLOW_FC", "/dev/ttyAMA0")
BAUD = os.environ.get("JTZERO_FLOW_FC_BAUD", "460800")
SYSID = os.environ.get("JTZERO_FC_SYSID", "1")
COMPID = os.environ.get("JTZERO_FC_COMPID", "1")
BIN = "/tmp/jtzero_fc_param_batch_checked"
SRC = ROOT / "tools/fc_param_batch_checked.cpp"

PARAMS = [
    "FS_THR_ENABLE",
    "FS_THR_VALUE",
    "FS_OPTIONS",
    "RC_OPTIONS",
    "RC_PROTOCOLS",
    "RC_FS_TIMEOUT",
    "FS_GCS_ENABLE",
    "FS_GCS_TIMEOUT",
    "ARMING_CHECK",
    "ARMING_SKIPCHK",
]

def find_mavlink_inc() -> str:
    for d in [
        KIMERA_ROOT / "third_party/mavlink",
        KIMERA_ROOT / "third_party/mavlink/include/mavlink/v2.0",
        Path("/usr/local/include/mavlink/v2.0"),
    ]:
        if (d / "common/mavlink.h").is_file() and (d / "ardupilotmega/mavlink.h").is_file():
            return f"-I{d}"
    raise RuntimeError("MAVLink headers не найдены")

def build():
    inc = find_mavlink_inc()
    p = subprocess.run(
        ["g++", "-std=c++17", "-O2", "-DNDEBUG", "-Wno-address-of-packed-member",
         inc, str(SRC), "-o", BIN],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    if p.returncode != 0:
        raise RuntimeError("Ошибка сборки:\n" + p.stdout[-5000:])

def read_one(name: str) -> tuple[str, str]:
    p = subprocess.run(
        [BIN, DEVICE, BAUD, SYSID, COMPID, "read", name],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    if p.returncode != 0:
        return name, "НЕ ПРОЧИТАН / НЕТ ПАРАМЕТРА"
    for line in p.stdout.splitlines():
        if line.startswith(name + "="):
            return name, line.split("=", 1)[1].strip()
    return name, "НЕ НАЙДЕН В ОТВЕТЕ"

class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("JT-ZERO — RC / Failsafe параметры")
        root.geometry("840x640")
        root.minsize(760, 560)

        tk.Label(
            root,
            text="JT-ZERO — RC / FAILSAFE ПАРАМЕТРЫ FC",
            font=("DejaVu Sans", 20, "bold")
        ).pack(anchor="w", padx=22, pady=(18, 4))

        self.status = tk.StringVar(value="Готов к чтению")
        tk.Label(root, textvariable=self.status, font=("DejaVu Sans", 12, "bold")).pack(
            anchor="w", padx=22, pady=(0, 8)
        )

        tk.Label(
            root,
            text=f"FC: {DEVICE} @ {BAUD}   sys={SYSID} comp={COMPID}\n"
                 "Только чтение. Параметры FC не изменяются.",
            font=("DejaVu Sans", 11),
            justify="left"
        ).pack(anchor="w", padx=22, pady=(0, 10))

        self.out = scrolledtext.ScrolledText(root, font=("DejaVu Sans Mono", 13), wrap="none")
        self.out.pack(fill="both", expand=True, padx=22, pady=10)

        self.btn = tk.Button(
            root, text="ПРОЧИТАТЬ ПАРАМЕТРЫ",
            font=("DejaVu Sans", 14, "bold"),
            command=self.start, padx=18, pady=8
        )
        self.btn.pack(pady=(0, 16))

        root.after(250, self.start)

    def start(self):
        self.btn.configure(state="disabled")
        self.status.set("Читаю параметры…")
        self.out.delete("1.0", "end")
        threading.Thread(target=self.worker, daemon=True).start()

    def worker(self):
        try:
            build()
            rows = [read_one(p) for p in PARAMS]
            text = []
            text.append("===== JT-ZERO — RC / FAILSAFE PARAM AUDIT =====")
            text.append("")
            for k, v in rows:
                text.append(f"{k:<18} = {v}")
            text.append("")
            text.append("РЕЖИМ: READ ONLY — ничего не изменено.")
            self.root.after(0, lambda: self.finish("\n".join(text)))
        except Exception as e:
            self.root.after(0, lambda: self.fail(str(e)))

    def finish(self, text: str):
        self.out.insert("end", text)
        self.status.set("ГОТОВО")
        self.btn.configure(state="normal")

    def fail(self, msg: str):
        self.out.insert("end", "ОШИБКА:\n" + msg)
        self.status.set("ОШИБКА")
        self.btn.configure(state="normal")
        messagebox.showerror("JT-ZERO", msg)

def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
