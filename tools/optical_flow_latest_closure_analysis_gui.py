#!/usr/bin/env python3
from __future__ import annotations
import glob,subprocess,tkinter as tk
from tkinter import scrolledtext,messagebox
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def latest_csv():
    xs=sorted(glob.glob("/home/vio/jtzero_runs/*_OPTICAL_FLOW_FLIGHT/optical_flow_mavlink.csv"))
    if not xs:
        raise RuntimeError("Не найден optical_flow_mavlink.csv в /home/vio/jtzero_runs")
    return xs[-1]

def main():
    root=tk.Tk()
    root.title("JT-ZERO — Анализ последнего 6-DoF прогона")
    root.geometry("1100x760")
    root.minsize(900,650)

    title=tk.Label(root,text="JT-ZERO — АНАЛИЗ ПОСЛЕДНЕГО 6-DoF ПРОГОНА",
                   font=("DejaVu Sans",20,"bold"))
    title.pack(anchor="w",padx=22,pady=(18,8))

    status=tk.StringVar(value="Ищу последний CSV…")
    tk.Label(root,textvariable=status,font=("DejaVu Sans",12,"bold")).pack(anchor="w",padx=22)

    box=scrolledtext.ScrolledText(root,font=("DejaVu Sans Mono",11),wrap="word")
    box.pack(fill="both",expand=True,padx=22,pady=14)

    def run():
        try:
            csv=latest_csv()
            status.set(f"Анализ: {csv}")
            p=subprocess.run(
                ["python3",str(ROOT/"tools/analyze_optical_flow_stale_displacement.py"),csv],
                cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,check=False
            )
            box.delete("1.0","end")
            box.insert("end",p.stdout)
            if p.returncode!=0:
                status.set("ОШИБКА АНАЛИЗА")
                messagebox.showerror("JT-ZERO","Анализ завершился с ошибкой. См. окно.")
            else:
                status.set("ГОТОВО — сравните SENT / STALE / ALL-VALID для A→H")
        except Exception as e:
            status.set("ОШИБКА")
            box.insert("end",str(e))
            messagebox.showerror("JT-ZERO",str(e))

    tk.Button(root,text="ПОВТОРИТЬ АНАЛИЗ",font=("DejaVu Sans",14,"bold"),
              command=run,padx=18,pady=8).pack(pady=(0,16))
    root.after(200,run)
    root.mainloop()

if __name__=="__main__":
    main()
