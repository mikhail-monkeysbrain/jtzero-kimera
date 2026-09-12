#!/usr/bin/env python3
# JT-Zero — continuous reciprocal OpticalFlow GUI.
# One camera/MAVLink/DataFlash process for all legs.

from __future__ import annotations
import csv, json, math, os, pty, queue, re, signal, statistics, subprocess, threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox

ROOT=Path(__file__).resolve().parent.parent
RUNS_ROOT=Path(os.environ.get("JTZERO_RUNS_ROOT","/home/vio/jtzero_runs"))
COUNT=int(os.environ.get("JTZERO_GUI_RUNS","6"))
FOCAL=os.environ.get("JTZERO_FLOW_FOCAL_SCALE","1.1060")
NOMINAL=os.environ.get("JTZERO_FLOW_TARGET_MM","300")

def fv(r,k,d=0.0):
    try: return float(r.get(k,d) or d)
    except Exception: return d

def analyze_leg(csv_path:Path, leg:int):
    rows=list(csv.DictReader(csv_path.open(newline="")))
    rows=[r for r in rows if int(fv(r,"guide_leg"))==leg]
    if not rows: raise RuntimeError(f"нет строк guide_leg={leg}")
    mags=[math.hypot(fv(r,"flow_body_x"),fv(r,"flow_body_y")) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=0.03 and int(fv(rows[i],"valid"))==1]
    if not idx: raise RuntimeError("не найдено движение")
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)

    dx=dy=0.0
    fresh=[]
    for i in range(i0,i1+1):
        r=rows[i]; dt=fv(r,"dt_s")
        if 0<dt<0.2 and int(fv(r,"valid"))==1:
            h=fv(r,"luna_m")
            if 0.05<h<20:
                dx+=h*fv(r,"flow_body_x")*dt
                dy+=h*fv(r,"flow_body_y")*dt
        if int(fv(r,"ekf_local_valid"))==1:
            fresh.append(i)

    raw=1000*math.hypot(dx,dy)
    ekf=float("nan")
    if fresh:
        a,b=rows[fresh[0]],rows[fresh[-1]]
        ekf=1000*math.hypot(fv(b,"ekf_x_ned")-fv(a,"ekf_x_ned"),
                            fv(b,"ekf_y_ned")-fv(a,"ekf_y_ned"))

    hagl=[-fv(r,"ekf_z_ned") for r in rows[i0:i1+1] if int(fv(r,"ekf_local_valid"))==1]
    hagl50=statistics.median(hagl) if hagl else float("nan")
    return {"raw_mm":raw,"ekf_mm":ekf,"hagl_local_p50_m":hagl50,
            "movement_row_start":i0,"movement_row_end":i1}

class Gui:
    def __init__(self,root):
        self.root=root; root.title("JT-Zero — Continuous Reciprocal OpticalFlow")
        root.geometry("1020x760")
        self.proc=None; self.fd=None; self.q=queue.Queue()
        self.csv=None; self.current_leg=0; self.results=[]; self.exit_rc=None

        tk.Label(root,text="JT-ZERO — CONTINUOUS RECIPROCAL OPTICAL FLOW",
                 font=("DejaVu Sans",21,"bold")).pack(pady=(16,6))
        self.stage=tk.Label(root,text="ПОДГОТОВКА",font=("DejaVu Sans",28,"bold"),fg="#174a7e")
        self.stage.pack(pady=5)
        self.ins=tk.Label(root,text="Один процесс камеры/MAVLink/DataFlash на всю серию.\n"
                                    "Направления A→B / B→A чередуются. Фактический сдвиг вводится после каждого прохода.",
                          font=("DejaVu Sans",15),justify="center",wraplength=950)
        self.ins.pack(pady=8)
        self.progress=tk.Label(root,text=f"0 / {COUNT}",font=("DejaVu Sans",14))
        self.progress.pack()

        ctl=tk.Frame(root); ctl.pack(pady=10)
        self.start=tk.Button(ctl,text="НАЧАТЬ СЕРИЮ",font=("DejaVu Sans",16,"bold"),
                             width=18,height=2,command=self.start_series)
        self.start.grid(row=0,column=0,padx=8)
        self.done=tk.Button(ctl,text="СДВИГ ЗАВЕРШЁН",font=("DejaVu Sans",16,"bold"),
                            width=18,height=2,state="disabled",command=self.finish_move)
        self.done.grid(row=0,column=1,padx=8)
        self.stop=tk.Button(ctl,text="ОСТАНОВИТЬ",font=("DejaVu Sans",13),
                            width=13,command=self.stop_series)
        self.stop.grid(row=0,column=2,padx=8)

        meas=tk.Frame(root); meas.pack(pady=8)
        tk.Label(meas,text="Фактический сдвиг, мм:",font=("DejaVu Sans",15)).grid(row=0,column=0,padx=6)
        self.entry=tk.Entry(meas,font=("DejaVu Sans",18),width=10,justify="center",state="disabled")
        self.entry.grid(row=0,column=1,padx=6)
        self.save=tk.Button(meas,text="СОХРАНИТЬ",font=("DejaVu Sans",13,"bold"),
                            state="disabled",command=self.save_measure)
        self.save.grid(row=0,column=2,padx=6)

        self.summary=tk.Label(root,text="",font=("DejaVu Sans",14,"bold"),justify="center")
        self.summary.pack(pady=8)
        self.log=tk.Text(root,height=17,font=("DejaVu Sans Mono",9),wrap="none")
        self.log.pack(fill="both",expand=True,padx=16,pady=(5,14))
        root.after(50,self.poll); root.protocol("WM_DELETE_WINDOW",self.close)

    def set_stage(self,title,text,color="#174a7e"):
        self.stage.config(text=title,fg=color); self.ins.config(text=text)

    def append(self,s):
        self.log.insert("end",s); self.log.see("end")

    def start_series(self):
        if self.proc and self.proc.poll() is None: return
        env=os.environ.copy()
        env["JTZERO_FLOW_FOCAL_SCALE"]=FOCAL
        env["JTZERO_FLOW_TARGET_MM"]=NOMINAL
        env["JTZERO_FLOW_TARGET_IS_NOMINAL"]="1"
        env["JTZERO_FLOW_GUIDED_MODE"]="armed-gate-open"
        env["JTZERO_FLOW_CONTINUOUS_LEGS"]=str(COUNT)
        master,slave=pty.openpty(); self.fd=master
        self.proc=subprocess.Popen(["bash","tools/run_optical_flow_mavlink_bench.sh"],cwd=str(ROOT),
                                   env=env,stdin=slave,stdout=slave,stderr=slave,
                                   start_new_session=True,close_fds=True)
        os.close(slave)
        threading.Thread(target=self.reader,daemon=True).start()
        self.start.config(state="disabled")
        self.set_stage("ЗАПУСК","Не двигать аппарат. Запускается единый continuous-контур.")

    def reader(self):
        buf=""
        sent_start=False
        while self.proc and self.proc.poll() is None:
            try: b=os.read(self.fd,4096)
            except OSError: break
            if not b: break
            s=b.decode("utf-8",errors="replace"); self.q.put(("log",s)); buf+=s
            if len(buf)>20000: buf=buf[-20000:]
            if "Запустить? [Enter]" in buf and not sent_start:
                sent_start=True; os.write(self.fd,b"\n"); buf=""
            m=re.findall(r"CSV:\s*(/[^\r\n ]*optical_flow_mavlink\.csv)",buf)
            if m: self.q.put(("csv",m[-1]))
            m=re.findall(r">>> LEG\s+(\d+)\s+ДВИГАЙТЕ",buf)
            if m: self.q.put(("move",int(m[-1]))); buf=""
            m=re.findall(r">>> LEG\s+(\d+)\s+СТОП",buf)
            if m: self.q.put(("post",int(m[-1]))); buf=""
            m=re.findall(r">>> LEG\s+(\d+)\s+COMPLETE",buf)
            if m: self.q.put(("complete",int(m[-1]))); buf=""
        rc=self.proc.wait() if self.proc else -1
        self.q.put(("exit",rc))

    def finish_move(self):
        if self.proc and self.proc.poll() is None:
            try: os.write(self.fd,b"\n")
            except OSError: return
            self.done.config(state="disabled")
            self.set_stage("НЕ ТРОГАТЬ","Финальная статика 5 секунд.","#9a5a00")

    def save_measure(self):
        try: physical=float(self.entry.get().strip().replace(",","."))
        except ValueError:
            messagebox.showerror("JT-Zero","Введите фактический сдвиг в мм."); return
        if not 20<=physical<=1000:
            messagebox.showerror("JT-Zero","Допустимо 20…1000 мм."); return
        if not self.csv or not self.csv.exists():
            messagebox.showerror("JT-Zero","CSV пока не найден."); return
        try: a=analyze_leg(self.csv,self.current_leg)
        except Exception as e:
            messagebox.showerror("JT-Zero",f"Анализ leg {self.current_leg}: {e}"); return

        direction="A->B" if self.current_leg%2 else "B->A"
        rec={"leg":self.current_leg,"direction":direction,"physical_measured_mm":physical,
             "csv":str(self.csv),"focal_scale":float(FOCAL),**a,
             "raw_ratio":a["raw_mm"]/physical,
             "ekf_ratio":a["ekf_mm"]/physical if math.isfinite(a["ekf_mm"]) else float("nan")}
        self.results.append(rec)
        self.summary.config(text=f"LEG {self.current_leg} {direction}: physical={physical:.1f} мм  "
                                 f"RAW={a['raw_mm']:.1f} ({rec['raw_ratio']:.4f}x)  "
                                 f"EKF={a['ekf_mm']:.1f} ({rec['ekf_ratio']:.4f}x)  "
                                 f"HAGL≈{a['hagl_local_p50_m']:.3f} м")
        self.entry.config(state="disabled"); self.save.config(state="disabled")
        if self.current_leg<COUNT:
            try: os.write(self.fd,b"\n")
            except OSError: pass
            self.set_stage("СЛЕДУЮЩИЙ ПРОХОД","Аппарат неподвижен. Continuous-процесс не перезапускается.")
        else:
            self.finish_summary()

    def finish_summary(self):
        if not self.results: return
        rr=[r["raw_ratio"] for r in self.results]
        er=[r["ekf_ratio"] for r in self.results if math.isfinite(r["ekf_ratio"])]
        rawm=statistics.mean(rr); raws=statistics.pstdev(rr) if len(rr)>1 else 0
        ekfm=statistics.mean(er); ekfs=statistics.pstdev(er) if len(er)>1 else 0
        ab=[r for r in self.results if r["direction"]=="A->B"]
        ba=[r for r in self.results if r["direction"]=="B->A"]
        def gm(g,k): return statistics.mean(r[k] for r in g) if g else float("nan")
        txt=(f"СЕРИЯ: RAW mean={rawm:.4f} SD={raws:.4f} ({(rawm-1)*100:+.2f}%)\n"
             f"EKF mean={ekfm:.4f} SD={ekfs:.4f} ({(ekfm-1)*100:+.2f}%)\n"
             f"A→B RAW={gm(ab,'raw_ratio'):.4f}  B→A RAW={gm(ba,'raw_ratio'):.4f}   "
             f"A→B EKF={gm(ab,'ekf_ratio'):.4f}  B→A EKF={gm(ba,'ekf_ratio'):.4f}")
        self.summary.config(text=txt)
        RUNS_ROOT.mkdir(parents=True,exist_ok=True)
        out=RUNS_ROOT/f"{datetime.now():%Y%m%d_%H%M%S}_OPTICAL_FLOW_CONTINUOUS_SERIES.json"
        out.write_text(json.dumps(self.results,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        self.append(f"\nSESSION={out}\n")
        self.set_stage("СЕРИЯ ЗАВЕРШЕНА","Все проходы записаны одним camera/MAVLink/DataFlash процессом.")

    def poll(self):
        while True:
            try: kind,val=self.q.get_nowait()
            except queue.Empty: break
            if kind=="log": self.append(val)
            elif kind=="csv": self.csv=Path(val)
            elif kind=="move":
                self.current_leg=val
                d="A→B" if val%2 else "B→A"
                self.progress.config(text=f"Прогон {val} / {COUNT}  {d}")
                self.set_stage("ДВИГАЙТЕ",f"LEG {val}: {d}. Сдвиньте стенд строго по столу.\n"
                                             "Точная длина не важна. После полной остановки нажмите «СДВИГ ЗАВЕРШЁН».","#0b7a28")
                self.done.config(state="normal")
            elif kind=="post":
                self.done.config(state="disabled")
                self.set_stage("НЕ ТРОГАТЬ","Финальная статика 5 секунд.","#9a5a00")
            elif kind=="complete":
                self.current_leg=val
                self.set_stage("ИЗМЕРЬТЕ СДВИГ",f"LEG {val} завершён. Измерьте фактическое расстояние и сохраните.","#6b2b8c")
                self.entry.delete(0,"end"); self.entry.config(state="normal"); self.save.config(state="normal"); self.entry.focus_set()
            elif kind=="exit":
                self.exit_rc=val
                if val!=0: self.set_stage("ОШИБКА",f"Процесс завершился с кодом {val}.","#a00000")
        self.root.after(50,self.poll)

    def stop_series(self):
        if self.proc and self.proc.poll() is None:
            try: os.killpg(self.proc.pid,signal.SIGINT)
            except ProcessLookupError: pass

    def close(self):
        self.stop_series(); self.root.after(150,self.root.destroy)

def main():
    if COUNT<2 or COUNT>30 or COUNT%2:
        raise SystemExit("JTZERO_GUI_RUNS должен быть чётным числом 2..30")
    root=tk.Tk(); Gui(root); root.mainloop()

if __name__=="__main__":
    main()
