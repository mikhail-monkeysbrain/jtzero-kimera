#!/usr/bin/env python3
import argparse, csv, math, queue, statistics, threading, time
from collections import defaultdict
from dataclasses import dataclass
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

RAD_STREAMS = {"HIGHRES_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "ATTITUDE"}
WANTED = ["RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3", "HIGHRES_IMU", "ATTITUDE"]
MSG_IDS = {"RAW_IMU": 27, "SCALED_IMU": 26, "SCALED_IMU2": 116, "SCALED_IMU3": 129, "HIGHRES_IMU": 105, "ATTITUDE": 30}


def phase_for(t):
    if t < PRE_S: return "PRE"
    if t < PRE_S + YAW_S: return "YAW"
    return "POST"


def block_for(t):
    if 0 <= t < 5: return "PRE_EARLY"
    if 5 <= t < 10: return "PRE_MID"
    if 10 <= t < 15: return "PRE_LATE"
    if 25 <= t < 30: return "POST_EARLY"
    if 30 <= t < 35: return "POST_MID"
    if 35 <= t <= 40.5: return "POST_LATE"
    return ""


def gyro_rad(msg):
    t = msg.get_type()
    if t == "HIGHRES_IMU":
        return float(msg.xgyro), float(msg.ygyro), float(msg.zgyro)
    if t in ("SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
        return float(msg.xgyro) * 1e-3, float(msg.ygyro) * 1e-3, float(msg.zgyro) * 1e-3
    if t == "ATTITUDE":
        return float(msg.rollspeed), float(msg.pitchspeed), float(msg.yawspeed)
    return None


def mean(v): return statistics.fmean(v) if v else float("nan")
def sd(v): return statistics.stdev(v) if len(v) >= 2 else float("nan")


def request_interval(m, msgid, hz):
    try:
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, 1e6 / hz, 0, 0, 0, 0, 0)
    except Exception:
        pass


def analyze(rows):
    by = defaultdict(list)
    for r in rows:
        if r["gx"] is not None:
            by[r["type"]].append(r)
    out = {}
    for typ, rr in by.items():
        pre = [r for r in rr if r["phase"] == "PRE"]
        post = [r for r in rr if r["phase"] == "POST"]
        if not pre or not post: continue
        d = {"n_pre": len(pre), "n_post": len(post), "n_total": len(rr)}
        for a in "xyz":
            key = "g" + a
            pm, qm = mean([r[key] for r in pre]), mean([r[key] for r in post])
            d["pre_"+a], d["post_"+a], d["delta_"+a] = pm, qm, qm-pm
            d["pre_sd_"+a], d["post_sd_"+a] = sd([r[key] for r in pre]), sd([r[key] for r in post])
        dxy = math.hypot(d["delta_x"], d["delta_y"])
        d["delta_xy"] = dxy
        d["ratio"] = dxy / BASELINE_DXY
        blocks = {}
        for bn in ("PRE_EARLY","PRE_MID","PRE_LATE","POST_EARLY","POST_MID","POST_LATE"):
            q = [r for r in rr if r["block"] == bn]
            if q:
                blocks[bn] = tuple(mean([r["g"+a] for r in q]) for a in "xyz")
        d["blocks"] = blocks
        out[typ] = d
    return out


def save(rows, result, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csvp = out_dir / f"main_f405_ardupilot_imu_path_{stamp}.csv"
    txtp = out_dir / f"main_f405_ardupilot_imu_path_{stamp}.txt"
    with csvp.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_t","t_rel","phase","block","type","fc_time","gx_rad_s","gy_rad_s","gz_rad_s","raw_xgyro","raw_ygyro","raw_zgyro","roll","pitch","yaw"])
        for r in rows:
            w.writerow([f"{r['host_t']:.9f}",f"{r['t']:.6f}",r['phase'],r['block'],r['type'],r['fc_time'],
                        "" if r['gx'] is None else f"{r['gx']:.12f}","" if r['gy'] is None else f"{r['gy']:.12f}","" if r['gz'] is None else f"{r['gz']:.12f}",
                        r['raw_x'],r['raw_y'],r['raw_z'],r['roll'],r['pitch'],r['yaw']])
    with txtp.open("w") as f:
        f.write("JT-Zero P11/B11 ArduPilot IMU path test\n")
        f.write("Target: main F405_V2 / STM32F405 / ArduPilot / ICM42688\n")
        f.write("Sequence: 15 s STILL -> ~90 deg YAW -> 15 s STILL\n")
        f.write("Streams requested: RAW_IMU, SCALED_IMU, SCALED_IMU2, SCALED_IMU3, HIGHRES_IMU, ATTITUDE\n")
        f.write("Units: HIGHRES_IMU and ATTITUDE are rad/s; SCALED_IMU* converted mrad/s -> rad/s; RAW_IMU kept as raw values.\n\n")
        counts = defaultdict(int)
        for r in rows: counts[r['type']] += 1
        f.write("Message counts:\n")
        for k in WANTED: f.write(f"  {k}: {counts[k]}\n")
        f.write("\n")
        for typ, d in result.items():
            f.write(f"[{typ}] PRE={d['n_pre']} POST={d['n_post']} TOTAL={d['n_total']}\n")
            for a in "xyz":
                f.write(f"  {a.upper()}: PRE={d['pre_'+a]:+.9f} POST={d['post_'+a]:+.9f} DELTA={d['delta_'+a]:+.9f} rad/s SDpre={d['pre_sd_'+a]:.9f} SDpost={d['post_sd_'+a]:.9f}\n")
            f.write(f"  |DELTA XY|={d['delta_xy']:.9f} rad/s  ratio_to_baseline={d['ratio']:.3f}\n")
            for bn, vals in d['blocks'].items():
                f.write(f"  {bn}: X={vals[0]:+.9f} Y={vals[1]:+.9f} Z={vals[2]:+.9f}\n")
            f.write("\n")
    return csvp, txtp


class App:
    def __init__(self, root, port, baud):
        self.root, self.port, self.baud = root, port, baud
        self.m = None; self.rows = []; self.running = False; self.start_t = None; self.last_att = None
        self.events = queue.Queue(); self.seen = defaultdict(int)
        root.title("JT-Zero — ArduPilot IMU path test")
        root.geometry("980x650")
        tk.Label(root,text="Основной F405_V2 / ArduPilot / ICM42688",font=("DejaVu Sans",20,"bold")).pack(pady=(18,8))
        self.status=tk.Label(root,text="Готово",font=("DejaVu Sans",25,"bold")); self.status.pack(pady=8)
        self.instr=tk.Label(root,text="Положите полётник неподвижно на стол",font=("DejaVu Sans",16)); self.instr.pack(pady=4)
        self.timer=tk.Label(root,text="00.0 с",font=("DejaVu Sans Mono",42,"bold")); self.timer.pack(pady=10)
        self.att=tk.Label(root,text="ATTITUDE: ---",font=("DejaVu Sans Mono",15)); self.att.pack(pady=6)
        self.streams=tk.Label(root,text="Потоки: ---",font=("DejaVu Sans Mono",12),justify="left"); self.streams.pack(pady=6)
        self.btn=tk.Button(root,text="НАЧАТЬ ТЕСТ",font=("DejaVu Sans",16,"bold"),width=22,command=self.start); self.btn.pack(pady=14)
        self.result=tk.Label(root,text="",font=("DejaVu Sans Mono",12),justify="left",anchor="w"); self.result.pack(fill="x",padx=45,pady=8)
        root.protocol("WM_DELETE_WINDOW",self.close); root.after(50,self.tick)

    def connect(self):
        self.m = mavutil.mavlink_connection(self.port, baud=self.baud, autoreconnect=False)
        hb = self.m.wait_heartbeat(timeout=5)
        if hb is None: raise RuntimeError("HEARTBEAT timeout")
        for k in ("RAW_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3","HIGHRES_IMU"):
            request_interval(self.m, MSG_IDS[k], 50)
        request_interval(self.m, MSG_IDS["ATTITUDE"], 20)
        return self.m.target_system, self.m.target_component

    def start(self):
        if self.running: return
        try: sysid, compid = self.connect()
        except Exception as e:
            messagebox.showerror("Ошибка MAVLink",str(e)); return
        self.rows=[]; self.seen=defaultdict(int); self.start_t=time.monotonic(); self.running=True; self.btn.config(state="disabled")
        self.result.config(text=f"MAVLink подключён: SYS={sysid} COMP={compid}")
        threading.Thread(target=self.collect,daemon=True).start()

    def collect(self):
        try:
            while self.running:
                now=time.monotonic(); t=now-self.start_t
                if t >= TOTAL_S: break
                msg=self.m.recv_match(type=WANTED, blocking=True, timeout=0.25)
                if msg is None: continue
                typ=msg.get_type(); self.seen[typ]+=1
                g=gyro_rad(msg)
                gx=gy=gz=None
                if g: gx,gy,gz=g
                raw_x=getattr(msg,"xgyro","") if typ=="RAW_IMU" else ""
                raw_y=getattr(msg,"ygyro","") if typ=="RAW_IMU" else ""
                raw_z=getattr(msg,"zgyro","") if typ=="RAW_IMU" else ""
                fc_time=getattr(msg,"time_usec",getattr(msg,"time_boot_ms",""))
                roll=pitch=yaw=""
                if typ=="ATTITUDE":
                    roll,pitch,yaw=float(msg.roll),float(msg.pitch),float(msg.yaw); self.last_att=(roll,pitch,yaw)
                self.rows.append(dict(host_t=now,t=t,phase=phase_for(t),block=block_for(t),type=typ,fc_time=fc_time,gx=gx,gy=gy,gz=gz,
                                      raw_x=raw_x,raw_y=raw_y,raw_z=raw_z,roll=roll,pitch=pitch,yaw=yaw))
        except Exception as e: self.events.put(("error",str(e)))
        finally:
            self.running=False
            try:
                if self.m: self.m.close()
            except Exception: pass
            self.events.put(("done",None))

    def tick(self):
        try:
            while True:
                kind,data=self.events.get_nowait()
                if kind=="error": messagebox.showerror("Ошибка",data)
                elif kind=="done" and self.start_t is not None: self.finish()
        except queue.Empty: pass
        if self.running:
            t=min(time.monotonic()-self.start_t,TOTAL_S); p=phase_for(min(t,TOTAL_S-1e-6))
            if p=="PRE": self.status.config(text="ПОКОЙ PRE"); self.instr.config(text="НЕ ДВИГАТЬ полётник"); rem=PRE_S-t
            elif p=="YAW": self.status.config(text="ПОВОРОТ YAW ~90°"); self.instr.config(text="Плавно поверните только по YAW примерно на 90°, затем снова положите"); rem=PRE_S+YAW_S-t
            else: self.status.config(text="ПОКОЙ POST"); self.instr.config(text="НЕ ДВИГАТЬ полётник"); rem=TOTAL_S-t
            self.timer.config(text=f"{max(0,rem):04.1f} с")
            if self.last_att:
                r,pit,y=self.last_att; self.att.config(text=f"ATTITUDE deg: R {math.degrees(r):+.2f}  P {math.degrees(pit):+.2f}  Y {math.degrees(y):+.2f}")
            self.streams.config(text="Потоки: "+"  ".join(f"{k}:{self.seen[k]}" for k in WANTED))
        self.root.after(50,self.tick)

    def finish(self):
        if not self.rows: self.btn.config(state="normal"); return
        result=analyze(self.rows)
        out=Path(__file__).resolve().parent/"results"; csvp,txtp=save(self.rows,result,out)
        lines=["ТЕСТ ЗАВЕРШЁН",f"TXT: {txtp.name}"]
        for typ,d in result.items(): lines.append(f"{typ}: |ΔXY|={d['delta_xy']:.7f} rad/s ({d['ratio']:.3f}x baseline)")
        self.status.config(text="ТЕСТ ЗАВЕРШЁН"); self.instr.config(text="Результат сохранён. Полётник можно двигать.")
        self.timer.config(text="00.0 с"); self.result.config(text="\n".join(lines)); self.btn.config(state="normal"); self.start_t=None

    def close(self):
        self.running=False
        try:
            if self.m: self.m.close()
        except Exception: pass
        self.root.destroy()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",default="/dev/ttyACM0")
    ap.add_argument("--baud",type=int,default=115200)
    a=ap.parse_args()
    root=tk.Tk(); App(root,a.port,a.baud); root.mainloop()

if __name__=="__main__": main()
