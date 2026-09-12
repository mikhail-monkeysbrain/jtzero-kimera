#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, math, statistics
from pathlib import Path
import importlib.util

HERE = Path(__file__).resolve().parent
base_path = HERE / "analyze_optical_flow_continuous_dataflash.py"
spec = importlib.util.spec_from_file_location("ofdf", base_path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def num(r,k,d=float("nan")):
    try: return float(r[k])
    except Exception: return d

def mean(v): return statistics.mean(v) if v else float("nan")
def median(v): return statistics.median(v) if v else float("nan")
def hypot2(r,a,b): return math.hypot(num(r,a,0.0),num(r,b,0.0))

def interp(rows,t,keys):
    if not rows: return None
    if t <= float(rows[0]["TimeUS"]): return {k:num(rows[0],k) for k in keys}
    if t >= float(rows[-1]["TimeUS"]): return {k:num(rows[-1],k) for k in keys}
    lo,hi=0,len(rows)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if float(rows[m]["TimeUS"]) < t: lo=m
        else: hi=m
    a,b=rows[lo],rows[hi]
    ta,tb=float(a["TimeUS"]),float(b["TimeUS"])
    u=0 if tb==ta else (t-ta)/(tb-ta)
    return {k:num(a,k)+(num(b,k)-num(a,k))*u for k in keys}

def dxy_mm(a,b):
    return 1000.0*math.hypot(b["PN"]-a["PN"],b["PE"]-a["PE"])

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("dataflash_bin",type=Path)
    args=ap.parse_args()

    with args.csv.open(newline="") as fh:
        crows=list(csv.DictReader(fh))
    log=mod.parse_dataflash(args.dataflash_bin)
    a,b,matches,p95=mod.fit_clock(crows,log["OF"])

    move=[r for r in crows if int(mod.f(r,"guide_stage"))==1]
    post=[r for r in crows if int(mod.f(r,"guide_stage"))==2]
    pre=[r for r in crows if int(mod.f(r,"guide_stage"))==0]
    if not move or not post:
        raise SystemExit("Не найдены guide_stage=1/2 в CSV")

    def rpi_us(r): return mod.f(r,"mono_ns")/1000.0
    tm0,tm1=a*rpi_us(move[0])+b,a*rpi_us(move[-1])+b
    tp1=a*rpi_us(post[-1])+b
    tp0=a*rpi_us(post[0])+b
    tpre0=a*rpi_us(pre[0])+b if pre else tm0-8e6

    x1_all=sorted(log.get("XKF1",[]),key=lambda r:float(r["TimeUS"]))
    x5_all=sorted(log.get("XKF5",[]),key=lambda r:float(r["TimeUS"]))
    rf=sorted(log.get("RFND",[]),key=lambda r:float(r["TimeUS"]))

    # LOCAL_POSITION_NED displacement from CSV, used only to identify matching core.
    local=[r for r in crows if int(mod.f(r,"ekf_local_valid"))==1 and tm0 <= a*rpi_us(r)+b <= tp1]
    target_mm=float("nan")
    if len(local)>=2:
        target_mm=1000.0*math.hypot(mod.f(local[-1],"ekf_x_ned")-mod.f(local[0],"ekf_x_ned"),
                                   mod.f(local[-1],"ekf_y_ned")-mod.f(local[0],"ekf_y_ned"))

    keys=["PN","PE","VN","VE","PD","VD"]
    candidates={}
    for core in sorted({int(round(num(r,"C",0))) for r in x1_all}):
        rr=[r for r in x1_all if int(round(num(r,"C",0)))==core]
        s0,s1=interp(rr,tm0,keys),interp(rr,tp1,keys)
        if s0 and s1: candidates[core]=dxy_mm(s0,s1)
    core=min(candidates,key=lambda k:abs(candidates[k]-target_mm)) if candidates else 0
    x1=[r for r in x1_all if int(round(num(r,"C",0)))==core]
    x5=[r for r in x5_all if int(round(num(r,"C",0)))==core]

    s_move0=interp(x1,tm0,keys)
    s_move1=interp(x1,tm1,keys)
    s_post0=interp(x1,tp0,keys)
    s_post1=interp(x1,tp1,keys)

    pre_x1=[r for r in x1 if tpre0<=float(r["TimeUS"])<tm0]
    mov_x1=[r for r in x1 if tm0<=float(r["TimeUS"])<=tm1]
    post_x1=[r for r in x1 if tp0<=float(r["TimeUS"])<=tp1]
    post_x5=[r for r in x5 if tp0<=float(r["TimeUS"])<=tp1]
    post_rf=[r for r in rf if tp0<=float(r["TimeUS"])<=tp1]

    def speeds(rows): return [hypot2(r,"VN","VE") for r in rows]

    # residual raw flow during post-static from CSV
    post_csv=[r for r in crows if int(mod.f(r,"guide_stage"))==2]
    flow_post=[math.hypot(mod.f(r,"flow_body_x"),mod.f(r,"flow_body_y"))
               for r in post_csv if int(mod.f(r,"valid"))==1]

    print("===== PRE-HOVER BENCH FORENSIC =====")
    print(f"clock matches={matches} p95={p95/1000:.3f} ms")
    print(f"selected EKF3 core={core}")
    print("per-core XKF1 total move: " + ", ".join(f"C{k}={v:.1f}mm" for k,v in sorted(candidates.items())))
    print()
    print(f"move duration     = {(tm1-tm0)/1e6:.3f} s")
    print(f"post duration     = {(tp1-tp0)/1e6:.3f} s")
    print(f"XKF1 move-window  = {dxy_mm(s_move0,s_move1):.1f} mm")
    print(f"XKF1 post drift   = {dxy_mm(s_post0,s_post1):.1f} mm")
    print(f"XKF1 total        = {dxy_mm(s_move0,s_post1):.1f} mm")
    print()
    print("horizontal speed:")
    print(f"  pre  mean/max   = {mean(speeds(pre_x1)):.4f} / {max(speeds(pre_x1),default=float('nan')):.4f} m/s")
    print(f"  move mean/max   = {mean(speeds(mov_x1)):.4f} / {max(speeds(mov_x1),default=float('nan')):.4f} m/s")
    print(f"  post mean/max   = {mean(speeds(post_x1)):.4f} / {max(speeds(post_x1),default=float('nan')):.4f} m/s")
    if s_post1:
        print(f"  final speed     = {math.hypot(s_post1['VN'],s_post1['VE']):.4f} m/s")
    print()
    print(f"post RAW flow |rate| median/mean/max = {median(flow_post):.4f} / {mean(flow_post):.4f} / {max(flow_post,default=float('nan')):.4f} rad/s")
    if post_x5:
        print(f"post XKF5 HAGL median/mean = {median([num(r,'HAGL') for r in post_x5]):.3f} / {mean([num(r,'HAGL') for r in post_x5]):.3f} m")
        print(f"post XKF5 NI mean/max = {mean([num(r,'NI') for r in post_x5]):.3f} / {max([num(r,'NI') for r in post_x5],default=float('nan')):.1f}")
    if post_rf:
        print(f"post RFND median = {median([num(r,'Dist') for r in post_rf]):.3f} m")

    # Verdict thresholds are engineering gates for this pre-hover bench, not calibration targets.
    final_v=math.hypot(s_post1["VN"],s_post1["VE"]) if s_post1 else float("nan")
    post_drift=dxy_mm(s_post0,s_post1)
    ok_v=math.isfinite(final_v) and final_v<=0.03
    ok_d=math.isfinite(post_drift) and post_drift<=30.0
    print()
    print("===== PRE-HOVER GATE =====")
    print(f"final speed <= 0.03 m/s : {'PASS' if ok_v else 'FAIL'}")
    print(f"post drift <= 30 mm     : {'PASS' if ok_d else 'FAIL'}")
    print(f"RESULT                   : {'PASS' if ok_v and ok_d else 'FAIL'}")

if __name__=="__main__":
    main()
