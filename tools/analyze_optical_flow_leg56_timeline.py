#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, json, math, statistics
from pathlib import Path
import importlib.util

HERE = Path(__file__).resolve().parent
base_path = HERE / "analyze_optical_flow_continuous_dataflash.py"
spec = importlib.util.spec_from_file_location("ofdf", base_path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def num(r, k, default=float("nan")):
    try: return float(r[k])
    except Exception: return default

def vecnorm(x,y): return math.hypot(x,y)

def interp_state(rows, t_us, keys):
    if not rows: return None
    lo, hi = 0, len(rows)-1
    if t_us <= float(rows[0]["TimeUS"]): return {k:num(rows[0],k) for k in keys}
    if t_us >= float(rows[-1]["TimeUS"]): return {k:num(rows[-1],k) for k in keys}
    while hi-lo>1:
        m=(lo+hi)//2
        if float(rows[m]["TimeUS"]) < t_us: lo=m
        else: hi=m
    a,b=rows[lo],rows[hi]
    ta,tb=float(a["TimeUS"]),float(b["TimeUS"])
    u=0.0 if tb==ta else (t_us-ta)/(tb-ta)
    return {k:num(a,k)+(num(b,k)-num(a,k))*u for k in keys}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    ap.add_argument("session_json",type=Path)
    ap.add_argument("dataflash_bin",type=Path)
    ap.add_argument("--legs",default="5,6")
    ap.add_argument("--post",type=float,default=1.5,help="post-stop окно, с")
    args=ap.parse_args()

    with args.csv.open(newline="") as fh:
        crows=list(csv.DictReader(fh))
    session=json.loads(args.session_json.read_text(encoding="utf-8"))
    log=mod.parse_dataflash(args.dataflash_bin)
    a,b,matches,p95=mod.fit_clock(crows,log["OF"])

    wanted={int(x) for x in args.legs.split(",")}
    x1_all=sorted(log.get("XKF1",[]),key=lambda r:float(r["TimeUS"]))
    x5_all=sorted(log.get("XKF5",[]),key=lambda r:float(r["TimeUS"]))
    of=sorted(log.get("OF",[]),key=lambda r:float(r["TimeUS"]))

    print("===== LEG 5/6 TEMPORAL FORENSIC =====")
    print(f"clock matches={matches} p95={p95/1000:.3f} ms")
    print(f"XKF1 rows={len(x1_all)} XKF5 rows={len(x5_all)} OF rows={len(of)}")
    print()

    for rec in session:
        leg=int(rec["leg"])
        if leg not in wanted: continue

        leg_rows=[r for r in crows if int(mod.f(r,"guide_leg"))==leg]
        i0,i1=int(rec["movement_row_start"]),int(rec["movement_row_end"])
        win=leg_rows[i0:i1+1]
        if not win:
            print(f"LEG {leg}: empty movement window")
            continue

        rpi0=mod.f(win[0],"mono_ns")/1000.0
        rpi1=mod.f(win[-1],"mono_ns")/1000.0
        t0,t1=a*rpi0+b,a*rpi1+b
        dur=(t1-t0)/1e6

        keys=["PN","PE","VN","VE","PD","VD"]

        # Determine which EKF3 core is actually represented by the bench
        # LOCAL_POSITION_NED result for this leg. Never mix XKF cores.
        candidates={}
        for core in sorted({int(round(num(r,"C",0))) for r in x1_all}):
            rr=[r for r in x1_all if int(round(num(r,"C",0)))==core]
            a0=interp_state(rr,t0,keys)
            a1=interp_state(rr,t1,keys)
            if a0 is None or a1 is None:
                continue
            move=1000.0*vecnorm(a1["PN"]-a0["PN"],a1["PE"]-a0["PE"])
            candidates[core]=move
        target=float(rec["ekf_mm"])
        core=min(candidates,key=lambda k:abs(candidates[k]-target))
        x1=[r for r in x1_all if int(round(num(r,"C",0)))==core]
        x5=[r for r in x5_all if int(round(num(r,"C",0)))==core]

        s0=interp_state(x1,t0,keys)
        s1=interp_state(x1,t1,keys)
        sp=interp_state(x1,t1+args.post*1e6,keys)

        def dxy(sa,sb):
            if sa is None or sb is None: return float("nan")
            return 1000.0*vecnorm(sb["PN"]-sa["PN"],sb["PE"]-sa["PE"])

        mov_x1=[r for r in x1 if t0<=float(r["TimeUS"])<=t1]
        post_x1=[r for r in x1 if t1<float(r["TimeUS"])<=t1+args.post*1e6]
        mov_x5=[r for r in x5 if t0<=float(r["TimeUS"])<=t1]
        mov_of=[r for r in of if t0<=float(r["TimeUS"])<=t1]

        vmax=max((vecnorm(num(r,"VN",0),num(r,"VE",0)) for r in mov_x1),default=float("nan"))
        vend=vecnorm(s1["VN"],s1["VE"]) if s1 else float("nan")
        vpost_mean=statistics.mean([vecnorm(num(r,"VN",0),num(r,"VE",0)) for r in post_x1]) if post_x1 else float("nan")
        post_drift=dxy(s1,sp)

        print(f"LEG {leg} {rec['direction']} phys={rec['physical_measured_mm']:.0f} mm CORE={core}")
        print("  per-core XKF1 move: " + ", ".join(f"C{k}={v:.1f}mm" for k,v in sorted(candidates.items())))
        print(f"  RAW/P={rec['raw_ratio']:.4f} EKF/P={rec['ekf_ratio']:.4f} EKF/RAW={rec['ekf_ratio']/rec['raw_ratio']:.4f}")
        print(f"  duration={dur:.3f}s XKF1_move={dxy(s0,s1):.1f}mm")
        print(f"  XKF speed: vmax={vmax:.3f}m/s vend={vend:.3f}m/s post_mean={vpost_mean:.3f}m/s post_drift_{args.post:.1f}s={post_drift:.1f}mm")

        # 10%-bins of XKF position/speed and XKF5 innovation state
        print("  frac   dpos_mm speed_mps HAGL   NI   FIX   FIY")
        for j in range(11):
            u=j/10
            t=t0+(t1-t0)*u
            sx=interp_state(x1,t,keys)
            pos=dxy(s0,sx)
            speed=vecnorm(sx["VN"],sx["VE"]) if sx else float("nan")
            near=[r for r in mov_x5 if abs(float(r["TimeUS"])-t)<=100000]
            if near:
                h=statistics.mean(num(r,"HAGL") for r in near)
                ni=statistics.mean(num(r,"NI") for r in near)
                fix=statistics.mean(num(r,"FIX") for r in near)
                fiy=statistics.mean(num(r,"FIY") for r in near)
            else:
                h=ni=fix=fiy=float("nan")
            print(f"  {u:>4.1f} {pos:>9.1f} {speed:>9.3f} {h:>5.3f} {ni:>5.2f} {fix:>6.1f} {fiy:>7.1f}")

        # Where XKF5 abnormal values peak
        if mov_x5:
            worst_ni=max(mov_x5,key=lambda r:abs(num(r,"NI",0)))
            worst_fiy=max(mov_x5,key=lambda r:abs(num(r,"FIY",0)))
            print(f"  peak NI : t={(float(worst_ni['TimeUS'])-t0)/1e6:+.3f}s NI={num(worst_ni,'NI'):.2f} FIX={num(worst_ni,'FIX'):.1f} FIY={num(worst_ni,'FIY'):.1f}")
            print(f"  peak FIY: t={(float(worst_fiy['TimeUS'])-t0)/1e6:+.3f}s NI={num(worst_fiy,'NI'):.2f} FIX={num(worst_fiy,'FIX'):.1f} FIY={num(worst_fiy,'FIY'):.1f}")
        print()

if __name__=="__main__":
    main()
