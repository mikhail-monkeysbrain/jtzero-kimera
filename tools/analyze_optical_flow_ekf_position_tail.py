#!/usr/bin/env python3
# JT-Zero — разбивка EKF position на движение и статические хвосты.
# Ничего не пишет в FC.

from __future__ import annotations
import argparse, csv, json, math, statistics, sys
from pathlib import Path

FLOW_THRESHOLD=0.03

def fv(r,k,d=0.0):
    try: return float(r.get(k,d) or d)
    except Exception: return d

def dist(a,b):
    return 1000.0*math.hypot(fv(b,"ekf_x_ned")-fv(a,"ekf_x_ned"),
                             fv(b,"ekf_y_ned")-fv(a,"ekf_y_ned"))

def nearest_valid(rows, target_ns, lo, hi):
    best=None; besterr=None
    for i in range(max(0,lo),min(len(rows),hi+1)):
        if int(fv(rows[i],"ekf_local_valid"))!=1: continue
        t=fv(rows[i],"mono_ns")
        e=abs(t-target_ns)
        if best is None or e<besterr:
            best=rows[i]; besterr=e
    return best

def analyze(csv_path:Path):
    rows=list(csv.DictReader(csv_path.open(newline="")))
    mags=[math.hypot(fv(r,"flow_body_x"),fv(r,"flow_body_y")) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=FLOW_THRESHOLD and int(fv(rows[i],"valid"))==1]
    if not idx: raise RuntimeError("movement не найден")
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)
    t0=fv(rows[i0],"mono_ns"); t1=fv(rows[i1],"mono_ns")

    a=nearest_valid(rows,t0,i0-20,i0+20)
    b=nearest_valid(rows,t1,i1-20,i1+20)
    if a is None or b is None: raise RuntimeError("нет EKF local на границах")

    vint_n=vint_e=0.0
    for i in range(i0,i1+1):
        r=rows[i]; dt=fv(r,"dt_s")
        if 0<dt<0.2 and int(fv(r,"ekf_local_valid"))==1:
            vint_n += fv(r,"ekf_vx_ned")*dt
            vint_e += fv(r,"ekf_vy_ned")*dt
    vint=1000.0*math.hypot(vint_n,vint_e)

    out={
        "move_pos_mm":dist(a,b),
        "move_vint_mm":vint,
        "residual_pos_minus_vint_mm":dist(a,b)-vint,
    }

    for sec in (1,2,5):
        pre=nearest_valid(rows,t0-sec*1e9,i0-int(sec*60)-80,i0)
        post=nearest_valid(rows,t1+sec*1e9,i1,min(len(rows)-1,i1+int(sec*60)+80))
        out[f"pre_{sec}s_drift_mm"]=dist(pre,a) if pre is not None else float("nan")
        out[f"post_{sec}s_drift_mm"]=dist(b,post) if post is not None else float("nan")

    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("session_json",nargs="?")
    args=ap.parse_args()
    if args.session_json:
        sp=Path(args.session_json)
    else:
        cand=sorted(Path("/home/vio/jtzero_runs").glob("*_OPTICAL_FLOW_GUI_SERIES.json"),key=lambda p:p.stat().st_mtime,reverse=True)
        if not cand:
            print("ОШИБКА: session JSON не найден",file=sys.stderr); return 2
        sp=cand[0]

    runs=json.loads(sp.read_text(encoding="utf-8"))
    print("="*106)
    print("JT-ZERO — EKF POSITION TAIL / DRIFT ANALYSIS")
    print("="*106)
    print(f"SESSION: {sp}")
    print()
    print(" # DIR PHYS  MOVE_POS  VEL_INT  POS-VINT  PRE1  POST1  POST2  POST5")
    print("-"*106)

    vals=[]
    for r in runs:
        d=analyze(Path(r["csv"]))
        vals.append((r,d))
        print(f"{int(r['run']):2d} {(r.get('direction') or '?'):4s} {float(r['physical_measured_mm']):4.0f} "
              f"{d['move_pos_mm']:9.1f} {d['move_vint_mm']:8.1f} {d['residual_pos_minus_vint_mm']:9.1f} "
              f"{d['pre_1s_drift_mm']:5.1f} {d['post_1s_drift_mm']:6.1f} "
              f"{d['post_2s_drift_mm']:6.1f} {d['post_5s_drift_mm']:6.1f}")

    print("\n===== SUMMARY =====")
    res=[d["residual_pos_minus_vint_mm"] for _,d in vals]
    print(f"POS-VINT residual: mean={statistics.mean(res):.1f} mm sd={statistics.pstdev(res):.1f} mm "
          f"min={min(res):.1f} max={max(res):.1f}")
    for sec in (1,2,5):
        arr=[d[f"post_{sec}s_drift_mm"] for _,d in vals if math.isfinite(d[f"post_{sec}s_drift_mm"])]
        if arr:
            print(f"post-stop drift {sec}s: mean={statistics.mean(arr):.1f} mm "
                  f"sd={statistics.pstdev(arr):.1f} max={max(arr):.1f}")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
