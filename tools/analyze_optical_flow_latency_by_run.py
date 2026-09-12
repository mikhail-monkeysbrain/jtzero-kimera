#!/usr/bin/env python3
# JT-Zero — сравнение camera->send latency по проходам последней GUI-серии.
# Разделяет всю запись и реальное окно движения. Ничего не пишет в FC.

from __future__ import annotations
import argparse,csv,json,math,statistics,sys
from pathlib import Path

FLOW_THRESHOLD=0.03

def fv(r,k,d=float("nan")):
    try:
        v=r.get(k,"")
        return float(v) if v not in ("",None) else d
    except Exception:
        return d

def pct(vals,q):
    if not vals: return float("nan")
    s=sorted(vals); x=(len(s)-1)*q; i=int(math.floor(x)); j=min(i+1,len(s)-1); a=x-i
    return s[i]*(1-a)+s[j]*a

def stats(vals):
    vals=[v for v in vals if math.isfinite(v) and v>=0]
    if not vals: return None
    return {
        "n":len(vals),"p50":pct(vals,.5),"p95":pct(vals,.95),"p99":pct(vals,.99),
        "min":min(vals),"max":max(vals),"mean":statistics.mean(vals),
        "sd":statistics.pstdev(vals),
        "over250":sum(v>250 for v in vals),
        "over300":sum(v>300 for v in vals),
    }

def analyze_csv(path:Path):
    rows=list(csv.DictReader(path.open(newline="")))
    if not rows: raise RuntimeError("CSV пуст")
    if "frame_pipeline_latency_ms" not in rows[0]:
        raise RuntimeError("CSV старого формата")

    mags=[math.hypot(fv(r,"flow_body_x",0),fv(r,"flow_body_y",0)) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=FLOW_THRESHOLD and int(fv(rows[i],"valid",0))==1]
    if not idx: raise RuntimeError("movement не найден")
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)

    all_lat=[fv(r,"frame_pipeline_latency_ms") for r in rows if int(fv(r,"valid",0))==1]
    move_lat=[fv(rows[i],"frame_pipeline_latency_ms") for i in range(i0,i1+1) if int(fv(rows[i],"valid",0))==1]

    return {
        "rows":len(rows),
        "i0":i0,"i1":i1,
        "all":stats(all_lat),
        "move":stats(move_lat),
    }

def line(tag,s):
    if not s: return f"{tag}: NO_DATA"
    return (f"{tag}: n={s['n']} p50={s['p50']:.1f} p95={s['p95']:.1f} p99={s['p99']:.1f} "
            f"max={s['max']:.1f} sd={s['sd']:.1f} >250ms={s['over250']} >300ms={s['over300']}")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("session_json",nargs="?")
    args=ap.parse_args()
    if args.session_json:
        sp=Path(args.session_json)
    else:
        cand=sorted(Path("/home/vio/jtzero_runs").glob("*_OPTICAL_FLOW_GUI_SERIES.json"),
                    key=lambda p:p.stat().st_mtime, reverse=True)
        if not cand:
            print("ОШИБКА: session JSON не найден",file=sys.stderr); return 2
        sp=cand[0]

    runs=json.loads(sp.read_text(encoding="utf-8"))
    print("="*96)
    print("JT-ZERO — LATENCY BY RUN / MOVEMENT WINDOW")
    print("="*96)
    print(f"SESSION: {sp}")

    rows_out=[]
    for r in runs:
        p=Path(r["csv"]); d=analyze_csv(p)
        physical=float(r["physical_measured_mm"])
        raw=float(r["raw_mm"]); ekf=float(r["ekf_mm"])
        direction=r.get("direction") or "?"
        eraw=raw/physical
        eekf=ekf/physical
        print()
        print(f"RUN {r['run']} {direction} physical={physical:.1f} raw={raw:.1f} ({eraw:.4f}x) "
              f"ekf={ekf:.1f} ({eekf:.4f}x) ekf/raw={ekf/raw:.4f}x")
        print(line("  ALL ",d["all"]))
        print(line("  MOVE",d["move"]))
        rows_out.append({
            "run":r["run"],"direction":direction,"physical":physical,
            "raw_ratio":eraw,"ekf_ratio":eekf,"ekf_raw":ekf/raw,
            "move_p50":d["move"]["p50"],"move_p95":d["move"]["p95"],
            "move_p99":d["move"]["p99"],"move_max":d["move"]["max"],
            "move_sd":d["move"]["sd"],"move_over250":d["move"]["over250"],
            "move_over300":d["move"]["over300"],
        })

    if len(rows_out)>=2:
        print("\n===== COMPARISON =====")
        best=min(rows_out,key=lambda z:abs(z["ekf_raw"]-1))
        worst=max(rows_out,key=lambda z:abs(z["ekf_raw"]-1))
        print(f"EKF/RAW closest to 1: run {best['run']} = {best['ekf_raw']:.4f}, "
              f"MOVE p50/p95={best['move_p50']:.1f}/{best['move_p95']:.1f} ms")
        print(f"EKF/RAW farthest from 1: run {worst['run']} = {worst['ekf_raw']:.4f}, "
              f"MOVE p50/p95={worst['move_p50']:.1f}/{worst['move_p95']:.1f} ms")
        dp50=worst["move_p50"]-best["move_p50"]
        dp95=worst["move_p95"]-best["move_p95"]
        print(f"latency delta worst-best: p50={dp50:+.1f} ms p95={dp95:+.1f} ms")
        print("\nINTERPRETATION:")
        if abs(dp50)<10 and abs(dp95)<20:
            print("RUN_LATENCY_LEVEL_SIMILAR=YES")
            print("Средний software-visible latency похож в хорошем и плохом проходах.")
            print("Простая гипотеза 'в плохом проходе просто была больше постоянная задержка' НЕ подтверждается.")
        else:
            print("RUN_LATENCY_LEVEL_SIMILAR=NO")
            print("У плохого и хорошего проходов заметно отличается latency; гипотеза timing остаётся сильной.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
