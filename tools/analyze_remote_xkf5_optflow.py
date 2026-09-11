#!/usr/bin/env python3
# Анализ XKF5 OpticalFlow fusion из remote DataFlash BIN.
# Ничего не пишет в FC.

from __future__ import annotations
import argparse, math, os, sys
from collections import Counter
from analyze_remote_xkv_gate import read_formats, parse_records, key_time, pct

def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument("bin")
    args=ap.parse_args()
    path=args.bin
    if not os.path.isfile(path):
        print(f"ОШИБКА: файл не найден: {path}", file=sys.stderr)
        return 2

    data=open(path,"rb").read()
    fmts=read_formats(data)
    wanted={"XKF5","OF","XKF4"}
    rec=parse_records(data,fmts,wanted)
    for n in wanted:
        rec[n].sort(key=key_time)

    print("="*70)
    print("JT-ZERO — XKF5 OPTICAL FLOW FUSION ANALYSIS")
    print("="*70)
    print(f"BIN: {path}")
    print(f"bytes={len(data)} formats={len(fmts)}")
    for n in sorted(wanted):
        print(f"{n}: {len(rec[n])} records")

    if not rec["XKF5"]:
        print("\nОШИБКА: XKF5 не найден. Нужен DataFlash с EK3_LOG_LEVEL=0 и работающим remote logger.")
        return 3

    x=rec["XKF5"]
    norm=[float(r.get("normInnov",0.0)) for r in x]
    fix=[float(r.get("FIX",0.0)) for r in x]
    fiy=[float(r.get("FIY",0.0)) for r in x]
    hagl=[float(r.get("HAGL",0.0)) for r in x]
    rng=[float(r.get("meaRng",0.0)) for r in x]
    ri=[float(r.get("RI",0.0)) for r in x]

    # normInnov логируется как 100*max(flowTestRatio), saturation 255.
    ratios=[v/100.0 for v in norm]
    accepted=[v for v in ratios if v < 1.0]
    rejected=[v for v in ratios if v >= 1.0]

    def line(name,vals,unit=""):
        if not vals:
            print(f"{name}: NO_DATA")
            return
        print(f"{name}: n={len(vals)} min={min(vals):.4f}{unit} p50={pct(vals,0.5):.4f}{unit} p95={pct(vals,0.95):.4f}{unit} max={max(vals):.4f}{unit}")

    print("\n===== OPTFLOW INNOVATION GATE =====")
    line("flowTestRatio(max axis)",ratios)
    print(f"ratio<1 accepted_samples={len(accepted)}/{len(ratios)} ({100.0*len(accepted)/len(ratios):.1f}%)")
    print(f"ratio>=1 rejected_samples={len(rejected)}/{len(ratios)} ({100.0*len(rejected)/len(ratios):.1f}%)")
    line("FIX",fix)
    line("FIY",fiy)

    print("\n===== HEIGHT USED BY EKF =====")
    line("HAGL",hagl," m")
    line("meaRng",rng," m")
    line("range innovation RI",ri," m")
    under05=sum(1 for v in hagl if v<0.5)
    print(f"HAGL<0.5m samples={under05}/{len(hagl)} ({100.0*under05/len(hagl):.1f}%)")

    print("\n===== VERDICT =====")
    rej_frac=len(rejected)/len(ratios) if ratios else 1.0
    if rej_frac>0.25:
        print("FLOW_INNOVATION_REJECTION=SIGNIFICANT")
        print("Существенная доля XKF5 имеет flowTestRatio>=1: часть OpticalFlow EKF отвергает innovation gate.")
    else:
        print("FLOW_INNOVATION_REJECTION=LOW")
        print("Innovation gate не объясняет большую потерю displacement сам по себе.")
    if hagl:
        print(f"HAGL_P50={pct(hagl,0.5):.3f}m")
        if pct(hagl,0.5)<0.5:
            print("EKF_HAGL_STILL_BELOW_0P5=YES")
            print("Даже при bench range override внутренний HAGL EKF остаётся ниже 0.5 м.")
        else:
            print("EKF_HAGL_STILL_BELOW_0P5=NO")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
