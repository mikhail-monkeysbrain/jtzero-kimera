#!/usr/bin/env python3
# JT-Zero — анализ задержки camera timestamp -> MAVLink OPTICAL_FLOW send.
# Ничего не пишет в FC.

from __future__ import annotations
import argparse,csv,math,statistics,sys
from pathlib import Path

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

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv")
    args=ap.parse_args()
    p=Path(args.csv)
    if not p.is_file():
        print(f"ОШИБКА: файл не найден: {p}",file=sys.stderr); return 2
    rows=list(csv.DictReader(p.open(newline="")))
    if not rows:
        print("ОШИБКА: CSV пуст",file=sys.stderr); return 3
    if "frame_pipeline_latency_ms" not in rows[0]:
        print("ОШИБКА: CSV старого формата; нет frame_pipeline_latency_ms",file=sys.stderr); return 4

    lat=[fv(r,"frame_pipeline_latency_ms") for r in rows]
    lat=[x for x in lat if math.isfinite(x) and x>=0]
    valid=[fv(r,"frame_pipeline_latency_ms") for r in rows if int(fv(r,"valid",0))==1]
    valid=[x for x in valid if math.isfinite(x) and x>=0]

    print("="*76)
    print("JT-ZERO — CAMERA -> OPTICAL_FLOW SEND LATENCY")
    print("="*76)
    print(f"CSV: {p}")
    print(f"rows={len(rows)} latency_samples={len(lat)} valid_flow_samples={len(valid)}")
    def line(name,v):
        if not v:
            print(f"{name}: NO_DATA"); return
        print(f"{name}: min={min(v):.3f} ms p50={pct(v,.5):.3f} ms p95={pct(v,.95):.3f} ms "
              f"p99={pct(v,.99):.3f} ms max={max(v):.3f} ms mean={statistics.mean(v):.3f} ms sd={statistics.pstdev(v):.3f} ms")
    line("ALL",lat)
    line("VALID",valid)
    if valid:
        print("\n===== INTERPRETATION =====")
        med=pct(valid,.5); p95=pct(valid,.95)
        print(f"CAMERA_TO_SEND_P50={med:.1f}ms")
        print(f"CAMERA_TO_SEND_P95={p95:.1f}ms")
        if p95-med>20:
            print("LATENCY_JITTER=HIGH")
        else:
            print("LATENCY_JITTER=LOW")
        print("ВАЖНО: это software-visible latency от V4L2 camera timestamp до отправки MAVLink.")
        print("Она не доказывает точный exposure midpoint и не должна автоматически копироваться в EK3_FLOW_DELAY.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
