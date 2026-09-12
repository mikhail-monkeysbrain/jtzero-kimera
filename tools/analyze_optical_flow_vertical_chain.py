#!/usr/bin/env python3
# JT-Zero — vertical-chain forensic for optical-flow bench.
# Compares RPi-presented range with EKF XKF1 PD and XKF5 HAGL/TOfs/RI/rng.
# Uses existing logs only. Does not write FC.

from __future__ import annotations
import argparse, csv, json, math, statistics, sys
from pathlib import Path

from analyze_remote_xkv_gate import read_formats, parse_records, key_time, pct

def fv(r,k,d=float("nan")):
    try:
        v=r.get(k,"")
        return float(v) if v not in ("",None) else d
    except Exception:
        return d

def iv(r,k,d=0):
    try: return int(r.get(k,d) or d)
    except Exception: return d

def clean(v):
    return [x for x in v if math.isfinite(x)]

def stat(v):
    v=clean(v)
    if not v: return None
    return {"n":len(v),"min":min(v),"p50":pct(v,.5),"p95":pct(v,.95),"max":max(v),"mean":statistics.mean(v)}

def fmt(s):
    if not s: return "NO_DATA"
    return f"n={s['n']} min={s['min']:.3f} p50={s['p50']:.3f} p95={s['p95']:.3f} max={s['max']:.3f} mean={s['mean']:.3f}"

def csv_range(path:Path):
    rows=list(csv.DictReader(path.open(newline="")))
    if not rows: raise RuntimeError("CSV пуст")
    presented=[fv(r,"range_to_fc_m") for r in rows]
    luna=[fv(r,"luna_m") for r in rows]
    send=[fv(r,"range_sent") for r in rows]
    return {
        "presented":stat([x for x in presented if 0.01<x<20]),
        "luna":stat([x for x in luna if 0.01<x<20]),
        "range_sent_rows":sum(1 for x in send if math.isfinite(x) and x>0),
        "rows":len(rows),
    }

def bin_vertical(path:Path):
    data=path.read_bytes()
    fmts=read_formats(data)
    rec=parse_records(data,fmts,{"XKF1","XKF5","XKF4"})
    for k in rec: rec[k].sort(key=key_time)

    cores=sorted({iv(r,"C",0) for r in rec["XKF1"]})
    core=0 if 0 in cores else (cores[0] if cores else 0)
    x1=[r for r in rec["XKF1"] if iv(r,"C",0)==core]
    x5=[r for r in rec["XKF5"] if iv(r,"C",0)==core]
    x4=[r for r in rec["XKF4"] if iv(r,"C",0)==core]

    pd=stat([fv(r,"PD") for r in x1])
    hagl=stat([fv(r,"HAGL") for r in x5])
    tofs=stat([fv(r,"TOfs") for r in x5])
    ri=stat([fv(r,"RI") for r in x5])
    rng=stat([fv(r,"rng") for r in x5])
    ss=sorted(set(iv(r,"SS") for r in x4))
    ts=sorted(set(iv(r,"TS") for r in x4))
    fs=sorted(set(iv(r,"FS") for r in x4))

    # Reconstruct HAGL from TOfs - PD using nearest XKF1 by timestamp.
    recon=[]
    if x1 and x5:
        times=[key_time(r) for r in x1]
        import bisect
        for r in x5:
            t=key_time(r); k=bisect.bisect_left(times,t)
            cand=[]
            if k<len(x1): cand.append(x1[k])
            if k>0: cand.append(x1[k-1])
            if not cand: continue
            q=min(cand,key=lambda z:abs(key_time(z)-t))
            if abs(key_time(q)-t)>100000: continue
            a=fv(r,"TOfs"); b=fv(q,"PD")
            if math.isfinite(a) and math.isfinite(b):
                recon.append(a-b)

    return {
        "core":core,
        "counts":{"XKF1":len(x1),"XKF5":len(x5),"XKF4":len(x4)},
        "PD":pd,"HAGL":hagl,"TOfs":tofs,"RI":ri,"rng":rng,
        "HAGL_reconstructed":stat(recon),
        "SS":ss,"TS":ts,"FS":fs,
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("session_json",nargs="?")
    args=ap.parse_args()
    if args.session_json:
        sp=Path(args.session_json)
    else:
        cand=sorted(Path("/home/vio/jtzero_runs").glob("*_OPTICAL_FLOW_GUI_SERIES.json"),
                    key=lambda p:p.stat().st_mtime,reverse=True)
        if not cand:
            print("ОШИБКА: session JSON не найден",file=sys.stderr); return 2
        sp=cand[0]

    runs=json.loads(sp.read_text(encoding="utf-8"))
    print("="*96)
    print("JT-ZERO — RANGE -> EKF HAGL VERTICAL CHAIN FORENSIC")
    print("="*96)
    print(f"SESSION: {sp}")

    for r in runs:
        csvp=Path(r["csv"]); binp=csvp.parent/"remote_ekf.bin"
        c=csv_range(csvp); b=bin_vertical(binp)
        print()
        print(f"RUN {r['run']} {r.get('direction')} physical={float(r['physical_measured_mm']):.1f}mm")
        print(f"CSV={csvp}")
        print(f"BIN={binp}")
        print(f"  RPi real Luna      : {fmt(c['luna'])}")
        print(f"  RPi range_to_fc    : {fmt(c['presented'])}")
        print(f"  range_sent rows    : {c['range_sent_rows']}/{c['rows']}")
        print(f"  XKF1 PD            : {fmt(b['PD'])}")
        print(f"  XKF5 TOfs          : {fmt(b['TOfs'])}")
        print(f"  XKF5 HAGL          : {fmt(b['HAGL'])}")
        print(f"  reconstructed HAGL : {fmt(b['HAGL_reconstructed'])}")
        print(f"  XKF5 rng           : {fmt(b['rng'])}")
        print(f"  XKF5 RI            : {fmt(b['RI'])}")
        print(f"  core={b['core']} counts={b['counts']} FS={b['FS']} TS={b['TS']} SS={b['SS']}")

        if c["presented"] and b["HAGL"]:
            rp=c["presented"]["p50"]; hp=b["HAGL"]["p50"]
            print(f"  HAGL / presented p50 = {hp/rp:.4f}x  delta={(hp-rp)*1000:+.1f} mm")
        if b["TOfs"] and b["PD"]:
            print(f"  TOfs_p50 - PD_p50 = {b['TOfs']['p50']-b['PD']['p50']:.3f} m")

    print("\nINTERPRETATION:")
    print("- Если range_to_fc остаётся ~0.600 м, а HAGL меняется из-за PD при почти постоянном TOfs,")
    print("  источник ошибки — вертикальная позиция EKF, а не MAVLink RangeFinder ingress.")
    print("- Если TOfs меняется, проблема находится в terrain/range vertical state.")
    print("- Если XKF5 rng/RI остаются нулевыми, это не означает автоматически отсутствие range fusion;")
    print("  в этой конфигурации ключевой диагностикой остаются HAGL, TOfs и PD.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
