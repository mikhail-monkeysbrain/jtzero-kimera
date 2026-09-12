#!/usr/bin/env python3
# JT-Zero — EKF internal forensic around the main horizontal-motion burst.
# Uses only existing remote_ekf.bin files from a GUI series. Does not write FC.

from __future__ import annotations
import argparse, json, math, statistics, sys
from pathlib import Path

from analyze_remote_xkv_gate import read_formats, parse_records, key_time, pct

def f(r,k,d=0.0):
    try: return float(r.get(k,d) or d)
    except Exception: return d

def i(r,k,d=0):
    try: return int(r.get(k,d) or d)
    except Exception: return d

def nearest_idx(times, t):
    if not times: return 0
    lo,hi=0,len(times)
    while lo<hi:
        m=(lo+hi)//2
        if times[m] < t: lo=m+1
        else: hi=m
    if lo<=0: return 0
    if lo>=len(times): return len(times)-1
    return lo if abs(times[lo]-t)<abs(times[lo-1]-t) else lo-1

def contiguous_burst(xkf1):
    # Primary core only when possible.
    cores=sorted({i(r,"C",0) for r in xkf1})
    core=0 if 0 in cores else (cores[0] if cores else 0)
    rr=[r for r in xkf1 if i(r,"C",0)==core]
    if len(rr)<10:
        return rr,0,max(0,len(rr)-1),core

    sp=[math.hypot(f(r,"VN"),f(r,"VE")) for r in rr]
    peak=max(sp)
    # Adaptive threshold: enough to isolate physical burst but robust to low residual drift.
    th=max(0.02, peak*0.20)
    active=[k for k,v in enumerate(sp) if v>=th]
    if not active:
        return rr,0,len(rr)-1,core

    # Split active indices by gaps >0.35s and choose segment with largest integrated speed.
    segs=[]; cur=[active[0]]
    for k in active[1:]:
        dt=(key_time(rr[k])-key_time(rr[cur[-1]]))*1e-6
        if dt>0.35:
            segs.append(cur); cur=[k]
        else:
            cur.append(k)
    segs.append(cur)

    def score(seg):
        a=max(0,seg[0]-2); b=min(len(rr)-1,seg[-1]+2)
        s=0.0
        for k in range(a+1,b+1):
            dt=(key_time(rr[k])-key_time(rr[k-1]))*1e-6
            if 0<dt<0.2:
                s += 0.5*(sp[k]+sp[k-1])*dt
        return s
    seg=max(segs,key=score)
    a=max(0,seg[0]-2); b=min(len(rr)-1,seg[-1]+2)
    return rr,a,b,core

def window_stats(name, rr, t0, t1):
    z=[r for r in rr if t0<=key_time(r)<=t1]
    if not z:
        return f"{name}: NO_DATA"
    if name=="XKF1":
        speed=[math.hypot(f(r,"VN"),f(r,"VE")) for r in z]
        return (f"XKF1 n={len(z)} speed mean={statistics.mean(speed):.4f} "
                f"p95={pct(speed,.95):.4f} max={max(speed):.4f} m/s "
                f"PN/PE start=({f(z[0],'PN'):.3f},{f(z[0],'PE'):.3f}) "
                f"end=({f(z[-1],'PN'):.3f},{f(z[-1],'PE'):.3f})")
    if name=="XKF5":
        hagl=[f(r,"HAGL") for r in z]
        fix=[f(r,"FIX") for r in z]; fiy=[f(r,"FIY") for r in z]
        evel=[f(r,"eVel") for r in z]; epos=[f(r,"ePos") for r in z]
        ni=[f(r,"NI")/100.0 for r in z]
        return (f"XKF5 n={len(z)} HAGL p50={pct(hagl,.5):.3f} "
                f"FIX p95={pct([abs(v) for v in fix],.95):.1f} "
                f"FIY p95={pct([abs(v) for v in fiy],.95):.1f} "
                f"NI max={max(ni):.3f} eVel p95={pct(evel,.95):.3f} ePos p95={pct(epos,.95):.3f}")
    if name=="XKF4":
        fs=[i(r,"FS") for r in z]; ts=[i(r,"TS") for r in z]; ss=[i(r,"SS") for r in z]
        return (f"XKF4 n={len(z)} FS_nonzero={sum(v!=0 for v in fs)} "
                f"TS_nonzero={sum(v!=0 for v in ts)} SS_unique={sorted(set(ss))[:8]}")
    return f"{name}: n={len(z)}"

def analyze_bin(path:Path):
    data=path.read_bytes()
    fmts=read_formats(data)
    wanted={"XKF1","XKF4","XKF5","OF","IMU"}
    rec=parse_records(data,fmts,wanted)
    for k in rec: rec[k].sort(key=key_time)

    rr,a,b,core=contiguous_burst(rec["XKF1"])
    if not rr:
        raise RuntimeError("XKF1 отсутствует")
    t0=key_time(rr[a]); t1=key_time(rr[b])
    # fixed windows around burst
    pre0=max(key_time(rr[0]),t0-2_000_000); pre1=t0
    post0=t1; post1=min(key_time(rr[-1]),t1+2_000_000)

    # Position and velocity integral over burst, from XKF1 itself.
    pn0,pe0=f(rr[a],"PN"),f(rr[a],"PE")
    pn1,pe1=f(rr[b],"PN"),f(rr[b],"PE")
    pos_mm=1000*math.hypot(pn1-pn0,pe1-pe0)
    vint_n=vint_e=0.0
    for k in range(a+1,b+1):
        dt=(key_time(rr[k])-key_time(rr[k-1]))*1e-6
        if 0<dt<0.2:
            vint_n += 0.5*(f(rr[k],"VN")+f(rr[k-1],"VN"))*dt
            vint_e += 0.5*(f(rr[k],"VE")+f(rr[k-1],"VE"))*dt
    vint_mm=1000*math.hypot(vint_n,vint_e)

    # OF format visibility; field names differ by branch/version.
    of_labels=[]
    for mf in fmts.values():
        if mf.name=="OF":
            of_labels=mf.labels
            break

    return {
        "core":core,"t0":t0,"t1":t1,"duration_s":(t1-t0)*1e-6,
        "pos_mm":pos_mm,"vint_mm":vint_mm,
        "counts":{k:len(v) for k,v in rec.items()},
        "of_labels":of_labels,
        "pre":{
            "XKF1":window_stats("XKF1",rec["XKF1"],pre0,pre1),
            "XKF5":window_stats("XKF5",rec["XKF5"],pre0,pre1),
            "XKF4":window_stats("XKF4",rec["XKF4"],pre0,pre1),
        },
        "move":{
            "XKF1":window_stats("XKF1",rec["XKF1"],t0,t1),
            "XKF5":window_stats("XKF5",rec["XKF5"],t0,t1),
            "XKF4":window_stats("XKF4",rec["XKF4"],t0,t1),
        },
        "post":{
            "XKF1":window_stats("XKF1",rec["XKF1"],post0,post1),
            "XKF5":window_stats("XKF5",rec["XKF5"],post0,post1),
            "XKF4":window_stats("XKF4",rec["XKF4"],post0,post1),
        }
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
    print("="*100)
    print("JT-ZERO — EKF INTERNAL MOTION FORENSIC")
    print("="*100)
    print(f"SESSION: {sp}")

    for r in runs:
        bp=Path(r["csv"]).parent/"remote_ekf.bin"
        d=analyze_bin(bp)
        phys=float(r["physical_measured_mm"])
        print()
        print(f"RUN {r['run']} {r.get('direction')} physical={phys:.1f}mm  BIN={bp}")
        print(f"XKF1 core={d['core']} burst={d['duration_s']:.3f}s pos={d['pos_mm']:.1f}mm "
              f"vint={d['vint_mm']:.1f}mm")
        print(f"records={d['counts']}")
        print(f"OF labels={d['of_labels']}")
        print("  PRE : "+d["pre"]["XKF1"])
        print("        "+d["pre"]["XKF5"])
        print("        "+d["pre"]["XKF4"])
        print("  MOVE: "+d["move"]["XKF1"])
        print("        "+d["move"]["XKF5"])
        print("        "+d["move"]["XKF4"])
        print("  POST: "+d["post"]["XKF1"])
        print("        "+d["post"]["XKF5"])
        print("        "+d["post"]["XKF4"])

    print("\nINTERPRETATION:")
    print("- Если плохой проход уже в MOVE имеет увеличенные XKF1 speed/vint при NI<1 и без TS/FS — EKF принимает flow,")
    print("  но prediction/fusion state расходится с метрическим RAW.")
    print("- Если POST сохраняет заметную XKF1 speed при неподвижном стенде — лишняя позиция создаётся остаточной velocity.")
    print("- Если TS/FS ненулевые только в плохом проходе — это отдельный внутренний EKF event, а не scale камеры.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
