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

def of_motion_burst(of):
    if not of:
        return 0, 0
    mag=[math.hypot(f(r,"flowX"),f(r,"flowY")) for r in of]
    peak=max(mag) if mag else 0.0
    th=max(0.03, peak*0.15)
    active=[k for k,v in enumerate(mag) if v>=th]
    if not active:
        return 0, len(of)-1

    segs=[]; cur=[active[0]]
    for k in active[1:]:
        dt=(key_time(of[k])-key_time(of[cur[-1]]))*1e-6
        if dt>0.35:
            segs.append(cur); cur=[k]
        else:
            cur.append(k)
    segs.append(cur)

    def score(seg):
        a=max(0,seg[0]-1); b=min(len(of)-1,seg[-1]+1)
        s=0.0
        for k in range(a+1,b+1):
            dt=(key_time(of[k])-key_time(of[k-1]))*1e-6
            if 0<dt<0.5:
                s += 0.5*(mag[k]+mag[k-1])*dt
        return s

    seg=max(segs,key=score)
    return max(0,seg[0]-1), min(len(of)-1,seg[-1]+1)


def filter_core(records, core):
    if not records:
        return []
    if "C" not in records[0]:
        return records
    return [r for r in records if i(r,"C",0)==core]


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
        return (f"XKF4 n={len(z)} FS_unique={sorted(set(fs))[:8]} "
                f"TS_unique={sorted(set(ts))[:8]} SS_unique={sorted(set(ss))[:8]}")
    return f"{name}: n={len(z)}"

def analyze_bin(path:Path):
    data=path.read_bytes()
    fmts=read_formats(data)
    wanted={"XKF1","XKF4","XKF5","OF","IMU"}
    rec=parse_records(data,fmts,wanted)
    for k in rec: rec[k].sort(key=key_time)

    if not rec["XKF1"] or not rec["OF"]:
        raise RuntimeError("нужны XKF1 и OF")

    cores=sorted({i(r,"C",0) for r in rec["XKF1"]})
    core=0 if 0 in cores else cores[0]
    x1=filter_core(rec["XKF1"],core)
    x4=filter_core(rec["XKF4"],core)
    x5=filter_core(rec["XKF5"],core)

    oa,ob=of_motion_burst(rec["OF"])
    t0=key_time(rec["OF"][oa]); t1=key_time(rec["OF"][ob])

    # Align XKF1 to the OF-defined physical-motion interval.
    times=[key_time(r) for r in x1]
    a=nearest_idx(times,t0); b=nearest_idx(times,t1)
    if b<a: a,b=b,a

    pre0=max(key_time(x1[0]),t0-2_000_000); pre1=t0
    post0=t1; post1=min(key_time(x1[-1]),t1+2_000_000)

    pn0,pe0=f(x1[a],"PN"),f(x1[a],"PE")
    pn1,pe1=f(x1[b],"PN"),f(x1[b],"PE")
    pos_mm=1000*math.hypot(pn1-pn0,pe1-pe0)
    vint_n=vint_e=0.0
    for k in range(a+1,b+1):
        dt=(key_time(x1[k])-key_time(x1[k-1]))*1e-6
        if 0<dt<0.2:
            vint_n += 0.5*(f(x1[k],"VN")+f(x1[k-1],"VN"))*dt
            vint_e += 0.5*(f(x1[k],"VE")+f(x1[k-1],"VE"))*dt
    vint_mm=1000*math.hypot(vint_n,vint_e)

    of_labels=[]
    for mf in fmts.values():
        if mf.name=="OF":
            of_labels=mf.labels
            break

    return {
        "core":core,"t0":t0,"t1":t1,"duration_s":(t1-t0)*1e-6,
        "pos_mm":pos_mm,"vint_mm":vint_mm,
        "counts":{k:len(v) for k,v in rec.items()},
        "core_counts":{"XKF1":len(x1),"XKF4":len(x4),"XKF5":len(x5)},
        "of_labels":of_labels,
        "pre":{
            "XKF1":window_stats("XKF1",x1,pre0,pre1),
            "XKF5":window_stats("XKF5",x5,pre0,pre1),
            "XKF4":window_stats("XKF4",x4,pre0,pre1),
        },
        "move":{
            "XKF1":window_stats("XKF1",x1,t0,t1),
            "XKF5":window_stats("XKF5",x5,t0,t1),
            "XKF4":window_stats("XKF4",x4,t0,t1),
        },
        "post":{
            "XKF1":window_stats("XKF1",x1,post0,post1),
            "XKF5":window_stats("XKF5",x5,post0,post1),
            "XKF4":window_stats("XKF4",x4,post0,post1),
        }
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
        print(f"records={d['counts']} core_records={d['core_counts']}")
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
    print("- MOVE теперь определяется по DataFlash OF, а XKF1/XKF4/XKF5 фильтруются по одному EKF core.")
    print("- TS — битовая маска timeout и может быть ненулевой штатно в GPS-denied конфигурации; сравнивать нужно её значение между проходами.")
    print("- Если плохой проход уже в OF-окне имеет увеличенные XKF1 speed/vint при NI<1 — расхождение рождается внутри EKF state.")
    print("- Если POST сохраняет заметную XKF1 speed после окончания OF burst — часть лишней позиции создаётся остаточной velocity.")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
