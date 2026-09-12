#!/usr/bin/env python3
# JT-Zero — совместный анализ reciprocal GUI series:
# physical -> RAW -> FC-presented -> EKF velocity integral -> EKF position,
# плюс XKF5 HAGL/innovation gate из remote_ekf.bin.
# Ничего не пишет в FC.

from __future__ import annotations
import argparse, csv, json, math, os, statistics, sys
from pathlib import Path

from analyze_remote_xkv_gate import read_formats, parse_records, key_time, pct

FLOW_THRESHOLD=0.03

def fv(r,k,d=0.0):
    try: return float(r.get(k,d) or d)
    except Exception: return d

def csv_diag(path: Path):
    rows=list(csv.DictReader(path.open(newline="")))
    if not rows: raise RuntimeError("CSV пуст")
    mags=[math.hypot(fv(r,"flow_body_x"),fv(r,"flow_body_y")) for r in rows]
    idx=[i for i,m in enumerate(mags) if m>=FLOW_THRESHOLD and int(fv(rows[i],"valid"))==1]
    if not idx: raise RuntimeError("movement не найден")
    i0=max(1,min(idx)-5); i1=min(len(rows)-1,max(idx)+5)

    dx=dy=fdx=fdy=0.0
    evn=eve=0.0
    max_speed=0.0
    dtv=[]; mono_gaps=[]; invalid=0
    for i in range(i0,i1+1):
        r=rows[i]; dt=fv(r,"dt_s"); dtv.append(dt)
        if i>i0:
            a=fv(rows[i-1],"mono_ns"); b=fv(r,"mono_ns")
            if b>a: mono_gaps.append((b-a)*1e-6)
        if int(fv(r,"valid"))==0: invalid+=1
        if 0<dt<0.2 and int(fv(r,"valid"))==1:
            h=fv(r,"luna_m")
            if 0.05<h<20:
                dx += h*fv(r,"flow_body_x")*dt
                dy += h*fv(r,"flow_body_y")*dt
            hf=fv(r,"range_to_fc_m")
            if 0.05<hf<20:
                fdx += hf*fv(r,"flow_send_x")*dt
                fdy += hf*fv(r,"flow_send_y")*dt
        if 0<dt<0.2 and int(fv(r,"ekf_local_valid"))==1:
            vn=fv(r,"ekf_vx_ned"); ve=fv(r,"ekf_vy_ned")
            evn += vn*dt; eve += ve*dt
            max_speed=max(max_speed,math.hypot(vn,ve))

    fresh=[i for i in range(i0,i1+1) if int(fv(rows[i],"ekf_local_valid"))==1]
    ekf_pos=float("nan")
    if fresh:
        a=rows[fresh[0]]; b=rows[fresh[-1]]
        ekf_pos=1000*math.hypot(fv(b,"ekf_x_ned")-fv(a,"ekf_x_ned"),fv(b,"ekf_y_ned")-fv(a,"ekf_y_ned"))

    # duration by monotonic rows in movement envelope
    duration=float("nan")
    if i1>i0:
        a=fv(rows[i0],"mono_ns"); b=fv(rows[i1],"mono_ns")
        if b>a: duration=(b-a)*1e-9

    return {
        "raw_mm":1000*math.hypot(dx,dy),
        "fc_mm":1000*math.hypot(fdx,fdy),
        "ekf_pos_mm":ekf_pos,
        "ekf_vel_int_mm":1000*math.hypot(evn,eve),
        "max_speed_mps":max_speed,
        "duration_s":duration,
        "max_dt_ms":1000*max(dtv) if dtv else float("nan"),
        "max_wall_gap_ms":max(mono_gaps) if mono_gaps else float("nan"),
        "invalid_rows":invalid,
        "i0":i0,"i1":i1,
    }

def xkf5_diag(path: Path):
    if not path.is_file():
        return None
    data=path.read_bytes()
    fmts=read_formats(data)
    rec=parse_records(data,fmts,{"XKF5"})
    x=sorted(rec.get("XKF5",[]),key=key_time)
    if not x: return None
    ratios=[float(r.get("normInnov",0.0))/100.0 for r in x]
    hagl=[float(r.get("HAGL",0.0)) for r in x]
    return {
        "xkf5_n":len(x),
        "flow_ratio_p50":pct(ratios,0.5),
        "flow_ratio_p95":pct(ratios,0.95),
        "flow_reject_pct":100.0*sum(v>=1.0 for v in ratios)/len(ratios),
        "hagl_p50":pct(hagl,0.5),
        "hagl_p95":pct(hagl,0.95),
        "hagl_min":min(hagl),
        "hagl_max":max(hagl),
    }

def fmt(v,n=3):
    return "NA" if not math.isfinite(v) else f"{v:.{n}f}"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("session_json")
    args=ap.parse_args()
    sp=Path(args.session_json)
    if not sp.is_file():
        print(f"ОШИБКА: session JSON не найден: {sp}",file=sys.stderr); return 2
    runs=json.loads(sp.read_text(encoding="utf-8"))

    print("="*110)
    print("JT-ZERO — RECIPROCAL SERIES FORENSIC")
    print("="*110)
    print(f"SESSION: {sp}")
    print()
    print(" # DIR  PHYS   RAW  FC-PRES  EKF-POS  EKF-VINT  RAW/P  EKF/P  EKF/RAW  HAGL50  REJ%  maxdt")
    print("-"*110)

    out=[]
    for r in runs:
        csvp=Path(r["csv"]); d=csv_diag(csvp)
        binp=csvp.parent/"remote_ekf.bin"; x=xkf5_diag(binp)
        phys=float(r["physical_measured_mm"])
        direction=r.get("direction") or "?"
        row={
            **r,**d,
            "bin":str(binp),
            "raw_ratio":d["raw_mm"]/phys,
            "fc_ratio":d["fc_mm"]/phys,
            "ekf_ratio":d["ekf_pos_mm"]/phys,
            "ekf_raw_ratio":d["ekf_pos_mm"]/d["raw_mm"] if d["raw_mm"] else float("nan"),
            "ekf_vint_ratio":d["ekf_vel_int_mm"]/phys,
            "xkf5":x,
        }
        out.append(row)
        hagl=x["hagl_p50"] if x else float("nan")
        rej=x["flow_reject_pct"] if x else float("nan")
        print(f"{int(r['run']):2d} {direction:4s} {phys:5.0f} {d['raw_mm']:6.1f} {d['fc_mm']:8.1f} "
              f"{d['ekf_pos_mm']:8.1f} {d['ekf_vel_int_mm']:9.1f} {row['raw_ratio']:6.3f} "
              f"{row['ekf_ratio']:6.3f} {row['ekf_raw_ratio']:8.3f} {fmt(hagl,3):>7s} {fmt(rej,1):>5s} {d['max_dt_ms']:6.1f}")

    print("\n===== AGGREGATES =====")
    def summary(name,key):
        v=[z[key] for z in out if math.isfinite(z[key])]
        print(f"{name:18s}: mean={statistics.mean(v):.4f} sd={statistics.pstdev(v):.4f} min={min(v):.4f} max={max(v):.4f}")
    summary("RAW/physical","raw_ratio")
    summary("FC/physical","fc_ratio")
    summary("EKFpos/physical","ekf_ratio")
    summary("EKFvel/physical","ekf_vint_ratio")
    summary("EKFpos/RAW","ekf_raw_ratio")

    for direction in ("A->B","B->A"):
        g=[z for z in out if z.get("direction")==direction]
        if g:
            print(f"\n{direction}:")
            print(f"  RAW/physical mean={statistics.mean(z['raw_ratio'] for z in g):.4f}")
            print(f"  EKF/physical mean={statistics.mean(z['ekf_ratio'] for z in g):.4f}")
            print(f"  EKF/RAW mean={statistics.mean(z['ekf_raw_ratio'] for z in g):.4f}")

    print("\n===== XKF5 =====")
    xs=[z["xkf5"] for z in out if z["xkf5"]]
    if xs:
        print(f"HAGL p50 across runs: mean={statistics.mean(x['hagl_p50'] for x in xs):.3f} m "
              f"min={min(x['hagl_p50'] for x in xs):.3f} max={max(x['hagl_p50'] for x in xs):.3f}")
        print(f"flow rejection: mean={statistics.mean(x['flow_reject_pct'] for x in xs):.2f}% "
              f"max={max(x['flow_reject_pct'] for x in xs):.2f}%")
    else:
        print("XKF5: NO_DATA")

    print("\n===== HEALTH =====")
    bad=[z for z in out if z["max_dt_ms"]>200 or z["max_wall_gap_ms"]>200]
    print(f"camera/pipeline gap >200ms: {len(bad)}/{len(out)}")
    for z in bad:
        print(f"  run {z['run']} {z.get('direction')} max_dt={z['max_dt_ms']:.1f}ms wall_gap={z['max_wall_gap_ms']:.1f}ms")

    outp=sp.with_name(sp.stem+"_forensic.json")
    outp.write_text(json.dumps(out,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(f"\nFORENSIC_JSON={outp}")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
