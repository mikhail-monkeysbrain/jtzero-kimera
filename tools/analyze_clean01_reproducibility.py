#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

def load_csv(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def f(x): return float(x)
def i(x): return int(x)

def mean_state(rows):
    keys=("px","py","pz")
    return {k:statistics.mean(f(r[k]) for r in rows) for k in keys}

def disp(a,b):
    dx=b["px"]-a["px"]; dy=b["py"]-a["py"]; dz=b["pz"]-a["pz"]
    h=math.hypot(dx,dy)
    return dx,dy,dz,h

def analyze(root,window_ms):
    ev=load_csv(root/"events.csv")
    be=load_csv(root/"backend.csv")
    starts=[r for r in ev if r["event"]=="MOVE_START"]
    ends=[r for r in ev if r["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1:
        raise RuntimeError(f"{root}: expected one MOVE_START/MOVE_END")
    a,b=starts[0],ends[0]
    t0,t1=i(a["wall_ns"]),i(b["wall_ns"])
    A={k:f(a[k]) for k in ("px","py","pz")}
    B={k:f(b[k]) for k in ("px","py","pz")}
    _,_,dz_e,h_e=disp(A,B)
    w=int(window_ms*1e6)
    pre=[r for r in be if t0-w <= i(r["callback_wall_ns"]) <= t0]
    post=[r for r in be if t1 <= i(r["callback_wall_ns"]) <= t1+w]
    if not pre or not post:
        raise RuntimeError(f"{root}: insufficient fixed-window backend states")
    P=mean_state(pre); Q=mean_state(post)
    _,_,dz_w,h_w=disp(P,Q)
    return {
        "run":root.name,
        "duration":(t1-t0)/1e9,
        "event_mm":h_e*1000,
        "window_mm":h_w*1000,
        "event_dz_mm":dz_e*1000,
        "window_dz_mm":dz_w*1000,
        "pre_n":len(pre),"post_n":len(post),
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",nargs="+")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--window-ms",type=float,default=1000.0)
    a=ap.parse_args()

    rows=[analyze(Path(r),a.window_ms) for r in a.runs]
    print("="*122)
    print("CLEAN-01 — REPRODUCIBILITY, SAME PROTOCOL / SAME CONFIGURATION")
    print("="*122)
    print(f"truth={a.truth_mm:.1f} mm   fixed-window={a.window_ms:.0f} ms")
    print()
    print(f"{'RUN':42s} {'MOVE s':>7s} {'EVENT mm':>10s} {'WINDOW mm':>11s} {'SCALE':>8s} {'ERR mm':>9s} {'dZ mm':>8s}")
    print("-"*122)
    for r in rows:
        scale=r["window_mm"]/a.truth_mm
        err=r["window_mm"]-a.truth_mm
        print(f"{r['run']:42s} {r['duration']:7.3f} {r['event_mm']:10.2f} {r['window_mm']:11.2f} {scale:8.4f} {err:+9.2f} {r['window_dz_mm']:+8.2f}")

    vals=[r["window_mm"] for r in rows]
    print()
    print("REPRODUCIBILITY SUMMARY")
    print("-"*122)
    print(f"n={len(vals)}")
    print(f"median={statistics.median(vals):.2f} mm")
    print(f"min={min(vals):.2f} mm")
    print(f"max={max(vals):.2f} mm")
    print(f"range={max(vals)-min(vals):.2f} mm")
    if len(vals)>=2:
        print(f"sample_std={statistics.stdev(vals):.2f} mm")
    print(f"median_scale={statistics.median(vals)/a.truth_mm:.4f}")

    print()
    print("STRUCTURE")
    print("-"*122)
    spread=max(vals)-min(vals)
    center=statistics.median(vals)
    if spread <= 30:
        print("STABLE_BIAS: runs cluster tightly; residual scale error is reproducible.")
    elif spread >= 80:
        print("RUN_TO_RUN_INSTABILITY: spread is large; do NOT introduce a single scale correction.")
    else:
        print("MIXED: spread is material; more important to explain run-to-run state differences than fit one scale.")

    print()
    print("No historical V25/V41/V43/V44 runs are read by this analyzer.")
    print("="*122)

if __name__=="__main__":
    main()
