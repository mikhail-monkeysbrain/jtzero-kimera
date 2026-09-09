#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

def f(x): return float(x)
def i(x): return int(x)

def med(v):
    return statistics.median(v) if v else float("nan")

def mean(v):
    return statistics.mean(v) if v else float("nan")

def span(v):
    return (min(v),max(v),max(v)-min(v)) if v else (float("nan"),)*3

def nearest_state(states, wall_ns):
    # backend has callback_wall_ns in same monotonic clock as event wall_ns
    return min(states, key=lambda r: abs(i(r["callback_wall_ns"])-wall_ns))

def window(states,a,b):
    return [r for r in states if a <= i(r["callback_wall_ns"]) <= b]

def mean_state(ws):
    if not ws: return None
    keys=["px","py","pz","vx","vy","vz","roll_deg","pitch_deg","yaw_deg"]
    out={k:mean([f(r[k]) for r in ws]) for k in keys}
    out["n"]=len(ws)
    return out

def displacement(a,b):
    dx=b["px"]-a["px"]; dy=b["py"]-a["py"]; dz=b["pz"]-a["pz"]
    h=math.hypot(dx,dy); d=math.sqrt(dx*dx+dy*dy+dz*dz)
    return dx,dy,dz,h,d

def event_state(e):
    return {k:f(e[k]) for k in ("px","py","pz")}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--window-ms",type=float,default=1000.0)
    a=ap.parse_args()
    root=Path(a.run)

    required=["events.csv","backend.csv","frontend.csv","imu.csv","attitude.csv","range.csv","selected_frames.csv","selected.mjpg","MANIFEST.txt"]
    missing=[x for x in required if not (root/x).exists()]
    if missing:
        raise SystemExit("ERROR missing: "+", ".join(missing))

    ev=rows(root/"events.csv")
    be=rows(root/"backend.csv")
    fr=rows(root/"frontend.csv")
    imu=rows(root/"imu.csv")
    att=rows(root/"attitude.csv")
    rng=rows(root/"range.csv")
    frames=rows(root/"selected_frames.csv")

    starts=[r for r in ev if r["event"]=="MOVE_START"]
    ends=[r for r in ev if r["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1:
        raise SystemExit(f"ERROR expected 1 MOVE_START/1 MOVE_END, got {len(starts)}/{len(ends)}")
    es,ee=starts[0],ends[0]
    t0=i(es["wall_ns"]); t1=i(ee["wall_ns"])
    if t1<=t0: raise SystemExit("ERROR MOVE_END <= MOVE_START")
    dur=(t1-t0)/1e9
    w=int(a.window_ms*1e6)

    print("="*118)
    print("CLEAN-01 STANDALONE — SAME-RUN INTEGRITY + VIO ENDPOINT ANALYSIS")
    print("="*118)
    print(f"run={root}")
    print(f"truth={a.truth_mm:.1f} mm")
    print(f"MOVE duration={dur:.3f} s")
    print(f"events={len(ev)} backend={len(be)} frontend={len(fr)} imu={len(imu)} attitude={len(att)} range={len(rng)} selected_frames={len(frames)}")
    print(f"selected.mjpg bytes={(root/'selected.mjpg').stat().st_size}")

    # Exact event-captured backend state (copied into events at key press).
    A=event_state(es); B=event_state(ee)
    dx,dy,dz,h,d=displacement(A,B)
    print("\nEVENT-CAPTURED VIO ENDPOINTS")
    print("-"*118)
    print(f"A P=[{A['px']:+.6f},{A['py']:+.6f},{A['pz']:+.6f}] m")
    print(f"B P=[{B['px']:+.6f},{B['py']:+.6f},{B['pz']:+.6f}] m")
    print(f"dP=[{dx*1000:+.2f},{dy*1000:+.2f},{dz*1000:+.2f}] mm")
    print(f"horizontal={h*1000:.2f} mm   3D={d*1000:.2f} mm")
    print(f"horizontal scale vs {a.truth_mm:.1f}mm = {h*1000/a.truth_mm:.6f}")
    print(f"horizontal error = {h*1000-a.truth_mm:+.2f} mm")

    # Independent fixed windows; report only, no tuning.
    pre=mean_state(window(be,t0-w,t0))
    post=mean_state(window(be,t1,t1+w))
    print(f"\nFIXED {a.window_ms:.0f} ms BACKEND WINDOWS")
    print("-"*118)
    if pre and post:
        wx,wy,wz,wh,wd=displacement(pre,post)
        print(f"PRE n={pre['n']} mean P=[{pre['px']:+.6f},{pre['py']:+.6f},{pre['pz']:+.6f}]")
        print(f"POST n={post['n']} mean P=[{post['px']:+.6f},{post['py']:+.6f},{post['pz']:+.6f}]")
        print(f"dP=[{wx*1000:+.2f},{wy*1000:+.2f},{wz*1000:+.2f}] mm")
        print(f"horizontal={wh*1000:.2f} mm   3D={wd*1000:.2f} mm")
        print(f"horizontal scale={wh*1000/a.truth_mm:.6f} error={wh*1000-a.truth_mm:+.2f} mm")
    else:
        print("insufficient backend states for one or both fixed windows")

    # Nearest callback age sanity.
    ns=nearest_state(be,t0); ne=nearest_state(be,t1)
    print("\nEVENT ↔ BACKEND CALLBACK AGE")
    print("-"*118)
    print(f"START nearest callback delta={(i(ns['callback_wall_ns'])-t0)/1e6:+.3f} ms  kf={ns['keyframe']}")
    print(f"END   nearest callback delta={(i(ne['callback_wall_ns'])-t1)/1e6:+.3f} ms  kf={ne['keyframe']}")

    # Move-scoped sensor/frontend/frame counts using wall clock where available.
    be_move=[r for r in be if t0<=i(r["callback_wall_ns"])<=t1]
    fr_move=[r for r in fr if t0<=i(r["callback_wall_ns"])<=t1]
    imu_move=[r for r in imu if t0<=i(r["recv_wall_ns"])<=t1]
    att_move=[r for r in att if t0<=i(r["recv_wall_ns"])<=t1]
    rng_move=[r for r in rng if t0<=i(r["recv_wall_ns"])<=t1]

    # selected frames are camera timestamps, not callback wall timestamps. Map event states by backend timestamp bounds.
    ts0=i(es["state_timestamp_ns"]); ts1=i(ee["state_timestamp_ns"])
    frame_move=[r for r in frames if ts0<=i(r["timestamp_ns"])<=ts1] if ts0 and ts1 else []

    print("\nMOVE-SCOPED DATA")
    print("-"*118)
    print(f"backend states={len(be_move)} frontend callbacks={len(fr_move)} imu={len(imu_move)} attitude={len(att_move)} range={len(rng_move)} selected_frames={len(frame_move)}")

    if fr_move:
        tracked=[i(r["tracked"]) for r in fr_move]
        inliers=[i(r["inliers"]) for r in fr_move]
        put=[i(r["putatives"]) for r in fr_move]
        valid=sum(1 for r in fr_move if r.get("mono_pose_valid","0")=="1")
        ratios=[inn/p if p>0 else 0 for inn,p in zip(inliers,put)]
        print(f"frontend tracked med={med(tracked):.1f} inliers med={med(inliers):.1f} putatives med={med(put):.1f}")
        print(f"mono_pose_valid={valid}/{len(fr_move)} ({100*valid/len(fr_move):.1f}%) inlier_ratio med={med(ratios):.3f}")

    if att_move:
        rr=[math.degrees(f(r["roll"])) for r in att_move]
        pp=[math.degrees(f(r["pitch"])) for r in att_move]
        yy=[math.degrees(f(r["yaw"])) for r in att_move]
        rs=span(rr); ps=span(pp); ys=span(yy)
        print(f"FC attitude span: roll={rs[2]:.3f}° pitch={ps[2]:.3f}° yaw={ys[2]:.3f}°")

    if rng_move:
        cm=[i(r["current_cm"]) for r in rng_move if i(r["current_cm"])>0]
        if cm:
            print(f"TF-Luna during move: median={med(cm):.2f} cm min={min(cm)} max={max(cm)} n={len(cm)}")

    # Pre/post range, purely observational.
    pre_rng=[i(r["current_cm"]) for r in rng if t0-w<=i(r["recv_wall_ns"])<=t0 and i(r["current_cm"])>0]
    post_rng=[i(r["current_cm"]) for r in rng if t1<=i(r["recv_wall_ns"])<=t1+w and i(r["current_cm"])>0]
    if pre_rng or post_rng:
        print(f"TF-Luna fixed windows: PRE med={med(pre_rng):.2f} cm (n={len(pre_rng)}) POST med={med(post_rng):.2f} cm (n={len(post_rng)})")

    print("\nINTERPRETATION RULE")
    print("-"*118)
    print("This report uses only this CLEAN-01 archive plus the explicit 500 mm truth supplied on the command line.")
    print("No historical run, focal multiplier, camera-height correction, timing target, or old pass threshold is read.")
    print("="*118)

if __name__=="__main__":
    main()
