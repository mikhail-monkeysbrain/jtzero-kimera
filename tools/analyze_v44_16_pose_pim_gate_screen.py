#!/usr/bin/env python3
"""V44.16 — cross-check the current-only VALID mono-pose jump against PIM/IMU evidence
and screen a conservative pre-fusion gate on existing archives.

No new physical run. The gate is NOT applied to production here; this script only
tests whether a direction-discontinuity rule would catch the bad current event
without firing on the reference run.
"""
import argparse,csv,math,statistics
from pathlib import Path

def load(p):
    if not p.exists(): return []
    with p.open(newline="") as f: return list(csv.DictReader(f))

def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d

def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d

def leg_time_window(run):
    legs=load(run/"jtzero_500mm_v25_legs.csv")
    back=load(run/"jtzero_500mm_v25_backend.csv")
    if not legs or not back: raise RuntimeError(f"missing legs/backend: {run}")
    lo=I(legs[0],"start_settled_kf"); hi=I(legs[0],"end_press_kf")
    sel=[r for r in back if lo<=I(r,"keyframe")<=hi]
    if len(sel)<2: raise RuntimeError(f"insufficient backend rows: {run}")
    return I(sel[0],"timestamp_ns"),I(sel[-1],"timestamp_ns")

def angle_deg(a,b):
    na=math.sqrt(sum(x*x for x in a)); nb=math.sqrt(sum(x*x for x in b))
    if na<1e-12 or nb<1e-12:return float("nan")
    c=sum(x*y for x,y in zip(a,b))/(na*nb)
    return math.degrees(math.acos(max(-1,min(1,c))))

def norm3(v): return math.sqrt(sum(x*x for x in v))

def q(v,p):
    if not v:return float("nan")
    s=sorted(v); x=(len(s)-1)*p; i=int(math.floor(x)); j=int(math.ceil(x))
    return s[i] if i==j else s[i]*(j-x)+s[j]*(x-i)

def series(run):
    t0,t1=leg_time_window(run)
    fr=[r for r in load(run/"jtzero_500mm_v25_frontend.csv") if t0<=I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1]
    out=[]; prev_valid=None
    for idx,r in enumerate(fr):
        t=(F(r,"mono_body_tx"),F(r,"mono_body_ty"),F(r,"mono_body_tz"))
        tn=norm3(t) if all(math.isfinite(x) for x in t) else float("nan")
        h=math.hypot(t[0],t[1]) if all(math.isfinite(x) for x in t[:2]) else float("nan")
        tilt=math.degrees(math.atan2(abs(t[2]),h)) if h>1e-12 and math.isfinite(t[2]) else float("nan")
        jump=float("nan")
        if I(r,"mono_pose_valid")==1 and r.get("mono_status")=="VALID":
            if prev_valid is not None: jump=angle_deg(prev_valid,t)
            prev_valid=t
        dp=(F(r,"pim_dpx"),F(r,"pim_dpy"),F(r,"pim_dpz"))
        dv=(F(r,"pim_dvx"),F(r,"pim_dvy"),F(r,"pim_dvz"))
        drot=(F(r,"pim_droll_deg"),F(r,"pim_dpitch_deg"),F(r,"pim_dyaw_deg"))
        out.append(dict(
            idx=idx,p=idx/max(1,len(fr)-1),ts=I(r,"timestamp_ns"),
            status=r.get("mono_status",""),valid=I(r,"mono_pose_valid"),
            inlier=F(r,"mono_inlier_ratio"),tracked=F(r,"tracked_features"),
            put=F(r,"mono_putatives"),ins=F(r,"mono_inliers"),iters=F(r,"mono_ransac_iters"),
            tx=t[0],ty=t[1],tz=t[2],tn=tn,tilt=tilt,jump=jump,
            pim_valid=I(r,"pim_valid"),pim_dt=F(r,"pim_dt_s"),
            dp=dp,dpn=norm3(dp) if all(math.isfinite(x) for x in dp) else float("nan"),
            dv=dv,dvn=norm3(dv) if all(math.isfinite(x) for x in dv) else float("nan"),
            drot=drot,drotn=norm3(drot) if all(math.isfinite(x) for x in drot) else float("nan"),
        ))
    return out

def gate(r,jump_thr,tilt_thr):
    return r["status"]=="VALID" and r["valid"]==1 and math.isfinite(r["jump"]) and math.isfinite(r["tilt"]) and r["jump"]>=jump_thr and r["tilt"]>=tilt_thr

def summarize(name,R,jump_thr,tilt_thr):
    valid=[r for r in R if r["status"]=="VALID" and r["valid"]==1]
    fires=[r for r in valid if gate(r,jump_thr,tilt_thr)]
    dpns=[r["dpn"] for r in valid if math.isfinite(r["dpn"])]
    drots=[r["drotn"] for r in valid if math.isfinite(r["drotn"])]
    print(f"\n{name}: keyframes={len(R)} VALID={len(valid)} gate_fires={len(fires)}")
    print(f"  PIM dp norm median/p10={statistics.median(dpns) if dpns else float('nan'):.6f}/{q(dpns,.10):.6f}")
    print(f"  PIM rot norm median/p90={statistics.median(drots) if drots else float('nan'):.3f}/{q(drots,.90):.3f} deg")
    for r in fires:
        pct_low=sum(x<=r["dpn"] for x in dpns)/len(dpns)*100 if dpns and math.isfinite(r["dpn"]) else float("nan")
        print(f"  FIRE p={r['p']:.0%} jump={r['jump']:.1f}deg tilt={r['tilt']:.1f}deg "
              f"inlier={r['inlier']:.3f} tracked={r['tracked']:.0f} "
              f"PIM_dp={r['dpn']:.6f} (percentile={pct_low:.1f}%) PIM_drot={r['drotn']:.3f}deg "
              f"PIM_dt={r['pim_dt']:.4f}s")
        print(f"       mono=({r['tx']:+.5f},{r['ty']:+.5f},{r['tz']:+.5f}) "
              f"pim_dp=({r['dp'][0]:+.6f},{r['dp'][1]:+.6f},{r['dp'][2]:+.6f}) "
              f"pim_dv=({r['dv'][0]:+.6f},{r['dv'][1]:+.6f},{r['dv'][2]:+.6f})")
    return fires

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--reference",required=True)
    ap.add_argument("--current",required=True)
    ap.add_argument("--jump-deg",type=float,default=30.0)
    ap.add_argument("--tilt-deg",type=float,default=30.0)
    a=ap.parse_args()
    A=series(Path(a.reference)); B=series(Path(a.current))
    print("="*150)
    print("V44.16 — VALID MONO-POSE JUMP / PIM CROSS-CHECK + COUNTERFACTUAL GATE SCREEN")
    print("="*150)
    print(f"candidate gate: VALID && pose_valid && direction_jump>={a.jump_deg:.1f}deg && out-of-plane tilt>={a.tilt_deg:.1f}deg")
    FA=summarize("REFERENCE",A,a.jump_deg,a.tilt_deg)
    FB=summarize("CURRENT",B,a.jump_deg,a.tilt_deg)

    print("\nCURRENT LATE WINDOW")
    print("-"*150)
    print("prog status         inlier tracked | mono_tilt jump | PIM_dp_norm PIM_dv_norm PIM_rot_norm PIM_dt")
    for r in B:
        if r["p"]>=.75:
            print(f"{r['p']:4.0%} {r['status'] or '-':14s} {r['inlier']:6.3f} {r['tracked']:7.0f} | "
                  f"{r['tilt']:8.1f} {r['jump']:6.1f} | {r['dpn']:11.6f} {r['dvn']:11.6f} {r['drotn']:12.3f} {r['pim_dt']:6.4f}")

    print("\nDECISION")
    print("-"*150)
    if len(FA)==0 and len(FB)==1:
        r=FB[0]
        print("PASS FOR GATE CANDIDATE: exactly one current-only anomalous VALID pose is caught; reference false positives=0.")
        if math.isfinite(r["drotn"]) and r["drotn"]<5.0:
            print("PIM rotation is small while mono translation direction jumps strongly -> camera attitude change is not a plausible explanation.")
        print("NEXT: implement an opt-in diagnostic pre-fusion rejection gate with these thresholds, preserving production defaults OFF.")
    elif len(FA)==0 and len(FB)>1:
        print("Candidate is current-specific but catches multiple rows; inspect all fires before implementing a gate.")
    else:
        print("Candidate is not selective enough; do not implement a pre-fusion gate yet.")
    print("This is a counterfactual screen only; it does not alter Kimera output.")
    print("="*150)

if __name__=="__main__":main()
