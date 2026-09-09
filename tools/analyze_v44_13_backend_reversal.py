#!/usr/bin/env python3
"""V44.13 — late-run backend reversal / frontend correlation.

Uses only archived CSVs. Frontend is matched to backend by nearest timestamp,
not keyframe number, because V44.12 showed keyframe IDs are not directly shared.
"""
import argparse,csv,math,bisect,statistics
from pathlib import Path

def loadcsv(p):
    if not p.exists(): return []
    with p.open(newline="") as f: return list(csv.DictReader(f))

def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d

def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d

def leg_bounds(run):
    legs=loadcsv(run/"jtzero_500mm_v25_legs.csv")
    if not legs: raise RuntimeError(f"missing legs csv: {run}")
    r=legs[0]
    return I(r,"start_settled_kf"),I(r,"end_press_kf")

def nearest_frontend(front,t):
    if not front:return None,None
    ts=[I(r,"timestamp_ns") for r in front]
    j=bisect.bisect_left(ts,t)
    cand=[]
    if j<len(front): cand.append((abs(ts[j]-t),front[j]))
    if j>0: cand.append((abs(ts[j-1]-t),front[j-1]))
    if not cand:return None,None
    dt,r=min(cand,key=lambda x:x[0])
    return r,dt/1e6

def get_series(run):
    k0,k1=leg_bounds(run)
    b=[r for r in loadcsv(run/"jtzero_500mm_v25_backend.csv") if k0<=I(r,"keyframe")<=k1]
    f=loadcsv(run/"jtzero_500mm_v25_frontend.csv")
    f=sorted([r for r in f if I(r,"timestamp_ns")>0],key=lambda r:I(r,"timestamp_ns"))
    if len(b)<2: raise RuntimeError(f"insufficient backend rows: {run}")
    x0,y0,z0=F(b[0],"px_m"),F(b[0],"py_m"),F(b[0],"pz_m")
    rows=[]
    for j,r in enumerate(b):
        x=(F(r,"px_m")-x0)*1000
        y=(F(r,"py_m")-y0)*1000
        z=(F(r,"pz_m")-z0)*1000
        fr,dtms=nearest_frontend(f,I(r,"timestamp_ns"))
        rows.append({
            "i":j,"kf":I(r,"keyframe"),"t":I(r,"timestamp_ns"),
            "p":j/max(1,len(b)-1),"x":x,"y":y,"z":z,"h":math.hypot(x,y),
            "vx":F(r,"vx_mps"),"vy":F(r,"vy_mps"),"vz":F(r,"vz_mps"),
            "front_dt_ms":dtms if dtms is not None else float("nan"),
            "status":(fr or {}).get("mono_status",""),
            "inlier":F(fr or {},"mono_inlier_ratio"),
            "tracked":F(fr or {},"tracked_features"),
            "pose_valid":I(fr or {},"mono_pose_valid"),
            "mtx":F(fr or {},"mono_body_tx"),"mty":F(fr or {},"mono_body_ty"),"mtz":F(fr or {},"mono_body_tz"),
        })
    # dominant displacement unit vector from the first 80% endpoint, before the bad run reverses
    anchor=rows[min(len(rows)-1,max(1,int(round(.80*(len(rows)-1)))))]
    n=math.hypot(anchor["x"],anchor["y"])
    ux,uy=(anchor["x"]/n,anchor["y"]/n) if n>1e-9 else (1.0,0.0)
    for i,r in enumerate(rows):
        r["along"]=r["x"]*ux+r["y"]*uy
        if i==0:
            r["d_along"]=0.0;r["d_h"]=0.0;r["dt_s"]=float("nan")
        else:
            prev=rows[i-1]
            r["d_along"]=r["along"]-prev["along"]
            r["d_h"]=r["h"]-prev["h"]
            dt=(r["t"]-prev["t"])/1e9
            r["dt_s"]=dt
            if not math.isfinite(r["vx"]) and dt>0:
                r["vx_est"]=(r["x"]-prev["x"])/1000/dt
                r["vy_est"]=(r["y"]-prev["y"])/1000/dt
            else:
                r["vx_est"]=r["vx"]; r["vy_est"]=r["vy"]
    return rows,(ux,uy)

def summarize(name,rows):
    peak=max(rows,key=lambda r:r["along"])
    end=rows[-1]
    neg=[r for r in rows[1:] if r["d_along"]<-0.5]
    firstneg=neg[0] if neg else None
    late=[r for r in rows if r["p"]>=.75]
    weak=[r for r in late if math.isfinite(r["inlier"]) and r["inlier"]<.40]
    invalid=[r for r in late if r["status"] and r["status"]!="VALID"]
    return {
      "name":name,"peak":peak,"end":end,"firstneg":firstneg,"neg_n":len(neg),
      "late_n":len(late),"weak_n":len(weak),"invalid_n":len(invalid),
      "loss_after_peak":peak["along"]-end["along"]
    }

def print_tail(name,rows,startp):
    print(f"\n{name} — BACKEND STEPS FROM {startp:.0%}")
    print("-"*164)
    print("prog  kf   along_mm  dAlong  horiz_mm   dx_mm   dy_mm | frontend_dt status   inlier tracked poseV | mono_tilt")
    for r in rows:
        if r["p"]<startp: continue
        tilt=float("nan")
        if all(math.isfinite(v) for v in (r["mtx"],r["mty"],r["mtz"])):
            tilt=math.degrees(math.atan2(abs(r["mtz"]),math.hypot(r["mtx"],r["mty"])))
        print(f"{r['p']:4.0%} {r['kf']:4d} {r['along']:10.1f} {r['d_along']:+7.1f} {r['h']:9.1f} {r['x']:7.1f} {r['y']:7.1f} | "
              f"{r['front_dt_ms']:8.2f}ms {r['status'] or '-':>8s} {r['inlier']:7.3f} {r['tracked']:7.0f} {r['pose_valid']:5d} | {tilt:8.2f}")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--reference",required=True)
    ap.add_argument("--current",required=True)
    ap.add_argument("--tail-start",type=float,default=.65)
    a=ap.parse_args()
    A,ua=get_series(Path(a.reference)); B,ub=get_series(Path(a.current))
    SA=summarize("REFERENCE",A); SB=summarize("CURRENT",B)

    print("="*164)
    print("V44.13 — BACKEND LATE-RUN REVERSAL / FRONTEND TIMESTAMP CORRELATION")
    print("="*164)
    print(f"reference rows={len(A)} current rows={len(B)}")
    print(f"reference dominant u=({ua[0]:+.4f},{ua[1]:+.4f}) current dominant u=({ub[0]:+.4f},{ub[1]:+.4f})")
    print()
    for S in (SA,SB):
        p=S["peak"]; e=S["end"]; fn=S["firstneg"]
        print(f"{S['name']:9s}: peak along={p['along']:.1f}mm at {p['p']:.0%}; end={e['along']:.1f}mm; post-peak loss={S['loss_after_peak']:.1f}mm")
        print(f"           negative backend steps >0.5mm={S['neg_n']}; late weak frontend={S['weak_n']}/{S['late_n']}; late non-VALID={S['invalid_n']}/{S['late_n']}")
        if fn:
            print(f"           first negative step: progress={fn['p']:.0%} kf={fn['kf']} dAlong={fn['d_along']:+.1f}mm frontend={fn['status'] or '-'} inlier={fn['inlier']:.3f} dt={fn['front_dt_ms']:.2f}ms")
    print_tail("REFERENCE",A,a.tail_start)
    print_tail("CURRENT",B,a.tail_start)

    print("\nDISCRIMINATOR")
    print("-"*164)
    loss_extra=SB["loss_after_peak"]-SA["loss_after_peak"]
    print(f"extra current post-peak loss versus reference = {loss_extra:+.1f} mm")
    if SB["loss_after_peak"]>10 and SB["weak_n"]==0 and SB["invalid_n"]==0:
        print("Late backend reversal occurs without weak/non-VALID nearest frontend rows -> prioritize backend/IMU state evolution.")
    elif SB["loss_after_peak"]>10 and (SB["weak_n"]>0 or SB["invalid_n"]>0):
        print("Late backend reversal overlaps weak/non-VALID frontend rows -> visual-update rejection/quality remains a primary branch.")
    else:
        print("No large isolated late reversal found; inspect earlier cumulative scale divergence.")
    print("Nearest frontend is timestamp-matched; a large frontend_dt_ms means the streams are not directly alignable and must not be over-interpreted.")
    print("="*164)

if __name__=="__main__": main()
