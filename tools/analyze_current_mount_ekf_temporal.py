#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

def fv(r,k,d=float("nan")):
    try:return float(r[k])
    except:return d

def iv(r,k,d=0):
    try:return int(float(r[k]))
    except:return d

def interp(ts,ys,t):
    if not ts:return float("nan")
    if t<=ts[0]:return ys[0]
    if t>=ts[-1]:return ys[-1]
    lo,hi=0,len(ts)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if ts[m]<=t:lo=m
        else:hi=m
    u=(t-ts[lo])/(ts[hi]-ts[lo])
    return ys[lo]*(1-u)+ys[hi]*u

def linfit(x,y):
    n=len(x)
    if n<3:return float("nan"),float("nan"),float("nan")
    mx=sum(x)/n; my=sum(y)/n
    sxx=sum((a-mx)**2 for a in x)
    if sxx<=0:return float("nan"),float("nan"),float("nan")
    a=sum((x[i]-mx)*(y[i]-my) for i in range(n))/sxx
    b=my-a*mx
    rms=math.sqrt(sum((y[i]-(a*x[i]+b))**2 for i in range(n))/n)
    return a,b,rms

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    args=ap.parse_args()
    rows=list(csv.DictReader(args.csv.open(newline="")))
    if not rows:raise SystemExit("empty CSV")

    t0=fv(rows[0],"mono_ns")*1e-9
    data=[]
    rawx=rawy=0.0
    prev_t=None
    for r in rows:
        t=fv(r,"mono_ns")*1e-9-t0
        valid=iv(r,"valid")==1 and iv(r,"flow_sent")==1
        rng=fv(r,"range_to_fc_m")
        fx=fv(r,"flow_send_x"); fy=fv(r,"flow_send_y")
        if valid and all(map(math.isfinite,[t,rng,fx,fy])) and rng>0:
            if prev_t is not None:
                dt=t-prev_t
                if 0<dt<0.2:
                    rawx+=fx*rng*dt
                    rawy+=fy*rng*dt
            prev_t=t
        else:
            prev_t=None
        ekf_ok=iv(r,"ekf_local_valid")==1
        en=fv(r,"ekf_x_ned"); ee=fv(r,"ekf_y_ned")
        vn=fv(r,"ekf_vx_ned"); ve=fv(r,"ekf_vy_ned")
        data.append((t,iv(r,"guide_stage",-1),rawx,rawy,ekf_ok,en,ee,vn,ve,rng))

    # Determine move window and reference vectors.
    move=[x for x in data if x[1]==1]
    if len(move)<5:raise SystemExit("guide_stage=1 not found")
    tm0,tm1=move[0][0],move[-1][0]

    # RAW direction from start to end of move.
    rx0,ry0=move[0][2],move[0][3]
    rdx=move[-1][2]-rx0; rdy=move[-1][3]-ry0
    rn=math.hypot(rdx,rdy)
    if rn<1e-6:raise SystemExit("RAW displacement too small")
    ur=(rdx/rn,rdy/rn)

    loc_move=[x for x in move if x[4]]
    if len(loc_move)<5:raise SystemExit("no EKF local rows")
    e0n,e0e=loc_move[0][5],loc_move[0][6]
    edn=loc_move[-1][5]-e0n; ede=loc_move[-1][6]-e0e
    enrm=math.hypot(edn,ede)
    ue=(edn/enrm,ede/enrm) if enrm>1e-9 else (1.0,0.0)

    # Build scalar progress series over whole run.
    tr=[]; pr=[]
    te=[]; pe=[]; ve=[]
    for x in data:
        t=x[0]
        prj=(x[2]-rx0)*ur[0]+(x[3]-ry0)*ur[1]
        tr.append(t); pr.append(prj)
        if x[4]:
            ep=(x[5]-e0n)*ue[0]+(x[6]-e0e)*ue[1]
            ev=x[7]*ue[0]+x[8]*ue[1]
            te.append(t);pe.append(ep);ve.append(ev)

    # Fit scale+offset for lag hypotheses using the move plus 3 s post.
    fit_end=tm1+3.0
    best=None
    for lag_ms in range(-800,801,20):
        lag=lag_ms/1000.0
        xs=[];ys=[]
        for t,y in zip(te,pe):
            if t<tm0 or t>fit_end:continue
            x=interp(tr,pr,t-lag)
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x);ys.append(y)
        a,b,rms=linfit(xs,ys)
        if not math.isfinite(rms):continue
        cand=(rms,lag_ms,a,b,len(xs))
        if best is None or cand<best:best=cand

    # End-of-move and post-catchup.
    def ekf_progress_at(t):
        return interp(te,pe,t)
    def raw_progress_at(t):
        return interp(tr,pr,t)

    checkpoints=[0,0.5,1,2,3,5,8]
    raw_end=raw_progress_at(tm1)
    ekf_end=ekf_progress_at(tm1)

    print("===== EKF vs RAW TEMPORAL FORENSIC =====")
    print(f"move window = {tm1-tm0:.3f} s")
    print(f"RAW move progress = {raw_end*1000:.1f} mm")
    print(f"EKF move progress = {ekf_end*1000:.1f} mm")
    if abs(raw_end)>1e-9:print(f"EKF/RAW at move end = {ekf_end/raw_end:.4f}")
    print()
    print("Post-stop catch-up (projected along each estimator's own motion direction):")
    for s in checkpoints:
        t=tm1+s
        if t>te[-1]+1e-6:continue
        ep=ekf_progress_at(t)
        rp=raw_progress_at(t)
        print(f"  +{s:>3.1f}s  RAW={rp*1000:8.1f} mm  EKF={ep*1000:8.1f} mm  delta={(ep-ekf_end)*1000:+7.1f} mm")

    if best:
        rms,lag_ms,a,b,n=best
        print()
        print("Best lag+scale fit over move + 3 s:")
        print(f"  lag = {lag_ms:+d} ms")
        print(f"  EKF ~= {a:.4f} * RAW(t-lag) + {b*1000:+.1f} mm")
        print(f"  fit RMS = {rms*1000:.1f} mm, samples={n}")

    # Velocity after stop.
    post=[(t,v) for t,v in zip(te,ve) if tm1<=t<=min(tm1+8.0,te[-1])]
    if post:
        abs_v=[abs(v) for _,v in post]
        print()
        print(f"post EKF projected |v| median/max = {statistics.median(abs_v):.4f}/{max(abs_v):.4f} m/s")
        # signed residual displacement from end to final available post.
        print(f"post EKF position change = {(post and ekf_progress_at(post[-1][0])-ekf_end)*1000:+.1f} mm")

    print()
    if best:
        if abs(best[1])>250 and 0.9<=best[2]<=1.1:
            print("INTERPRETATION: mostly temporal lag; scale is near 1 after alignment.")
        elif best[2]<0.9:
            print("INTERPRETATION: EKF remains lower-scale than RAW even after lag alignment.")
        else:
            print("INTERPRETATION: no single simple lag/scale model fully explains EKF tracking.")

if __name__=="__main__":main()
