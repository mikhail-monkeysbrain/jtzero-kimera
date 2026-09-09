#!/usr/bin/env python3
"""STEP 2/8 — offline metric motion estimator.

Potential production architecture prototype:
OV9281 median optical flow + TF-Luna height + FC attitude rotation compensation
-> per-sample metric dx/dy -> integrated XY.

Uses only existing archives. No Kimera backend terms are used in the estimator.
"""
import argparse,csv,math,statistics
from pathlib import Path

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641

# LEGACY camera->body rotation retained only for Step-2 reproducibility.\n# Step 3 audits this against the current LeftCameraParams.yaml; do not treat it as current production geometry.
RBC=[
 [0.012724080, 0.995473080, 0.094188300],
 [0.998083560,-0.006939440,-0.061490300],
 [-0.060558320,0.094790200,-0.993653620],
]

def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mt(A): return [list(x) for x in zip(*A)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def rx(a):
    c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def ry(a):
    c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def rz(a):
    c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def rpy(r,p,y): return mm(rz(y),mm(ry(p),rx(r)))

def unwrap_deg(v):
    if not v:return []
    out=[v[0]]
    for x in v[1:]:
        y=x
        while y-out[-1]>180:y-=360
        while y-out[-1]<-180:y+=360
        out.append(y)
    return out

def interp(xs,ys,x):
    if x<=xs[0]:return ys[0]
    if x>=xs[-1]:return ys[-1]
    lo,hi=0,len(xs)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if xs[m]<=x:lo=m
        else:hi=m
    d=xs[hi]-xs[lo]
    a=0 if d==0 else (x-xs[lo])/d
    return ys[lo]*(1-a)+ys[hi]*a

def read_attitude(path):
    rows=list(csv.reader(path.open()))
    if not rows:raise RuntimeError("empty attitude CSV")
    try:
        float(rows[0][0]); header=None; data=rows
    except:
        header=rows[0]; data=rows[1:]
    if header:
        idx={k:i for i,k in enumerate(header)}
        ti=idx.get("recv_ns",0); ri=idx.get("roll_deg",2); pi=idx.get("pitch_deg",3); yi=idx.get("yaw_deg",4)
    else:
        ti,ri,pi,yi=0,2,3,4
    T=[float(r[ti]) for r in data]
    RR=[float(r[ri]) for r in data]
    PP=[float(r[pi]) for r in data]
    YY=unwrap_deg([float(r[yi]) for r in data])
    return T,RR,PP,YY

def pose_at(t,T,RR,PP,YY):
    # FC ATTITUDE is FRD; project convention is FLU.
    rr=math.radians(interp(T,RR,t))
    pp=math.radians(-interp(T,PP,t))
    yy=math.radians(-interp(T,YY,t))
    return rpy(rr,pp,yy)

def integrate(rows,bounds,T,RR,PP,YY,mode,height_fixed=None):
    sx=sy=path=0.0
    rot_px=[]
    steps=[]
    for i,row in enumerate(rows):
        h=float(row["height_m"]) if height_fixed is None else height_fixed
        if mode=="affine":
            fx=float(row["tx_px"]); fy=float(row["ty_px"])
            rfx=rfy=0.0
        else:
            fx=float(row["med_flow_x_px"]); fy=float(row["med_flow_y_px"])
            rfx=rfy=0.0
            if mode=="flow_rot":
                A=pose_at(bounds[i],T,RR,PP,YY)
                B=pose_at(bounds[i+1],T,RR,PP,YY)
                rel=mm(mt(RBC),mm(mt(B),mm(A,RBC)))
                u=float(row["c0x"]); v=float(row["c0y"])
                q=mv(rel,[(u-CX)/FX,(v-CY)/FY,1.0])
                if abs(q[2])>1e-12:
                    ur=FX*q[0]/q[2]+CX
                    vr=FY*q[1]/q[2]+CY
                    rfx=ur-u; rfy=vr-v
                    fx-=rfx; fy-=rfy
        # Image displacement -> planar metric displacement.
        dx=-fx*h/FX
        dy=-fy*h/FY
        sx+=dx; sy+=dy
        ds=math.hypot(dx,dy); path+=ds
        rot_px.append(math.hypot(rfx,rfy))
        steps.append((dx,dy,h,fx,fy))
    return dict(x=sx,y=sy,net=math.hypot(sx,sy),path=path,rot_px=rot_px,steps=steps)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    R=Path(a.run).expanduser()

    forensic=list(csv.DictReader((R/"jtzero_v43_camera_forensic.csv").open()))
    events=list(csv.DictReader((R/"jtzero_500mm_v25_events.csv").open()))
    if len(events)<2:
        raise SystemExit("ERROR: need at least start/end events")
    T,RR,PP,YY=read_attitude(R/"jtzero_500mm_v25_attitude.csv")

    start=float(events[0]["event_wall_ns"])
    end=float(events[1]["event_wall_ns"])
    N=len(forensic)
    bounds=[start+(end-start)*i/N for i in range(N+1)]
    truth=a.truth_mm/1000.0

    cases=[
      ("affine + logged height", integrate(forensic,bounds,T,RR,PP,YY,"affine")),
      ("median flow + logged height", integrate(forensic,bounds,T,RR,PP,YY,"flow")),
      ("flow + FC rotation + logged height", integrate(forensic,bounds,T,RR,PP,YY,"flow_rot")),
      (f"flow + FC rotation + fixed {a.physical_height_mm:.1f}mm",
       integrate(forensic,bounds,T,RR,PP,YY,"flow_rot",a.physical_height_mm/1000.0)),
    ]

    print("="*118)
    print("STEP 2/8 — OFFLINE OF + HEIGHT + FC-ROTATION METRIC MOTION ESTIMATOR")
    print("="*118)
    print(f"run={R}")
    print(f"rows={N} duration={(end-start)/1e9:.3f}s truth={a.truth_mm:.1f}mm")
    hs=[float(r["height_m"])*1000 for r in forensic]
    print(f"logged height mean/median/min/max={statistics.mean(hs):.2f}/{statistics.median(hs):.2f}/{min(hs):.1f}/{max(hs):.1f} mm")
    print()
    print("RESULTS")
    print("-"*118)
    for name,z in cases:
        err=(z["net"]-truth)*1000
        scale=z["net"]/truth
        print(f"{name:43s}: net={z['net']*1000:8.2f}mm  x={z['x']*1000:+8.2f}  y={z['y']*1000:+8.2f}  "
              f"path={z['path']*1000:8.2f}  error={err:+7.2f}mm scale={scale:.5f}")

    zr=cases[2][1]
    print()
    print("ROTATION COMPENSATION")
    print("-"*118)
    print(f"per-step predicted rotational image motion mean/max="
          f"{statistics.mean(zr['rot_px']):.4f}/{max(zr['rot_px']):.4f} px")

    # This ratio is diagnostic only: no coefficient is applied.
    print()
    print("PRODUCTION-CANDIDATE BRANCH")
    print("-"*118)
    print("Candidate estimator = median OV9281 flow - predicted FC-attitude rotational flow, scaled per sample by logged TF-Luna height.")
    print(f"candidate endpoint={zr['net']*1000:.2f}mm, error={(zr['net']-truth)*1000:+.2f}mm.")
    print("No Kimera backend state, focal tuning, or empirical 500-mm correction coefficient is used.")
    print()
    print("STEP 2 DECISION")
    print("-"*118)
    print("- If candidate is materially closer to 500mm than the existing camera-only/Kimera branches, proceed to Step 3 with this estimator.")
    print("- If it remains strongly biased, Step 3 isolates units/axes/height/rotation using the SAME archive; no new physical pass yet.")
    print("- Do not tune a global scale in Step 2.")
    print("="*118)

if __name__=="__main__":
    main()
