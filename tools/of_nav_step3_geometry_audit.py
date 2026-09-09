#!/usr/bin/env python3
"""STEP 3/8 — geometry source audit on the SAME archived run.

Critical check: Step 2 used a stale hard-coded camera->body rotation. This tool
loads the current production LeftCameraParams.yaml and compares it against the
legacy rotation in one pass. No physical run is required.
"""
import argparse,csv,math,re,statistics
from pathlib import Path

LEGACY_FX=568.53170752165227
LEGACY_FY=569.68005562865858
LEGACY_CX=315.98271077441063
LEGACY_CY=239.88148589100641
LEGACY_RBC=[
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
    a=0.0 if d==0 else (x-xs[lo])/d
    return ys[lo]*(1-a)+ys[hi]*a

def read_current_camera(path):
    txt=path.read_text()
    m=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt)
    if not m: raise RuntimeError("intrinsics not found")
    intr=[float(x.strip()) for x in m.group(1).split(",")]
    fx,fy,cx,cy=intr[:4]
    m=re.search(r'T_BS:\s*\n\s*cols:\s*4\s*\n\s*rows:\s*4\s*\n\s*data:\s*\[([^\]]+)\]',txt,re.S)
    if not m: raise RuntimeError("T_BS not found")
    vals=[float(x.strip()) for x in m.group(1).replace("\n"," ").split(",")]
    R=[[vals[r*4+c] for c in range(3)] for r in range(3)]
    return fx,fy,cx,cy,R

def read_att(path):
    rows=list(csv.reader(path.open()))
    try: float(rows[0][0]); hdr=None; data=rows
    except: hdr=rows[0]; data=rows[1:]
    if hdr:
        idx={k:i for i,k in enumerate(hdr)}
        ti=idx.get("recv_ns",0);ri=idx.get("roll_deg",2);pi=idx.get("pitch_deg",3);yi=idx.get("yaw_deg",4)
    else:ti,ri,pi,yi=0,2,3,4
    T=[float(r[ti]) for r in data]
    RR=[float(r[ri]) for r in data]
    PP=[float(r[pi]) for r in data]
    YY=unwrap_deg([float(r[yi]) for r in data])
    return T,RR,PP,YY

def pose(t,T,RR,PP,YY):
    return rpy(math.radians(interp(T,RR,t)),
               math.radians(-interp(T,PP,t)),
               math.radians(-interp(T,YY,t)))

def integrate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,RBC,height_fixed=None):
    sx=sy=path=0.0; rot=[]
    for i,row in enumerate(fr):
        A=pose(bounds[i],T,RR,PP,YY); B=pose(bounds[i+1],T,RR,PP,YY)
        rel=mm(mt(RBC),mm(mt(B),mm(A,RBC)))
        u=float(row["c0x"]);v=float(row["c0y"])
        q=mv(rel,[(u-cx)/fx,(v-cy)/fy,1.0])
        rfx=rfy=0.0
        if abs(q[2])>1e-12:
            rfx=fx*q[0]/q[2]+cx-u
            rfy=fy*q[1]/q[2]+cy-v
        ofx=float(row["med_flow_x_px"])-rfx
        ofy=float(row["med_flow_y_px"])-rfy
        h=float(row["height_m"]) if height_fixed is None else height_fixed
        dx=-ofx*h/fx;dy=-ofy*h/fy
        sx+=dx;sy+=dy;path+=math.hypot(dx,dy);rot.append(math.hypot(rfx,rfy))
    return math.hypot(sx,sy),sx,sy,path,statistics.mean(rot),max(rot)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    run=Path(a.run).expanduser()
    cam=Path(a.camera_yaml)
    if not cam.is_absolute(): cam=Path.cwd()/cam

    fx,fy,cx,cy,Rcur=read_current_camera(cam)
    fr=list(csv.DictReader((run/"jtzero_v43_camera_forensic.csv").open()))
    ev=list(csv.DictReader((run/"jtzero_500mm_v25_events.csv").open()))
    T,RR,PP,YY=read_att(run/"jtzero_500mm_v25_attitude.csv")
    start,end=float(ev[0]["event_wall_ns"]),float(ev[1]["event_wall_ns"])
    N=len(fr);bounds=[start+(end-start)*i/N for i in range(N+1)]
    truth=a.truth_mm/1000.0
    hf=a.physical_height_mm/1000.0

    cases=[
      ("LEGACY extrinsic + logged h",LEGACY_FX,LEGACY_FY,LEGACY_CX,LEGACY_CY,LEGACY_RBC,None),
      ("CURRENT extrinsic + logged h",fx,fy,cx,cy,Rcur,None),
      (f"LEGACY extrinsic + fixed {a.physical_height_mm:.1f}mm",LEGACY_FX,LEGACY_FY,LEGACY_CX,LEGACY_CY,LEGACY_RBC,hf),
      (f"CURRENT extrinsic + fixed {a.physical_height_mm:.1f}mm",fx,fy,cx,cy,Rcur,hf),
    ]

    print("="*122)
    print("STEP 3/8 — CURRENT CAMERA GEOMETRY VS STALE STEP-2 GEOMETRY")
    print("="*122)
    print(f"camera yaml: {cam}")
    print(f"current intrinsics fx/fy/cx/cy={fx:.6f}/{fy:.6f}/{cx:.6f}/{cy:.6f}")
    print("current R_BC/T_BS rotation:")
    for r in Rcur: print("  "+" ".join(f"{x:+.9f}" for x in r))
    print("\nlegacy R used by Step 2:")
    for r in LEGACY_RBC: print("  "+" ".join(f"{x:+.9f}" for x in r))

    # Quantify matrix disagreement directly.
    d=mm(mt(LEGACY_RBC),Rcur)
    c=max(-1.0,min(1.0,(d[0][0]+d[1][1]+d[2][2]-1)/2))
    angle=math.degrees(math.acos(c))
    print(f"legacy->current rotation disagreement: {angle:.3f} deg")

    print("\nSAME-RUN ESTIMATOR SWEEP")
    print("-"*122)
    for name,afx,afy,acx,acy,R,h in cases:
        net,x,y,path,rm,rmax=integrate(fr,bounds,T,RR,PP,YY,afx,afy,acx,acy,R,h)
        print(f"{name:44s}: net={net*1000:8.2f}mm err={(net-truth)*1000:+7.2f} "
              f"scale={net/truth:.5f} x={x*1000:+8.2f} y={y*1000:+8.2f} "
              f"rot_mean/max={rm:.4f}/{rmax:.4f}px")

    cur=integrate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,Rcur,None)
    req_h=hf*(truth/integrate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,Rcur,hf)[0])
    req_fk=cur[0]/truth
    print("\nDISCRIMINATORS")
    print("-"*122)
    print(f"with CURRENT geometry + fixed physical height, height needed for exact 500mm by linear scaling: {req_h*1000:.2f}mm")
    print(f"with CURRENT geometry + logged height, isotropic focal multiplier that would reconcile endpoint: {req_fk:.5f}")
    print("These are diagnostics only; neither value is applied.")
    print("\nSTEP 3 DECISION")
    print("-"*122)
    print("- If CURRENT extrinsic materially improves the result, Step 2 was using stale geometry and the estimator implementation must be corrected before any new run.")
    print("- If CURRENT and LEGACY are nearly identical, remaining bias is not caused by this stale extrinsic.")
    print("- No global scale correction is permitted in Step 3.")
    print("="*122)

if __name__=="__main__":
    main()
