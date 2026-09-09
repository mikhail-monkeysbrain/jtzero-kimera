#!/usr/bin/env python3
import argparse,csv,math,statistics,re
from pathlib import Path
import cv2
import numpy as np

def mm(A,B): return A @ B

def rpy(r,p,y):
    cr,sr=np.cos(r),np.sin(r); cp,sp=np.cos(p),np.sin(p); cy,sy=np.cos(y),np.sin(y)
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]],float)
    Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]],float)
    Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]],float)
    return Rz@Ry@Rx

def read_csv(path):
    with path.open(newline="") as f: return list(csv.DictReader(f))

def read_camera(path):
    txt=Path(path).read_text()
    mi=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt)
    md=re.search(r'distortion_coefficients:\s*\[([^\]]+)\]',txt)
    mt=re.search(r'T_BS:\s*\n\s*cols:\s*4\s*\n\s*rows:\s*4\s*\n\s*data:\s*\[([^\]]+)\]',txt,re.S)
    if not(mi and md and mt): raise RuntimeError("camera yaml parse failed")
    fx,fy,cx,cy=[float(x.strip()) for x in mi.group(1).split(",")[:4]]
    D=np.array([float(x.strip()) for x in md.group(1).split(",")],float)
    vals=[float(x.strip()) for x in mt.group(1).replace("\n"," ").split(",")]
    RBC=np.array([[vals[r*4+c] for c in range(3)] for r in range(3)],float)
    return fx,fy,cx,cy,D,RBC

def interp(rows,t,key):
    ts=[int(r["recv_mono_ns"]) for r in rows]
    if t<=ts[0]: return float(rows[0][key])
    if t>=ts[-1]: return float(rows[-1][key])
    lo,hi=0,len(ts)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if ts[m]<=t: lo=m
        else: hi=m
    a=(t-ts[lo])/(ts[hi]-ts[lo])
    return float(rows[lo][key])*(1-a)+float(rows[hi][key])*a

def attitude_R(rows,t):
    # R1 attitude.csv stores MAVLink ATTITUDE in FRD/NED convention.
    roll=interp(rows,t,"roll")
    pitch=interp(rows,t,"pitch")
    yaw=interp(rows,t,"yaw")
    # Same FRD->FLU sign convention used by existing project planar tool.
    return rpy(roll,-pitch,-yaw)

def range_m(rows,t):
    valid=[r for r in rows if int(r["valid"])==1]
    return interp(valid,t,"distance_cm")/100.0

def decode(mj,row):
    mj.seek(int(row["offset"])); b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): return None
    return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

def undistort_pixels(pts,K,D):
    p=np.asarray(pts,np.float64).reshape(-1,1,2)
    q=cv2.undistortPoints(p,K,D).reshape(-1,2)
    return np.c_[q,np.ones(len(q))]

def pair_motion(im0,im1,K,D,RWC0,RWC1,h0,h1):
    g0=cv2.createCLAHE(2.0,(8,8)).apply(im0)
    g1=cv2.createCLAHE(2.0,(8,8)).apply(im1)
    p0=cv2.goodFeaturesToTrack(g0,700,0.01,6,blockSize=7)
    if p0 is None or len(p0)<30: return None
    p1,st1,_=cv2.calcOpticalFlowPyrLK(g0,g1,p0,None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
    if p1 is None: return None
    p0b,st2,_=cv2.calcOpticalFlowPyrLK(g1,g0,p1,None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
    if p0b is None: return None
    a=p0.reshape(-1,2); b=p1.reshape(-1,2); back=p0b.reshape(-1,2)
    good=(st1.ravel()==1)&(st2.ravel()==1)&(np.linalg.norm(a-back,axis=1)<1.0)
    a=a[good]; b=b[good]
    if len(a)<20: return None
    H,mask=cv2.findHomography(a,b,cv2.RANSAC,2.0)
    if H is None or mask is None: return None
    inl=mask.ravel().astype(bool); a=a[inl]; b=b[inl]
    if len(a)<15: return None
    qa=undistort_pixels(a,K,D); qb=undistort_pixels(b,K,D)
    rw0=(RWC0@qa.T).T; rw1=(RWC1@qb.T).T
    ok=(np.abs(rw0[:,2])>1e-8)&(np.abs(rw1[:,2])>1e-8)
    lam0=-h0/rw0[:,2]; lam1=-h1/rw1[:,2]
    ok &= (lam0>0)&(lam1>0)
    if np.count_nonzero(ok)<12: return None
    G0=rw0[ok]*lam0[ok,None]; G1=rw1[ok]*lam1[ok,None]
    d=G0[:,:2]-G1[:,:2]
    med=np.median(d,axis=0)
    resid=np.linalg.norm(d-med,axis=1)
    keep=resid<=np.percentile(resid,70)
    if np.count_nonzero(keep)>=8: med=np.median(d[keep],axis=0)
    return med, len(a), int(np.count_nonzero(ok)), float(np.median(resid))

def run_case(frames,mj,att,rng,K,D,RBC,t0,t1,period_ns,height_mode):
    sel=[]
    last=-10**30
    for r in frames:
        t=int(r["recv_mono_ns"])
        if t<t0 or t>t1: continue
        if not sel or t-last>=period_ns:
            sel.append(r); last=t
    sx=sy=path=0.0; valid=fail=0; stats=[]
    prev=None; prevrow=None
    for row in sel:
        im=decode(mj,row)
        if im is None: continue
        if prev is None:
            prev,prevrow=im,row; continue
        ta=int(prevrow["recv_mono_ns"]); tb=int(row["recv_mono_ns"])
        R0=attitude_R(att,ta)@RBC; R1=attitude_R(att,tb)@RBC
        if isinstance(height_mode,float):
            h0=h1=height_mode
        elif height_mode=="luna":
            h0=range_m(rng,ta); h1=range_m(rng,tb)
        elif isinstance(height_mode,tuple) and height_mode[0]=="luna_offset":
            off=float(height_mode[1])
            h0=max(0.05,range_m(rng,ta)+off)
            h1=max(0.05,range_m(rng,tb)+off)
        else:
            raise RuntimeError("bad height mode")
        z=pair_motion(prev,im,K,D,R0,R1,h0,h1)
        if z is None:
            fail+=1
        else:
            d,nin,nray,res=z
            sx+=d[0]; sy+=d[1]; path+=float(np.linalg.norm(d)); valid+=1
            stats.append((nin,nray,res,float(np.linalg.norm(d))))
        prev,prevrow=im,row
    return dict(x=sx,y=sy,net=math.hypot(sx,sy),path=path,valid=valid,fail=fail,stats=stats,nsel=len(sel))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--period-ms",type=float,default=35.0)
    args=ap.parse_args()
    root=Path(args.run)
    frames=read_csv(root/"frames.csv"); ev=read_csv(root/"events.csv")
    att=read_csv(root/"attitude.csv"); rng=read_csv(root/"range.csv")
    starts=[int(r["recv_mono_ns"]) for r in ev if r["event"]=="MOVE_START"]
    ends=[int(r["recv_mono_ns"]) for r in ev if r["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1: raise SystemExit("ERROR: need one MOVE_START/MOVE_END")
    t0,t1=starts[0],ends[0]
    fx,fy,cx,cy,D,RBC=read_camera(Path(args.camera_yaml))
    truth=args.truth_mm/1000.0
    cases=[
      ("stored K, H=169.9mm",1.0,0.1699),
      ("stored K, H=170.2mm (R9)",1.0,0.1702),
      ("stored K, H=185.5mm",1.0,0.1855),
      ("stored K, raw Luna H",1.0,"luna"),
      ("stored K, Luna-20mm (production)",1.0,("luna_offset",-0.020)),
      ("stored K, Luna-16.36mm",1.0,("luna_offset",-0.01636)),
      ("R1 Kx1.0925, H=185.5mm",1.0925,0.1855),
      ("MOVE Kx1.10561, H=185.5mm",1.10561,0.1855),
    ]
    print("="*126)
    print("R10 — RAW R1 KLT + GROUND-PLANE METRIC MOTION, INDEPENDENT OF KIMERA")
    print("="*126)
    print(f"run={root} truth={args.truth_mm:.1f}mm movement_duration={(t1-t0)/1e9:.3f}s")
    print(f"stored K fx/fy={fx:.3f}/{fy:.3f}; selected period={args.period_ms:.1f}ms")
    with (root/"frames.mjpg").open("rb") as mj:
        for name,k,h in cases:
            K=np.array([[fx*k,0,cx],[0,fy*k,cy],[0,0,1]],float)
            z=run_case(frames,mj,att,rng,K,D,RBC,t0,t1,int(args.period_ms*1e6),h)
            frac=z["valid"]/max(1,z["valid"]+z["fail"])
            if z["stats"]:
                med_in=statistics.median(s[0] for s in z["stats"])
                med_res=statistics.median(s[2] for s in z["stats"])
            else: med_in=0; med_res=float("nan")
            print(f"{name:32s}: net={z['net']*1000:8.2f}mm scale={z['net']/truth:7.4f} "
                  f"x={z['x']*1000:+8.2f} y={z['y']*1000:+8.2f} path={z['path']*1000:8.2f} "
                  f"pairs={z['valid']}/{z['valid']+z['fail']} ({frac*100:5.1f}%) inliers_med={med_in:.0f} geom_res_med={med_res*1000:.2f}mm")
    print("\nINTERPRETATION")
    print("-"*126)
    print("This estimator uses only raw R1 images + FC attitude + chosen camera-plane height; no Kimera state and no endpoint scale correction.")
    print("If H~170mm with stored K gives ~500mm while H=185.5mm gives ~545mm, the dominant ambiguity is camera-plane height reference, not focal.")
    print("If all plausible height/K branches remain near MOVE500 ~553mm, investigate tracking/attitude/extrinsics rather than camera intrinsics.")
    print("="*126)

if __name__=="__main__": main()
