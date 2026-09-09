#!/usr/bin/env python3
import argparse,csv,math,re,statistics
from pathlib import Path
import cv2
import numpy as np

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
    return rpy(interp(rows,t,"roll"),-interp(rows,t,"pitch"),-interp(rows,t,"yaw"))

def decode(mj,row):
    mj.seek(int(row["offset"])); b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): return None
    return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

def undistort_pixels(pts,K,D):
    p=np.asarray(pts,np.float64).reshape(-1,1,2)
    q=cv2.undistortPoints(p,K,D).reshape(-1,2)
    return np.c_[q,np.ones(len(q))]

def pair_geometry(im0,im1,K,D,RWC0,RWC1):
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
    # unit-height ground intersection; metric displacement scales linearly with H
    lam0=-1.0/rw0[:,2]; lam1=-1.0/rw1[:,2]
    ok &= (lam0>0)&(lam1>0)
    if np.count_nonzero(ok)<12: return None
    G0=rw0[ok]*lam0[ok,None]; G1=rw1[ok]*lam1[ok,None]
    d=G0[:,:2]-G1[:,:2]
    med=np.median(d,axis=0)
    resid=np.linalg.norm(d-med,axis=1)
    keep=resid<=np.percentile(resid,70)
    if np.count_nonzero(keep)>=8: med=np.median(d[keep],axis=0)
    return med,len(a),float(np.median(resid))

def build_unit_chain(root,camera_yaml,period_ms):
    frames=read_csv(root/"frames.csv"); ev=read_csv(root/"events.csv"); att=read_csv(root/"attitude.csv")
    starts=[int(r["recv_mono_ns"]) for r in ev if r["event"]=="MOVE_START"]
    ends=[int(r["recv_mono_ns"]) for r in ev if r["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1: raise RuntimeError("need one MOVE_START/MOVE_END")
    t0,t1=starts[0],ends[0]
    fx,fy,cx,cy,D,RBC=read_camera(camera_yaml)
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)

    sel=[]; last=-10**30; period_ns=int(period_ms*1e6)
    for r in frames:
        t=int(r["recv_mono_ns"])
        if t<t0 or t>t1: continue
        if not sel or t-last>=period_ns:
            sel.append(r); last=t

    steps=[]; failed=0
    with (root/"frames.mjpg").open("rb") as mj:
        prev=None; prevrow=None
        for row in sel:
            im=decode(mj,row)
            if im is None: continue
            if prev is None:
                prev,prevrow=im,row; continue
            ta=int(prevrow["recv_mono_ns"]); tb=int(row["recv_mono_ns"])
            z=pair_geometry(prev,im,K,D,attitude_R(att,ta)@RBC,attitude_R(att,tb)@RBC)
            if z is None: failed+=1
            else: steps.append(z)
            prev,prevrow=im,row
    return steps,failed,len(sel),fx,fy

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--h-min-mm",type=float,default=168.0)
    ap.add_argument("--h-max-mm",type=float,default=180.0)
    ap.add_argument("--h-step-mm",type=float,default=0.5)
    ap.add_argument("--tolerance-mm",type=float,default=10.0)
    ap.add_argument("--period-ms",type=float,default=35.0)
    a=ap.parse_args()

    root=Path(a.run)
    cam=Path(a.camera_yaml)
    steps,failed,nsel,fx,fy=build_unit_chain(root,cam,a.period_ms)
    if not steps: raise SystemExit("ERROR: no valid motion steps")
    ux=sum(s[0][0] for s in steps); uy=sum(s[0][1] for s in steps)
    unit_net=math.hypot(ux,uy)
    unit_path=sum(float(np.linalg.norm(s[0])) for s in steps)

    truth=a.truth_mm
    h_exact=truth/unit_net
    print("="*118)
    print("R12 — HEIGHT SENSITIVITY BAND FOR RAW R1 METRIC MOTION")
    print("="*118)
    print(f"run={root}")
    print(f"stored K fx/fy={fx:.3f}/{fy:.3f}")
    print(f"valid pairs={len(steps)}/{len(steps)+failed}; selected_frames={nsel}")
    print(f"unit-height net={unit_net:.6f} mm/mm; exact H for {truth:.1f}mm truth = {h_exact:.3f} mm")
    print(f"acceptance tolerance = ±{a.tolerance_mm:.1f} mm")

    accepted=[]
    h=a.h_min_mm
    print("\nSWEEP")
    print("-"*118)
    while h<=a.h_max_mm+1e-9:
        net=unit_net*h
        err=net-truth
        ok=abs(err)<=a.tolerance_mm
        if ok: accepted.append((h,net,err))
        print(f"H={h:7.2f} mm -> net={net:8.2f} mm err={err:+7.2f} mm scale={net/truth:7.4f} {'PASS' if ok else ''}")
        h+=a.h_step_mm

    print("\nACCEPTED HEIGHT BAND")
    print("-"*118)
    if accepted:
        print(f"H range = [{accepted[0][0]:.2f}, {accepted[-1][0]:.2f}] mm")
        print(f"net range = [{accepted[0][1]:.2f}, {accepted[-1][1]:.2f}] mm")
    else:
        print("none in requested sweep")

    print("\nREFERENCE HEIGHTS")
    print("-"*118)
    for name,h in [("R9 ChArUco PnP",170.2),("R10 exact fit",h_exact),("manual entered",185.5),("TF-Luna nominal",190.0)]:
        net=unit_net*h
        print(f"{name:20s} H={h:7.2f} mm -> net={net:8.2f} mm err={net-truth:+7.2f} mm")

    print("\nDECISION")
    print("-"*118)
    print("Use the accepted H interval as a sensitivity band, not as a calibrated constant.")
    print("Do not alter production focal length based on the earlier R1 height-match.")
    print("="*118)

if __name__=="__main__": main()
