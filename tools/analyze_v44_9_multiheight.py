#!/usr/bin/env python3
import argparse, math
from pathlib import Path
try:
    import cv2
    import numpy as np
except ModuleNotFoundError as e:
    raise SystemExit("ERROR: run with /usr/bin/python3 (cv2 required)") from e

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641
D0=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def make_board(square_mm, marker_mm):
    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            return cv2.aruco.CharucoBoard((7,5), square_mm, marker_mm, DICT)
        except TypeError:
            pass
    return cv2.aruco.CharucoBoard_create(7,5,square_mm,marker_mm,DICT)

def detect(gray, board):
    if hasattr(cv2.aruco,"ArucoDetector"):
        det=cv2.aruco.ArucoDetector(DICT,cv2.aruco.DetectorParameters())
        mcorners,mids,_=det.detectMarkers(gray)
    else:
        mcorners,mids,_=cv2.aruco.detectMarkers(gray,DICT)
    if mids is None or len(mids)<4:
        return None
    n,cc,cids=cv2.aruco.interpolateCornersCharuco(mcorners,mids,gray,board)
    if cc is None or cids is None or int(n)<6:
        return None
    return cc.reshape(-1,2).astype(np.float64), cids.flatten().astype(int)

def board_corners(board):
    return np.asarray(board.getChessboardCorners() if hasattr(board,"getChessboardCorners") else board.chessboardCorners,dtype=np.float64)

def aggregate(images, board):
    per={}
    used=0
    for fn in images:
        g=cv2.imread(str(fn),cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        d=detect(g,board)
        if d is None:
            continue
        pts,ids=d
        used+=1
        for p,i in zip(pts,ids):
            per.setdefault(int(i),[]).append(p)
    stable={i:np.median(np.stack(v),axis=0) for i,v in per.items() if len(v)>=max(3,used//2)}
    return used,stable

def solve(obj,img,k,dscale):
    K=np.array([[FX*k,0,CX],[0,FY*k,CY],[0,0,1]],dtype=np.float64)
    D=D0*dscale
    ok,rvec,tvec=cv2.solvePnP(obj,img,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
    proj=proj.reshape(-1,2)
    rms=math.sqrt(float(np.mean(np.sum((proj-img)**2,axis=1))))
    R,_=cv2.Rodrigues(rvec)
    C=-R.T@tvec.reshape(3,1)
    h=abs(float(C[2,0]))
    return rms,h

def read_meta(d):
    meta={}
    for line in (d/"METADATA.txt").read_text().splitlines():
        if "=" in line:
            k,v=line.split("=",1)
            meta[k.strip()]=v.strip()
    return meta

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--root",default=str(Path.home()/"jtzero_runs"))
    ap.add_argument("--pattern",default="v44_9_*")
    ap.add_argument("--square-mm",type=float,default=26.47)
    ap.add_argument("--marker-mm",type=float,default=None)
    a=ap.parse_args()
    if a.marker_mm is None:
        a.marker_mm=a.square_mm*(22.0/30.0)

    board=make_board(a.square_mm,a.marker_mm)
    bc=board_corners(board)
    datasets=[]
    for d in sorted(Path(a.root).glob(a.pattern)):
        if not (d/"METADATA.txt").exists():
            continue
        m=read_meta(d)
        if "physical_height_mm" not in m:
            continue
        images=sorted(d.glob("frame_*.jpg"))
        used,stable=aggregate(images,board)
        if len(stable)<8:
            print(f"SKIP {d.name}: stable corners={len(stable)} used_frames={used}")
            continue
        ids=np.array(sorted(stable),dtype=int)
        img=np.array([stable[int(i)] for i in ids],dtype=np.float64)
        obj=bc[ids].astype(np.float64)
        datasets.append((d.name,float(m["physical_height_mm"]),used,len(stable),obj,img))

    if len(datasets)<2:
        raise SystemExit("Need at least 2 valid V44.9 static heights")

    print("="*120)
    print("V44.9 — MULTI-HEIGHT CHARUCO COMMON-FOCAL FIT")
    print("="*120)
    for name,h,used,nc,obj,img in datasets:
        print(f"{name}: physical_h={h:.2f}mm frames={used} stable_corners={nc}")

    ks=np.linspace(0.90,1.20,301)
    ds=np.linspace(0.0,1.0,21)
    rows=[]
    for k in ks:
        for dscale in ds:
            details=[]
            rms_sum=0.0
            herr_sum=0.0
            ok=True
            for name,htrue,used,nc,obj,img in datasets:
                r=solve(obj,img,float(k),float(dscale))
                if r is None:
                    ok=False
                    break
                rms,h=r
                hpct=100.0*(h/htrue-1.0)
                rms_sum+=rms
                herr_sum+=abs(hpct)
                details.append((name,htrue,h,hpct,rms))
            if ok:
                mean_rms=rms_sum/len(datasets)
                mean_herr=herr_sum/len(datasets)
                rows.append((mean_rms+mean_herr,mean_rms,mean_herr,float(k),float(dscale),details))
    rows.sort()

    print()
    print("BEST COMMON MODEL")
    print("-"*120)
    for score,rr,hh,k,dscale,det in rows[:10]:
        print(f"score={score:6.3f} mean_rms={rr:5.3f}px mean_abs_herr={hh:5.2f}% k={k:7.4f} dist_scale={dscale:4.2f}")
        print("  "+" | ".join(f"{n}: {hp:+5.2f}% ({h:.1f}/{ht:.1f}mm)" for n,ht,h,hp,r in det))

    print()
    print("FIXED K COMPARISON")
    print("-"*120)
    for label,target in [("stored",1.0),("motion",1.068),("V44.8",1.110)]:
        cand=[x for x in rows if abs(x[3]-target)<=0.0006]
        cand.sort()
        if not cand:
            print(f"{label}: no candidate")
            continue
        score,rr,hh,k,dscale,det=cand[0]
        print(f"{label:8s}: k={k:.4f} dist_scale={dscale:.2f} mean_rms={rr:.3f}px mean_abs_herr={hh:.2f}% score={score:.3f}")
        print("  "+" | ".join(f"{n}: {hp:+5.2f}%" for n,ht,h,hp,r in det))

    print()
    print("PER-HEIGHT IMPLIED K (distortion disabled)")
    print("-"*120)
    for name,htrue,used,nc,obj,img in datasets:
        cand=[]
        for k in ks:
            r=solve(obj,img,float(k),0.0)
            if r is None:
                continue
            rms,h=r
            cand.append((abs(h-htrue),float(k),rms,h))
        cand.sort()
        dh,k,rms,h=cand[0]
        print(f"{name}: k={k:.4f} h={h:.2f}mm target={htrue:.2f}mm rms={rms:.3f}px")

    print()
    print("DECISION")
    print("-"*120)
    print("- Constant implied k across heights supports a real focal-scale error.")
    print("- Systematic k-vs-height trend rejects a single focal correction.")
    print("- Do not update production intrinsics until common-k is stable across at least 3 heights.")
    print("="*120)

if __name__=="__main__":
    main()
