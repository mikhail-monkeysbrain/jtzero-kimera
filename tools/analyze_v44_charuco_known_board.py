#!/usr/bin/env python3
import argparse, glob, math, statistics
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
    # OpenCV 4.x constructor compatibility
    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            return cv2.aruco.CharucoBoard((7,5), square_mm, marker_mm, DICT)
        except TypeError:
            pass
    return cv2.aruco.CharucoBoard_create(7,5,square_mm,marker_mm,DICT)

def detect_charuco(gray, board):
    if hasattr(cv2.aruco,"ArucoDetector"):
        det=cv2.aruco.ArucoDetector(DICT,cv2.aruco.DetectorParameters())
        mcorners,mids,_=det.detectMarkers(gray)
    else:
        mcorners,mids,_=cv2.aruco.detectMarkers(gray,DICT)
    if mids is None or len(mids)<4:
        return None
    n,ccorners,cids=cv2.aruco.interpolateCornersCharuco(mcorners,mids,gray,board)
    if cids is None or ccorners is None or int(n)<6:
        return None
    return ccorners.reshape(-1,2).astype(np.float64), cids.flatten().astype(int), len(mids)

def board_corners(board):
    if hasattr(board,"getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(),dtype=np.float64)
    return np.asarray(board.chessboardCorners,dtype=np.float64)

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
    normal=R[:,2]
    tilt=math.degrees(math.acos(max(-1.0,min(1.0,abs(float(normal[2]))))))
    return rms,h,tilt,rvec,tvec

def main():
    ap=argparse.ArgumentParser(description="V44.8 robust ChArUco pose/focal sweep using known 7x5 board layout")
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--square-mm",type=float,default=26.47,
                    help="ACTUAL printed ChArUco square size")
    ap.add_argument("--marker-mm",type=float,default=None,
                    help="ACTUAL printed marker outer black-square side; default derives from PDF ratio 22/30")
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    if a.marker_mm is None:
        a.marker_mm=a.square_mm*(22.0/30.0)

    board=make_board(a.square_mm,a.marker_mm)
    bc=board_corners(board)
    files=sorted(glob.glob(a.glob))
    if not files: raise SystemExit("no images matched")

    per_id={}
    frame_stats=[]
    for fn in files:
        gray=cv2.imread(fn,cv2.IMREAD_GRAYSCALE)
        if gray is None: continue
        d=detect_charuco(gray,board)
        if d is None:
            frame_stats.append((Path(fn).name,0,0))
            continue
        corners,ids,nmarkers=d
        frame_stats.append((Path(fn).name,len(ids),nmarkers))
        for p,i in zip(corners,ids):
            per_id.setdefault(int(i),[]).append(p)

    stable={i:np.median(np.stack(v),axis=0) for i,v in per_id.items() if len(v)>=max(3,len(files)//2)}
    if len(stable)<8:
        raise SystemExit(f"too few stable ChArUco corners: {len(stable)}")

    ids=np.array(sorted(stable),dtype=int)
    img=np.array([stable[int(i)] for i in ids],dtype=np.float64)
    obj=bc[ids].astype(np.float64)

    print("="*120)
    print("V44.8 — CHARUCO 7x5 KNOWN-LAYOUT POSE / FOCAL SWEEP")
    print("="*120)
    print(f"frames={len(files)} stable_charuco_corners={len(ids)}")
    print(f"actual square={a.square_mm:.3f}mm marker={a.marker_mm:.3f}mm physical sensor-plane height={a.physical_height_mm:.2f}mm")
    print(f"board source: 7x5, nominal 30/22 mm, DICT_4X4_50; marker derived ratio={a.marker_mm/a.square_mm:.6f}")
    print(f"stored fx/fy={FX:.3f}/{FY:.3f}px")
    print()
    print("FRAME DETECTION SUMMARY")
    print("-"*120)
    for name,nc,nm in frame_stats:
        print(f"{name}: charuco={nc:2d} markers={nm:2d}")
    print()
    print("stable IDs:", ",".join(map(str,ids.tolist())))

    ks=np.linspace(0.75,1.15,161)
    ds=np.linspace(0.0,1.5,31)
    rows=[]
    for k in ks:
        for dscale in ds:
            r=solve(obj,img,float(k),float(dscale))
            if r is None: continue
            rms,h,tilt,rv,tv=r
            rows.append((rms,abs(h-a.physical_height_mm),float(k),float(dscale),h,tilt))

    print()
    print("BEST REPROJECTION")
    print("-"*120)
    for rms,dh,k,dscale,h,tilt in sorted(rows)[:12]:
        print(f"rms={rms:6.3f}px k={k:7.4f} dist_scale={dscale:5.2f} h={h:7.2f}mm "
              f"h_err={(h/a.physical_height_mm-1)*100:+6.2f}% tilt={tilt:5.2f}deg")

    print()
    print("BEST PHYSICAL-HEIGHT MATCH")
    print("-"*120)
    for rms,dh,k,dscale,h,tilt in sorted(rows,key=lambda x:(x[1],x[0]))[:12]:
        print(f"h={h:7.2f}mm k={k:7.4f} dist_scale={dscale:5.2f} rms={rms:6.3f}px "
              f"h_err={(h/a.physical_height_mm-1)*100:+6.2f}% tilt={tilt:5.2f}deg")

    print()
    print("FIXED HYPOTHESES")
    print("-"*120)
    for label,target_k in [("stored K",1.0),("motion k",1.068),("local naive",0.78661)]:
        cand=[x for x in rows if abs(x[2]-target_k)<=0.0013]
        if not cand:
            print(f"{label:14s}: no candidate")
            continue
        cand.sort(key=lambda x:(x[0],x[1]))
        rms,dh,k,dscale,h,tilt=cand[0]
        print(f"{label:14s}: k={k:7.4f} dist_scale={dscale:5.2f} rms={rms:6.3f}px "
              f"h={h:7.2f}mm h_err={(h/a.physical_height_mm-1)*100:+6.2f}% tilt={tilt:5.2f}deg")

    # Joint score: normalize 1 px reprojection and 1% height error roughly equally.
    scored=[]
    for rms,dh,k,dscale,h,tilt in rows:
        hpct=100.0*dh/a.physical_height_mm
        score=rms + hpct
        scored.append((score,rms,hpct,k,dscale,h,tilt))
    print()
    print("JOINT LOW-RMS + PHYSICAL-HEIGHT")
    print("-"*120)
    for score,rms,hpct,k,dscale,h,tilt in sorted(scored)[:10]:
        print(f"score={score:6.3f} rms={rms:6.3f}px h_err={hpct:5.2f}% "
              f"k={k:7.4f} dist_scale={dscale:5.2f} h={h:7.2f}mm tilt={tilt:5.2f}deg")

    print()
    print("DECISION")
    print("-"*120)
    print("- This test uses the actual 7x5 ChArUco layout and actual print scale, so board-ID lattice inference is eliminated.")
    print("- If k~1.068 gives both low reprojection and h~185.5mm, the effective-focal explanation is directly supported.")
    print("- If k~1.0 wins, stored focal scale is supported and the motion-scale residual must be sought elsewhere.")
    print("- If neither can satisfy both geometry and height, do not tune intrinsics; inspect height reference, plane flatness, or motion model.")
    print("="*120)

if __name__=="__main__":
    main()
