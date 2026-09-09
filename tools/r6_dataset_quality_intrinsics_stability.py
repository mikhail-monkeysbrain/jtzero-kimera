#!/usr/bin/env python3
import argparse
import statistics
from pathlib import Path

import cv2
import numpy as np


def detect(root, square_mm=26.47, marker_mm=19.411):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((7,5), square_mm/1000.0, marker_mm/1000.0, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    obj_all = np.asarray(board.getChessboardCorners(), dtype=np.float32)

    rows=[]
    for p in sorted(root.glob("frame_*.png")):
        im=cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if im is None or im.shape!=(480,640):
            continue
        cc,ci,_,_=detector.detectBoard(im)
        if ci is None or cc is None or len(ci)<8:
            continue
        ids=np.asarray(ci,dtype=np.int32).reshape(-1)
        pts=np.asarray(cc,dtype=np.float32).reshape(-1,2)
        obj=obj_all[ids].reshape(-1,3)

        xmin,ymin=np.min(pts,axis=0)
        xmax,ymax=np.max(pts,axis=0)
        bbox_w=float(xmax-xmin); bbox_h=float(ymax-ymin)
        center=np.mean(pts,axis=0)

        # image-space scale proxy robust to partial boards
        scales=[]
        for i in range(len(ids)):
            for j in range(i+1,len(ids)):
                od=float(np.linalg.norm(obj[i,:2]-obj[j,:2]))
                if od>1e-9:
                    scales.append(float(np.linalg.norm(pts[i]-pts[j]))/od)
        scale=statistics.median(scales) if scales else 0.0

        # planar homography condition / perspective proxy
        H,_=cv2.findHomography(obj[:,:2],pts,0)
        tilt_proxy=float("nan")
        if H is not None:
            h1=H[:,0]; h2=H[:,1]
            n1=np.linalg.norm(h1); n2=np.linalg.norm(h2)
            if n1>0 and n2>0:
                tilt_proxy=abs(float(np.dot(h1,h2)/(n1*n2)))

        rows.append(dict(path=p,obj=obj,img=pts,corners=len(ids),
                         cx=float(center[0]),cy=float(center[1]),
                         bbox_w=bbox_w,bbox_h=bbox_h,scale=scale,
                         tilt_proxy=tilt_proxy))
    return rows


def calibrate(rows):
    obj=[r["obj"] for r in rows]
    img=[r["img"] for r in rows]
    K=np.eye(3,dtype=np.float64)
    K[0,2]=319.5; K[1,2]=239.5
    D=np.zeros((5,1),dtype=np.float64)
    crit=(cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS,100,1e-12)
    rms,K,D,rv,tv=cv2.calibrateCamera(obj,img,(640,480),K,D,flags=0,criteria=crit)
    return float(rms),K,D


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",required=True)
    a=ap.parse_args()
    root=Path(a.input)
    rows=detect(root)
    print("="*110)
    print("R6Q — CURRENT CHARUCO DATASET QUALITY / INTRINSICS STABILITY")
    print("="*110)
    print(f"input={root}")
    print(f"usable detections={len(rows)}")
    if len(rows)<10:
        raise SystemExit("ERROR: insufficient detections")

    cs=[r["corners"] for r in rows]
    cxs=[r["cx"] for r in rows]; cys=[r["cy"] for r in rows]
    bws=[r["bbox_w"] for r in rows]; bhs=[r["bbox_h"] for r in rows]
    ss=[r["scale"] for r in rows]
    print("\nDATASET GEOMETRY")
    print("-"*110)
    print(f"corners min/med/max = {min(cs)}/{statistics.median(cs):.1f}/{max(cs)}")
    print(f"frames corners>=18 = {sum(x>=18 for x in cs)}/{len(cs)}")
    print(f"frames corners>=20 = {sum(x>=20 for x in cs)}/{len(cs)}")
    print(f"center_x span = [{min(cxs):.1f},{max(cxs):.1f}] px")
    print(f"center_y span = [{min(cys):.1f},{max(cys):.1f}] px")
    print(f"bbox_w span   = [{min(bws):.1f},{max(bws):.1f}] px")
    print(f"bbox_h span   = [{min(bhs):.1f},{max(bhs):.1f}] px")
    print(f"scale span    = [{min(ss):.1f},{max(ss):.1f}] px/m")

    # 4x3 coverage by detected corner count
    grid=np.zeros((3,4),dtype=int)
    for r in rows:
        for x,y in r["img"]:
            ix=min(3,max(0,int(x*4/640)))
            iy=min(2,max(0,int(y*3/480)))
            grid[iy,ix]+=1
    print("\nCOVERAGE GRID (detected ChArUco corners)")
    for rr in grid:
        print(" ".join(f"{int(v):5d}" for v in rr))

    tests=[]
    tests.append(("ALL", rows))
    high=[r for r in rows if r["corners"]>=18]
    if len(high)>=10: tests.append(("CORNERS>=18",high))
    high20=[r for r in rows if r["corners"]>=20]
    if len(high20)>=10: tests.append(("CORNERS>=20",high20))
    # every-other deterministic subsets
    even=rows[::2]; odd=rows[1::2]
    if len(even)>=10: tests.append(("EVEN",even))
    if len(odd)>=10: tests.append(("ODD",odd))
    # thirds
    for k in range(3):
        sub=rows[k::3]
        if len(sub)>=10: tests.append((f"MOD3={k}",sub))

    print("\nINTRINSICS STABILITY")
    print("-"*110)
    vals=[]
    for name,sub in tests:
        rms,K,D=calibrate(sub)
        vals.append((name,len(sub),rms,K,D))
        print(f"{name:12s} n={len(sub):2d} rms={rms:.4f}px "
              f"fx={K[0,0]:.3f} fy={K[1,1]:.3f} cx={K[0,2]:.3f} cy={K[1,2]:.3f} "
              f"k1={D[0,0]:+.4f} k2={D[1,0]:+.4f} p1={D[2,0]:+.4f} p2={D[3,0]:+.4f} k3={D[4,0]:+.4f}")

    fxs=[v[3][0,0] for v in vals]; fys=[v[3][1,1] for v in vals]
    print("\nCROSS-CHECK")
    print("-"*110)
    print(f"fx across tested subsets = [{min(fxs):.3f},{max(fxs):.3f}] span={max(fxs)-min(fxs):.3f}px")
    print(f"fy across tested subsets = [{min(fys):.3f},{max(fys):.3f}] span={max(fys)-min(fys):.3f}px")
    print("OLD validated K = fx 568.532 / fy 569.680")
    print("R1 height-match equivalent ≈ fx 621.121 / fy 622.376")
    print("MOVE500 equivalent          ≈ fx 628.573 / fy 629.845")
    if (max(fxs)-min(fxs))>15 or (max(fys)-min(fys))>15:
        print("VERDICT: UNSTABLE CURRENT CALIBRATION — do not use this R6 fit as evidence of optical focal change.")
    else:
        print("VERDICT: CURRENT CALIBRATION SUBSETS ARE INTERNALLY STABLE — compare stable K against old/R1 values.")
    print("="*110)


if __name__=="__main__":
    main()
