#!/usr/bin/env python3
import argparse
from pathlib import Path
import cv2
import numpy as np

OLD_FX=568.53170752165227
OLD_FY=569.68005562865858
OLD_CX=315.98271077441063
OLD_CY=239.88148589100641
OLD_D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)

R1_K=1.0925
MOVE_K=1.105610


def collect(root):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),26.47/1000.0,19.411/1000.0,dictionary)
    detector=cv2.aruco.CharucoDetector(board)
    all_obj=np.asarray(board.getChessboardCorners(),dtype=np.float32)
    obj=[]; img=[]
    for p in sorted(root.glob("frame_*.png")):
        g=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
        if g is None or g.shape!=(480,640): continue
        cc,ci,_,_=detector.detectBoard(g)
        if ci is None or cc is None or len(ci)<12: continue
        ids=np.asarray(ci,dtype=np.int32).reshape(-1)
        pts=np.asarray(cc,dtype=np.float32).reshape(-1,2)
        obj.append(all_obj[ids].reshape(-1,3))
        img.append(pts)
    return obj,img


def rms_fixed(obj,img,K,D):
    vals=[]
    for op,ip in zip(obj,img):
        ok,r,t=cv2.solvePnP(op,ip,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: continue
        pr,_=cv2.projectPoints(op,r,t,K,D)
        e=pr.reshape(-1,2)-ip.reshape(-1,2)
        vals.append(float(np.sqrt(np.mean(np.sum(e*e,axis=1)))))
    return float(np.sqrt(np.mean(np.square(vals)))), float(np.median(vals))


def calibrate(obj,img,K,D,flags):
    crit=(cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS,200,1e-13)
    rms,K,D,rv,tv=cv2.calibrateCamera(obj,img,(640,480),K,D,flags=flags,criteria=crit)
    return float(rms),K,D


def show(name,rms,K,D):
    print(f"{name:28s} rms={rms:.6f}px fx={K[0,0]:.3f} fy={K[1,1]:.3f} cx={K[0,2]:.3f} cy={K[1,2]:.3f} "
          f"D=[{D[0,0]:+.4f},{D[1,0]:+.4f},{D[2,0]:+.4f},{D[3,0]:+.4f},{D[4,0]:+.4f}]")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--input",required=True)
    a=ap.parse_args()
    obj,img=collect(Path(a.input))
    print("="*132)
    print("R7 — CURRENT DATASET CONSTRAINED-INTRINSICS SENSITIVITY")
    print("="*132)
    print(f"usable_views={len(obj)}")

    K0=np.array([[OLD_FX,0,OLD_CX],[0,OLD_FY,OLD_CY],[0,0,1]],dtype=np.float64)

    # A: fully free
    rms,K,D=calibrate(obj,img,np.eye(3,dtype=np.float64),np.zeros((5,1),dtype=np.float64),0)
    show("A FREE ALL",rms,K,D)

    # B: principal point fixed to old, D free
    K=K0.copy(); D=np.zeros((5,1),dtype=np.float64)
    flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_PRINCIPAL_POINT
    rms,K,D=calibrate(obj,img,K,D,flags)
    show("B FIX OLD CX/CY",rms,K,D)

    # C: old D fixed, principal point free
    K=K0.copy(); D=OLD_D.reshape(-1,1).copy()
    flags=(cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2|
           cv2.CALIB_FIX_K3|cv2.CALIB_FIX_TANGENT_DIST)
    rms,K,D=calibrate(obj,img,K,D,flags)
    show("C FIX OLD D",rms,K,D)

    # D: old D and principal point fixed; only fx/fy free
    K=K0.copy(); D=OLD_D.reshape(-1,1).copy()
    flags=(cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_PRINCIPAL_POINT|
           cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3|cv2.CALIB_FIX_TANGENT_DIST)
    rms,K,D=calibrate(obj,img,K,D,flags)
    show("D FIX OLD D + CX/CY",rms,K,D)

    print("\nFIXED-K POSE-ONLY REPROJECTION")
    print("-"*132)
    candidates=[
        ("OLD K x1.0000",1.0),
        ("R6 FREE approx x0.9473",538.563490423/OLD_FX),
        ("R1 HEIGHT x1.0925",R1_K),
        ("MOVE500 x1.105610",MOVE_K),
    ]
    for name,k in candidates:
        K=np.array([[OLD_FX*k,0,OLD_CX],[0,OLD_FY*k,OLD_CY],[0,0,1]],dtype=np.float64)
        ar,med=rms_fixed(obj,img,K,OLD_D.reshape(-1,1))
        print(f"{name:28s} aggregate={ar:.6f}px median={med:.6f}px fx={K[0,0]:.3f} fy={K[1,1]:.3f}")

    print("\nINTERPRETATION")
    print("-"*132)
    print("If constrained fits B/C/D disagree strongly with A, the current dataset does not uniquely identify focal vs principal point/distortion.")
    print("If D is stable and near one candidate, that candidate is much more defensible because old cx/cy and D are held fixed.")
    print("="*132)


if __name__=="__main__":
    main()
