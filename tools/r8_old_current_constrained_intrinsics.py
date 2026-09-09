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

def collect(root):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),26.47/1000.0,19.411/1000.0,dictionary)
    det=cv2.aruco.CharucoDetector(board)
    all_obj=np.asarray(board.getChessboardCorners(),dtype=np.float32)
    out=[]
    for p in sorted(root.glob("frame_*.png")):
        g=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
        if g is None or g.shape!=(480,640): continue
        cc,ci,_,_=det.detectBoard(g)
        if ci is None or cc is None or len(ci)<12: continue
        ids=np.asarray(ci,dtype=np.int32).reshape(-1)
        pts=np.asarray(cc,dtype=np.float32).reshape(-1,2)
        out.append((all_obj[ids].reshape(-1,3),pts))
    return out

def fit(rows):
    obj=[x[0] for x in rows]; img=[x[1] for x in rows]
    K=np.array([[OLD_FX,0,OLD_CX],[0,OLD_FY,OLD_CY],[0,0,1]],dtype=np.float64)
    D=OLD_D.reshape(-1,1).copy()
    flags=(cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_PRINCIPAL_POINT|
           cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3|cv2.CALIB_FIX_TANGENT_DIST)
    crit=(cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS,200,1e-13)
    rms,K,D,rv,tv=cv2.calibrateCamera(obj,img,(640,480),K,D,flags=flags,criteria=crit)
    return float(rms),float(K[0,0]),float(K[1,1])

def report(name,rows):
    print(f"\n{name} n={len(rows)}")
    print("-"*100)
    tests=[("ALL",rows),("EVEN",rows[::2]),("ODD",rows[1::2])]
    for k in range(3):
        tests.append((f"MOD3={k}",rows[k::3]))
    vals=[]
    for label,sub in tests:
        if len(sub)<10: continue
        rms,fx,fy=fit(sub); vals.append((fx,fy))
        print(f"{label:8s} n={len(sub):2d} rms={rms:.6f}px fx={fx:.3f} fy={fy:.3f}")
    fxs=[x[0] for x in vals]; fys=[x[1] for x in vals]
    print(f"fx range=[{min(fxs):.3f},{max(fxs):.3f}] span={max(fxs)-min(fxs):.3f}px")
    print(f"fy range=[{min(fys):.3f},{max(fys):.3f}] span={max(fys)-min(fys):.3f}px")
    return fit(rows)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--old",default="/home/vio/charuco_calibration")
    ap.add_argument("--current",default="/home/vio/r6_charuco_current")
    a=ap.parse_args()
    old=collect(Path(a.old)); cur=collect(Path(a.current))
    print("="*100)
    print("R8 — OLD VS CURRENT OV9281, SAME CONSTRAINED INTRINSICS MODEL")
    print("="*100)
    ro=report("OLD DATASET",old)
    rc=report("CURRENT R6 DATASET",cur)
    print("\nCROSS-DATASET")
    print("-"*100)
    print(f"OLD     rms={ro[0]:.6f}px fx={ro[1]:.3f} fy={ro[2]:.3f}")
    print(f"CURRENT rms={rc[0]:.6f}px fx={rc[1]:.3f} fy={rc[2]:.3f}")
    print(f"fx ratio current/old = {rc[1]/ro[1]:.6f} ({(rc[1]/ro[1]-1)*100:+.3f}%)")
    print(f"fy ratio current/old = {rc[2]/ro[2]:.6f} ({(rc[2]/ro[2]-1)*100:+.3f}%)")
    print("R1 claimed multiplier = +9.250%")
    print("MOVE500 equivalent    = +10.561%")
    if rc[1] > ro[1]*1.07 and rc[2] > ro[2]*1.07:
        print("VERDICT: CURRENT CAMERA SUPPORTS LARGE POSITIVE FOCAL CHANGE.")
    elif rc[1] < ro[1] and rc[2] < ro[2]:
        print("VERDICT: CURRENT CAMERA DOES NOT SUPPORT R1/MOVE500 POSITIVE FOCAL HYPOTHESIS; fitted change is opposite-sign.")
    else:
        print("VERDICT: NO LARGE POSITIVE FOCAL CHANGE.")
    print("="*100)

if __name__=="__main__":
    main()
