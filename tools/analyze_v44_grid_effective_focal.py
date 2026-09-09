#!/usr/bin/env python3
import argparse, glob, math, statistics
from pathlib import Path

import cv2
import numpy as np

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641
D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)
K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],dtype=np.float64)

def detect_best(gray):
    candidates=[]
    # Sweep plausible full-board checkerboard inner-corner dimensions.
    for cols in range(4,11):
        for rows in range(3,9):
            ok,corners=cv2.findChessboardCornersSB(
                gray,(cols,rows),
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY
            )
            if ok and corners is not None:
                candidates.append((cols*rows,cols,rows,corners.reshape(-1,2)))
    if not candidates:
        return None
    # Largest detected grid is preferred; ties prefer wider pattern.
    candidates.sort(key=lambda x:(x[0],x[1],x[2]),reverse=True)
    return candidates[0]

def plane_distance_and_pose(corners, cols, rows, square_mm):
    obj=np.zeros((cols*rows,3),np.float64)
    pts=[]
    for r in range(rows):
        for c in range(cols):
            pts.append((c*square_mm,r*square_mm,0.0))
    obj[:]=np.asarray(pts,np.float64)

    ok,rvec,tvec=cv2.solvePnP(obj,corners.astype(np.float64),K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R,_=cv2.Rodrigues(rvec)
    C=-R.T@tvec.reshape(3,1)
    plane_dist=abs(float(C[2,0]))
    optical_z=float(tvec[2,0])

    proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
    proj=proj.reshape(-1,2)
    rms=math.sqrt(float(np.mean(np.sum((proj-corners)**2,axis=1))))

    # Board normal in camera coordinates and tilt from optical axis.
    normal=R[:,2]
    cosang=max(-1.0,min(1.0,abs(float(normal[2]))))
    tilt=math.degrees(math.acos(cosang))
    return plane_dist,optical_z,rms,tilt

def main():
    ap=argparse.ArgumentParser(description="V44.4 independent effective-focal check from checker/ArUco grid")
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--square-mm",type=float,default=40.385,
                    help="physical size of ONE large checker square; default=80.77/2 mm")
    ap.add_argument("--physical-height-mm",type=float,default=185.5,
                    help="working plane to OV9281 sensor plane")
    args=ap.parse_args()

    files=sorted(glob.glob(args.glob))
    if not files:
        raise SystemExit(f"no images matched: {args.glob}")

    print("="*118)
    print("V44.4 — INDEPENDENT GRID GEOMETRY / EFFECTIVE FOCAL CHECK")
    print("="*118)
    print(f"images={len(files)} square={args.square_mm:.3f} mm physical sensor-plane height={args.physical_height_mm:.2f} mm")
    print(f"stored calibration fx/fy={FX:.3f}/{FY:.3f} px")
    print()

    results=[]
    patterns={}
    for fn in files:
        im=cv2.imread(fn,cv2.IMREAD_GRAYSCALE)
        if im is None:
            print(f"{Path(fn).name}: READ FAIL")
            continue
        best=detect_best(im)
        if best is None:
            print(f"{Path(fn).name}: GRID NOT FOUND")
            continue
        _,cols,rows,corners=best
        pose=plane_distance_and_pose(corners,cols,rows,args.square_mm)
        if pose is None:
            print(f"{Path(fn).name}: PNP FAIL pattern={cols}x{rows}")
            continue
        h,z,rms,tilt=pose
        k=args.physical_height_mm/h
        fx_eff=FX*k; fy_eff=FY*k
        results.append((h,z,rms,tilt,k,fx_eff,fy_eff))
        patterns[(cols,rows)]=patterns.get((cols,rows),0)+1
        print(f"{Path(fn).name}: pattern={cols}x{rows} h_plane={h:7.2f}mm z_opt={z:7.2f}mm "
              f"tilt={tilt:5.2f}deg reproj={rms:5.3f}px focal_k={k:7.4f} fx_eff={fx_eff:7.2f}")

    if not results:
        raise SystemExit("no usable frames")

    H=[x[0] for x in results]; RMS=[x[2] for x in results]; T=[x[3] for x in results]; KFAC=[x[4] for x in results]
    FXE=[x[5] for x in results]; FYE=[x[6] for x in results]
    print()
    print("SUMMARY")
    print("-"*118)
    print("patterns:",", ".join(f"{c}x{r}:{n}" for (c,r),n in sorted(patterns.items())))
    print(f"usable={len(results)}/{len(files)}")
    print(f"plane height from stored K: mean={statistics.mean(H):.2f} median={statistics.median(H):.2f} "
          f"min/max={min(H):.2f}/{max(H):.2f} mm")
    print(f"board tilt: mean={statistics.mean(T):.2f} deg  reprojection RMS mean={statistics.mean(RMS):.3f} px")
    print(f"required focal multiplier from physical h={args.physical_height_mm:.2f} mm: "
          f"mean={statistics.mean(KFAC):.5f} median={statistics.median(KFAC):.5f}")
    print(f"effective fx/fy: mean={statistics.mean(FXE):.2f}/{statistics.mean(FYE):.2f} px")
    print()
    print("RECONCILIATION")
    print("-"*118)
    km=statistics.median(KFAC)
    print("V44 motion-scale residual at h=185 mm after FC-rotation compensation required focal_k ~= 1.068.")
    print(f"Independent static grid result gives focal_k={km:.5f}.")
    if abs(km-1.068)<=0.015:
        print("VERDICT: SUPPORTS effective-focal / runtime projection-scale mismatch as the main residual source.")
    elif abs(km-1.0)<=0.015:
        print("VERDICT: SUPPORTS stored calibration scale; the remaining motion-scale error is NOT explained by focal scale.")
    else:
        print("VERDICT: INTERMEDIATE / ANISOTROPIC OR TARGET-GEOMETRY EFFECT; inspect square measurement, tilt and distortion.")
    print()
    print("NOTE: this result is valid only if --square-mm is the actual physical size of ONE large checker square.")
    print("="*118)

if __name__=="__main__":
    main()
