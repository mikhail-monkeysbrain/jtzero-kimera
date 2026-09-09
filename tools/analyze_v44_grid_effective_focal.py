#!/usr/bin/env python3
import argparse, glob, math, statistics
from pathlib import Path
try:
    import cv2
    import numpy as np
except ModuleNotFoundError as e:
    raise SystemExit(
        "ERROR: OpenCV Python module is missing in the active interpreter.\n"
        "Run with /usr/bin/python3 on this system."
    ) from e

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641
D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)
K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],dtype=np.float64)

def find_candidates(gray):
    out=[]
    for cols in range(4,11):
        for rows in range(3,9):
            ok,c=cv2.findChessboardCornersSB(
                gray,(cols,rows),
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY
            )
            if not ok or c is None:
                continue
            pts=c.reshape(rows,cols,2)
            # regularity score: adjacent spacing CV + row/col direction consistency
            hs=[]; vs=[]
            for r in range(rows):
                for cc in range(cols-1):
                    hs.append(np.linalg.norm(pts[r,cc+1]-pts[r,cc]))
            for r in range(rows-1):
                for cc in range(cols):
                    vs.append(np.linalg.norm(pts[r+1,cc]-pts[r,cc]))
            if not hs or not vs: continue
            mh,mv=float(np.mean(hs)),float(np.mean(vs))
            cvh=float(np.std(hs)/mh) if mh>1e-9 else 999
            cvv=float(np.std(vs)/mv) if mv>1e-9 else 999
            score=cvh+cvv
            out.append((score,cols,rows,pts))
    out.sort(key=lambda x:(x[0],-(x[1]*x[2])))
    return out

def analyze_grid(pts,cols,rows,square_mm,physical_h):
    # Adjacent spans in pixels, robust medians.
    hsp=[float(np.linalg.norm(pts[r,c+1]-pts[r,c])) for r in range(rows) for c in range(cols-1)]
    vsp=[float(np.linalg.norm(pts[r+1,c]-pts[r,c])) for r in range(rows-1) for c in range(cols)]
    med_h=statistics.median(hsp); med_v=statistics.median(vsp)

    # First-order local effective focal estimates from physical square scale.
    fx_eff=med_h*physical_h/square_mm
    fy_eff=med_v*physical_h/square_mm

    # Undistort corner coordinates and recompute local spacing in normalized-pixel coordinates.
    flat=pts.reshape(-1,1,2).astype(np.float64)
    und=cv2.undistortPoints(flat,K,D,P=K).reshape(rows,cols,2)
    uh=[float(np.linalg.norm(und[r,c+1]-und[r,c])) for r in range(rows) for c in range(cols-1)]
    uv=[float(np.linalg.norm(und[r+1,c]-und[r,c])) for r in range(rows-1) for c in range(cols)]
    med_uh=statistics.median(uh); med_uv=statistics.median(uv)
    fx_eff_u=med_uh*physical_h/square_mm
    fy_eff_u=med_uv*physical_h/square_mm

    # Homography regularity / projective tilt proxy.
    obj=np.array([[c*square_mm,r*square_mm] for r in range(rows) for c in range(cols)],dtype=np.float64)
    img=pts.reshape(-1,2).astype(np.float64)
    H,mask=cv2.findHomography(obj,img,0)
    if H is None:
        reproj=float("nan")
    else:
        homog=np.c_[obj,np.ones(len(obj))]
        q=(H@homog.T).T
        q=q[:,:2]/q[:,2:3]
        reproj=math.sqrt(float(np.mean(np.sum((q-img)**2,axis=1))))

    return dict(
        med_h=med_h,med_v=med_v,fx_eff=fx_eff,fy_eff=fy_eff,
        med_uh=med_uh,med_uv=med_uv,fx_eff_u=fx_eff_u,fy_eff_u=fy_eff_u,
        reproj=reproj
    )

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--square-mm",type=float,required=True)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    ap.add_argument("--max-regularity-cv",type=float,default=0.20)
    a=ap.parse_args()

    files=sorted(glob.glob(a.glob))
    if not files: raise SystemExit("no images matched")

    print("="*120)
    print("V44.4b — ROBUST GRID LOCAL-SCALE / EFFECTIVE-FOCAL CHECK")
    print("="*120)
    print(f"images={len(files)} square={a.square_mm:.3f}mm physical_h={a.physical_height_mm:.2f}mm")
    print(f"stored fx/fy={FX:.3f}/{FY:.3f}px")
    print("NOTE: prior V44.4 PnP output was invalid because checker autodetection produced false grids with huge reprojection errors.")
    print()

    good=[]
    for fn in files:
        im=cv2.imread(fn,cv2.IMREAD_GRAYSCALE)
        if im is None:
            print(f"{Path(fn).name}: READ FAIL"); continue
        cand=find_candidates(im)
        if not cand:
            print(f"{Path(fn).name}: NO GRID"); continue
        score,cols,rows,pts=cand[0]
        if score>a.max_regularity_cv:
            print(f"{Path(fn).name}: REJECT irregular candidate {cols}x{rows} regularity={score:.3f}")
            continue
        m=analyze_grid(pts,cols,rows,a.square_mm,a.physical_height_mm)
        good.append((cols,rows,score,m))
        print(f"{Path(fn).name}: {cols}x{rows} reg={score:.3f} "
              f"raw_spacing={m['med_h']:.2f}/{m['med_v']:.2f}px "
              f"undist={m['med_uh']:.2f}/{m['med_uv']:.2f}px "
              f"fx/fy_eff_u={m['fx_eff_u']:.1f}/{m['fy_eff_u']:.1f}px "
              f"H_rms={m['reproj']:.2f}px")

    print()
    print("SUMMARY")
    print("-"*120)
    if not good:
        print("usable=0")
        print("VERDICT: current ArUco/checker artwork is not reliably detectable as one regular checkerboard.")
        print("NEXT: use ArUco/ChArUco marker geometry directly or provide explicit pixel endpoints for the measured 80.77mm span.")
        print("="*120)
        return

    fxe=[x[3]["fx_eff_u"] for x in good]; fye=[x[3]["fy_eff_u"] for x in good]
    kmx=statistics.median(fxe)/FX; kmy=statistics.median(fye)/FY
    print(f"usable={len(good)}/{len(files)}")
    print(f"median effective fx/fy={statistics.median(fxe):.2f}/{statistics.median(fye):.2f}px")
    print(f"median focal multipliers kx/ky={kmx:.5f}/{kmy:.5f}")
    print("V44 motion-scale residual requires isotropic focal_k ~= 1.068 at h=185mm after rotation correction.")
    kval=(kmx+kmy)/2
    if abs(kval-1.068)<=0.02:
        print("VERDICT: SUPPORTS effective focal/runtime projection-scale mismatch.")
    elif abs(kval-1.0)<=0.02:
        print("VERDICT: SUPPORTS stored focal scale; remaining motion error lies elsewhere.")
    else:
        print("VERDICT: DOES NOT CLEANLY MATCH either hypothesis; inspect target geometry/perspective/anisotropy.")
    print("="*120)

if __name__=="__main__":
    main()
