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
D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)
K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],dtype=np.float64)

def aruco_dicts():
    # The printed target's dictionary was not recorded. Sweep common dictionaries,
    # then demand cross-frame consistency instead of choosing a dictionary by desired focal.
    names=[
      "DICT_4X4_50","DICT_4X4_100","DICT_4X4_250","DICT_4X4_1000",
      "DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000",
      "DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000",
      "DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000",
      "DICT_ARUCO_ORIGINAL"
    ]
    return [(n,cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,n))) for n in names]

def detect(gray,dic):
    if hasattr(cv2.aruco,"ArucoDetector"):
        det=cv2.aruco.ArucoDetector(dic,cv2.aruco.DetectorParameters())
        corners,ids,_=det.detectMarkers(gray)
    else:
        corners,ids,_=cv2.aruco.detectMarkers(gray,dic)
    if ids is None: return []
    return [(int(i),c.reshape(4,2).astype(np.float64)) for i,c in zip(ids.flatten(),corners)]

def side_lengths(c):
    return [float(np.linalg.norm(c[(i+1)%4]-c[i])) for i in range(4)]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--marker-mm",type=float,required=True,
                    help="actual physical OUTER black-square side of one ArUco marker")
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    files=sorted(glob.glob(a.glob))
    if not files: raise SystemExit("no images matched")
    if not hasattr(cv2,"aruco"):
        raise SystemExit("cv2.aruco unavailable")

    frames=[]
    for fn in files:
        im=cv2.imread(fn,cv2.IMREAD_GRAYSCALE)
        if im is not None: frames.append((fn,im))
    print("="*116)
    print("V44.5 — ARUCO-DIRECT EFFECTIVE FOCAL DISCRIMINATOR")
    print("="*116)
    print(f"images={len(frames)} marker_outer_side={a.marker_mm:.3f}mm physical sensor-plane height={a.physical_height_mm:.2f}mm")
    print(f"stored fx/fy={FX:.3f}/{FY:.3f}px; motion residual target k~=1.068 at h~=185mm")
    print()

    scored=[]
    for name,dic in aruco_dicts():
        per=[]
        ids_seen=[]
        for fn,gray in frames:
            ds=detect(gray,dic)
            if not ds: continue
            vals=[]
            for mid,c in ds:
                und=cv2.undistortPoints(c.reshape(-1,1,2),K,D,P=K).reshape(4,2)
                sl=side_lengths(und)
                # perspective-safe local scale: geometric mean of opposite-pair means
                horiz=(sl[0]+sl[2])/2
                vert=(sl[1]+sl[3])/2
                fxeff=horiz*a.physical_height_mm/a.marker_mm
                fyeff=vert*a.physical_height_mm/a.marker_mm
                vals.append((mid,fxeff,fyeff))
                ids_seen.append(mid)
            if vals:
                per.append((Path(fn).name,vals))
        ndet=sum(len(v) for _,v in per)
        nframes=len(per)
        if ndet:
            scored.append((nframes,ndet,name,per,ids_seen))

    if not scored:
        print("No ArUco detections with common OpenCV dictionaries.")
        print("NEXT: identify the target dictionary or use an explicit measured pixel segment.")
        return

    scored.sort(reverse=True)
    print("DICTIONARY SWEEP")
    print("-"*116)
    for nf,nd,n,per,ids in scored[:8]:
        print(f"{n:20s}: frames={nf:2d}/{len(frames)} markers={nd:3d} unique_ids={len(set(ids)):2d}")
    best=scored[0]
    nf,nd,name,per,ids=best
    # Require meaningful repeatability before interpreting geometry.
    if nf < max(3,len(frames)//2):
        print()
        print(f"STOP: best dictionary {name} detects markers in only {nf}/{len(frames)} frames.")
        print("Dictionary is not established reliably; do not infer focal.")
        return

    fxs=[]; fys=[]
    print()
    print(f"SELECTED BY DETECTION CONSISTENCY: {name}")
    print("-"*116)
    for fn,vals in per:
        fx=[x[1] for x in vals]; fy=[x[2] for x in vals]
        fxs.extend(fx); fys.extend(fy)
        print(f"{fn}: markers={len(vals):2d} ids={','.join(str(x[0]) for x in vals)} "
              f"fx_eff_med={statistics.median(fx):7.2f} fy_eff_med={statistics.median(fy):7.2f}")

    mx=statistics.median(fxs); my=statistics.median(fys)
    kx=mx/FX; ky=my/FY
    print()
    print("SUMMARY")
    print("-"*116)
    print(f"detections={len(fxs)} across {nf}/{len(frames)} frames")
    print(f"effective focal medians fx/fy={mx:.2f}/{my:.2f}px")
    kval=(kx+ky)/2
    print(f"focal multipliers kx/ky={kx:.5f}/{ky:.5f}; mean={kval:.5f}")
    # Convert the observed pixel scale into the physical marker side required by two competing hypotheses.
    # Because fx_eff = pixel_side * h / marker_side, marker_side scales inversely with focal multiplier.
    req_marker_stored=a.marker_mm*kval
    req_marker_motion=a.marker_mm*kval/1.068
    # The argument is now a real physical measurement, so report the directly inferred focal.
    # Keep this separate from the two inverse "required marker size" predictions.
    measured_fx=mx
    measured_fy=my
    residual_vs_stored=(kval-1.0)*100.0
    residual_vs_motion=(kval/1.068-1.0)*100.0
    print()
    print("PHYSICAL MARKER-SIDE DISCRIMINATOR")
    print("-"*116)
    print(f"outer black-square side required if stored focal is correct (k=1.000): {req_marker_stored:.3f} mm")
    print(f"outer black-square side required if motion residual focal hypothesis is correct (k=1.068): {req_marker_motion:.3f} mm")
    print()
    print("DIRECT RESULT FOR SUPPLIED PHYSICAL MARKER SIZE")
    print("-"*116)
    print(f"inferred effective fx/fy={measured_fx:.2f}/{measured_fy:.2f}px, mean k={kval:.5f}")
    print(f"vs stored focal k=1.000: {residual_vs_stored:+.2f}%")
    print(f"vs motion hypothesis k=1.068: {residual_vs_motion:+.2f}%")
    if kval < 0.95 or kval > 1.15:
        print("WARNING: large disagreement; before changing calibration, test full planar pose/tilt and verify camera-target distance definition.")
    print()
    print("CAUTION: marker side must be the actual OUTER black-square side. This direct local-scale estimate")
    print("also assumes the target plane is approximately parallel to the sensor plane. It is a discriminator,")
    print("not a replacement for full planar pose estimation.")
    print("="*116)

if __name__=="__main__":
    main()
