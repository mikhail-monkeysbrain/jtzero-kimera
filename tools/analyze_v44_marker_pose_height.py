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

def detect(gray):
    if hasattr(cv2.aruco,"ArucoDetector"):
        det=cv2.aruco.ArucoDetector(DICT,cv2.aruco.DetectorParameters())
        corners,ids,_=det.detectMarkers(gray)
    else:
        corners,ids,_=cv2.aruco.detectMarkers(gray,DICT)
    if ids is None: return []
    return [(int(i),c.reshape(4,2).astype(np.float64)) for i,c in zip(ids.flatten(),corners)]

def solve_marker(corners, marker_mm, k, dist_scale):
    # ArUco corners are TL,TR,BR,BL in the marker plane.
    s=marker_mm/2.0
    obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],dtype=np.float64)
    K=np.array([[FX*k,0,CX],[0,FY*k,CY],[0,0,1]],dtype=np.float64)
    D=D0*dist_scale
    flags=getattr(cv2,"SOLVEPNP_IPPE_SQUARE",cv2.SOLVEPNP_ITERATIVE)
    ok,rvec,tvec=cv2.solvePnP(obj,corners,K,D,flags=flags)
    if not ok:
        return None
    R,_=cv2.Rodrigues(rvec)
    C=-R.T@tvec.reshape(3,1)
    # Perpendicular camera-center distance to the marker plane.
    h=abs(float(C[2,0]))
    proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
    proj=proj.reshape(4,2)
    rms=math.sqrt(float(np.mean(np.sum((proj-corners)**2,axis=1))))
    n=R[:,2]
    tilt=math.degrees(math.acos(max(-1.0,min(1.0,abs(float(n[2]))))))
    return h,rms,tilt

def collect(frames, marker_mm, k, dist_scale):
    hs=[]; rms=[]; tilts=[]
    for fn,gray in frames:
        for mid,c in detect(gray):
            r=solve_marker(c,marker_mm,k,dist_scale)
            if r is None: continue
            h,e,t=r
            if math.isfinite(h) and 20<h<1000:
                hs.append(h);rms.append(e);tilts.append(t)
    return hs,rms,tilts

def main():
    ap=argparse.ArgumentParser(description="V44.7 independent per-marker planar pose height/focal sweep")
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--marker-mm",type=float,default=26.47)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()

    files=sorted(glob.glob(a.glob))
    frames=[(fn,cv2.imread(fn,cv2.IMREAD_GRAYSCALE)) for fn in files]
    frames=[x for x in frames if x[1] is not None]
    if not frames: raise SystemExit("no frames")

    print("="*120)
    print("V44.7 — PER-MARKER PLANAR POSE HEIGHT / FOCAL SWEEP")
    print("="*120)
    print(f"frames={len(frames)} marker_outer_side={a.marker_mm:.3f}mm physical height={a.physical_height_mm:.2f}mm")
    print(f"stored fx/fy={FX:.3f}/{FY:.3f}px")
    print("This test does NOT use the board lattice. Each marker is an independent metric square.")
    print()

    ks=np.linspace(0.70,1.20,201)
    ds=np.linspace(0.0,1.5,31)
    allres=[]
    for k in ks:
        for dscale in ds:
            hs,errs,tilts=collect(frames,a.marker_mm,float(k),float(dscale))
            if len(hs)<20: continue
            med=statistics.median(hs)
            mad=statistics.median([abs(x-med) for x in hs])
            er=statistics.median(errs)
            ht=statistics.median(tilts)
            allres.append((abs(med-a.physical_height_mm),mad,er,float(k),float(dscale),med,ht,len(hs)))
    if not allres: raise SystemExit("no valid marker poses")

    print("BEST PHYSICAL-HEIGHT MATCHES")
    print("-"*120)
    for dh,mad,er,k,dscl,med,tilt,n in sorted(allres)[:12]:
        print(f"k={k:7.4f} dist_scale={dscl:5.2f} h_med={med:7.2f}mm "
              f"h_err={(med/a.physical_height_mm-1)*100:+6.2f}% h_MAD={mad:6.2f}mm "
              f"reproj_med={er:5.3f}px tilt_med={tilt:5.2f}deg n={n}")

    print()
    print("FIXED HYPOTHESES")
    print("-"*120)
    for label,k in [("stored K",1.0),("motion k",1.068),("naive local",0.78661)]:
        cand=[x for x in allres if abs(x[3]-k)<0.0013]
        cand.sort()
        if not cand:
            print(f"{label:14s}: no candidate"); continue
        dh,mad,er,kk,dscl,med,tilt,n=cand[0]
        print(f"{label:14s}: k={kk:7.4f} dist_scale={dscl:5.2f} h_med={med:7.2f}mm "
              f"h_err={(med/a.physical_height_mm-1)*100:+6.2f}% h_MAD={mad:6.2f}mm "
              f"reproj_med={er:5.3f}px tilt_med={tilt:5.2f}deg")

    # Also rank models by within-plane consistency, then physical height.
    print()
    print("BEST COPLANAR CONSISTENCY")
    print("-"*120)
    for dh,mad,er,k,dscl,med,tilt,n in sorted(allres,key=lambda x:(x[1],x[0],x[2]))[:10]:
        print(f"k={k:7.4f} dist_scale={dscl:5.2f} h_med={med:7.2f}mm h_MAD={mad:6.2f}mm "
              f"h_err={(med/a.physical_height_mm-1)*100:+6.2f}% reproj_med={er:5.3f}px")

    print()
    print("DECISION")
    print("-"*120)
    print("- A credible model should put the median marker-plane distance near 185.5 mm AND keep marker-to-marker height spread small.")
    print("- If k~1.0 does that, stored focal scale is supported.")
    print("- If k~1.068 does that materially better, the motion residual focal hypothesis gains support.")
    print("- If neither does, do not tune intrinsics; the remaining motion-scale error lies elsewhere or the physical target model is incomplete.")
    print("="*120)

if __name__=="__main__":
    main()
