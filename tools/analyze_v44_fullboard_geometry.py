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
    if ids is None: return {}
    return {int(i):c.reshape(4,2).astype(np.float64) for i,c in zip(ids.flatten(),corners)}

def median_geometry(frames):
    byid={}
    for fn,gray in frames:
        ds=detect(gray)
        for mid,c in ds.items():
            byid.setdefault(mid,[]).append(c)
    med={}
    for mid,arr in byid.items():
        if len(arr)>=max(3,len(frames)//2):
            med[mid]=np.median(np.stack(arr),axis=0)
    return med

def cluster(vals,tol):
    groups=[]
    for v in sorted(vals):
        if not groups or abs(v-statistics.mean(groups[-1]))>tol:
            groups.append([v])
        else:
            groups[-1].append(v)
    return [statistics.mean(g) for g in groups]

def assign_lattice(med, cell_mm):
    centers={mid:np.mean(c,axis=0) for mid,c in med.items()}
    xs=[float(c[0]) for c in centers.values()]
    ys=[float(c[1]) for c in centers.values()]
    # infer large-cell pitch from repeated center separations: markers are on alternating squares,
    # so same-row adjacent marker centers are typically 2 cells apart.
    # Use robust smallest horizontal/vertical center separation above marker size.
    def sep(vals):
        ds=sorted(abs(a-b) for i,a in enumerate(vals) for b in vals[i+1:] if abs(a-b)>20)
        return statistics.median(ds[:max(1,min(8,len(ds)))]) if ds else None
    sx2=sep(xs); sy2=sep(ys)
    if sx2 is None or sy2 is None: raise RuntimeError("cannot infer lattice spacing")
    cell_px_x=sx2/2.0
    cell_px_y=sy2/2.0
    # Quantize centers to a 1-cell lattice using the minimum as origin.
    x0=min(xs); y0=min(ys)
    coords={}
    for mid,c in centers.items():
        ix=round((float(c[0])-x0)/cell_px_x)
        iy=round((float(c[1])-y0)/cell_px_y)
        coords[mid]=(ix,iy)
    return coords,cell_px_x,cell_px_y

def build_object_points(med, coords, cell_mm, marker_mm):
    obj=[]; img=[]
    half=marker_mm/2.0
    # OpenCV aruco corner order: TL, TR, BR, BL.
    offs=[(-half,-half),(half,-half),(half,half),(-half,half)]
    for mid,c in med.items():
        if mid not in coords: continue
        ix,iy=coords[mid]
        cx=ix*cell_mm; cy=iy*cell_mm
        for (ox,oy),p in zip(offs,c):
            obj.append([cx+ox,cy+oy,0.0]); img.append(p)
    return np.asarray(obj,np.float64),np.asarray(img,np.float64)

def score_k(obj,img,k,dist_scale=1.0):
    K=np.array([[FX*k,0,CX],[0,FY*k,CY],[0,0,1]],dtype=np.float64)
    D=D0*dist_scale
    ok,rvec,tvec=cv2.solvePnP(obj,img,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: return None
    proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
    proj=proj.reshape(-1,2)
    rms=math.sqrt(float(np.mean(np.sum((proj-img)**2,axis=1))))
    R,_=cv2.Rodrigues(rvec)
    C=-R.T@tvec.reshape(3,1)
    h=abs(float(C[2,0]))
    normal=R[:,2]
    tilt=math.degrees(math.acos(max(-1,min(1,abs(float(normal[2]))))))
    return rms,h,tilt,rvec,tvec

def main():
    ap=argparse.ArgumentParser(description="V44.6 full-board planar ArUco geometry sweep")
    ap.add_argument("--glob",default=str(Path.home()/"v44_4_ov9281_*.jpg"))
    ap.add_argument("--cell-mm",type=float,default=40.385)
    ap.add_argument("--marker-mm",type=float,default=26.47)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    files=sorted(glob.glob(a.glob))
    frames=[(fn,cv2.imread(fn,cv2.IMREAD_GRAYSCALE)) for fn in files]
    frames=[x for x in frames if x[1] is not None]
    med=median_geometry(frames)
    if len(med)<8: raise SystemExit(f"too few stable markers: {len(med)}")
    coords,cpx,cpy=assign_lattice(med,a.cell_mm)
    obj,img=build_object_points(med,coords,a.cell_mm,a.marker_mm)

    print("="*118)
    print("V44.6 — FULL-BOARD PLANAR ARUCO GEOMETRY / FOCAL SWEEP")
    print("="*118)
    print(f"frames={len(frames)} stable_markers={len(med)} points={len(obj)}")
    print(f"physical cell={a.cell_mm:.3f}mm marker={a.marker_mm:.3f}mm sensor-plane height={a.physical_height_mm:.2f}mm")
    print(f"inferred image lattice cell pitch ~= {cpx:.2f}px x {cpy:.2f}px")
    print("ID -> inferred lattice cell (ix,iy):")
    print("  "+", ".join(f"{mid}:{coords[mid]}" for mid in sorted(coords)))
    print()

    # Joint sweep: focal multiplier and distortion strength. Pose is solved independently for each pair.
    ks=np.linspace(0.70,1.15,181)
    ds=np.linspace(0.0,1.5,31)
    best=None
    top=[]
    for k in ks:
        for dscale in ds:
            r=score_k(obj,img,float(k),float(dscale))
            if r is None: continue
            rms,h,tilt,rv,tv=r
            height_pen=abs(h-a.physical_height_mm)/a.physical_height_mm
            # Primary score is reprojection; report independent height mismatch separately.
            rec=(rms,float(k),float(dscale),h,tilt,height_pen)
            top.append(rec)
            if best is None or rms<best[0]: best=rec
    top.sort(key=lambda x:x[0])
    print("BEST REPROJECTION CANDIDATES")
    print("-"*118)
    for rms,k,dscl,h,tilt,hpen in top[:10]:
        print(f"rms={rms:6.3f}px k={k:7.4f} dist_scale={dscl:5.2f} h={h:7.2f}mm "
              f"h_err={(h/a.physical_height_mm-1)*100:+6.2f}% tilt={tilt:5.2f}deg")

    print()
    print("FIXED-HYPOTHESIS CHECKS")
    print("-"*118)
    for label,k in [("stored K",1.0),("motion hypothesis",1.068),("local-scale naive",0.7866)]:
        vals=[]
        for dscl in ds:
            r=score_k(obj,img,k,float(dscl))
            if r: vals.append((r[0],float(dscl),r[1],r[2]))
        vals.sort()
        rms,dscl,h,tilt=vals[0]
        print(f"{label:18s}: best_rms={rms:6.3f}px dist_scale={dscl:5.2f} h={h:7.2f}mm "
              f"h_err={(h/a.physical_height_mm-1)*100:+6.2f}% tilt={tilt:5.2f}deg")

    # Candidate closest to physical height, then reprojection.
    close=sorted(top,key=lambda x:(abs(x[3]-a.physical_height_mm),x[0]))
    print()
    print("PHYSICAL-HEIGHT MATCH")
    print("-"*118)
    for rms,k,dscl,h,tilt,hpen in close[:8]:
        print(f"h={h:7.2f}mm k={k:7.4f} dist_scale={dscl:5.2f} rms={rms:6.3f}px tilt={tilt:5.2f}deg")

    print()
    print("DECISION GUIDE")
    print("-"*118)
    print("- If one k gives both low reprojection RMS and h near 185.5mm, that supports a concrete effective-focal model.")
    print("- If stored K gives low RMS and correct h, the +6.8% motion residual is not a focal-scale problem.")
    print("- If no k/distortion pair gives both low RMS and correct h, the inferred board lattice/plane geometry is wrong or")
    print("  the physical height definition is not compatible with this planar model; do not tune intrinsics.")
    print("="*118)

if __name__=="__main__":
    main()
