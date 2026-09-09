#!/usr/bin/env python3
import argparse,csv,statistics
from pathlib import Path
import cv2, numpy as np

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641
D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64).reshape(-1,1)

def read_csv(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))

def decode(mj,row):
    mj.seek(int(row["offset"]))
    b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): return None
    return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

def analyze(root,square_mm,marker_mm,stride):
    frames=read_csv(root/"frames.csv")
    ranges=read_csv(root/"range.csv")
    valid_range=[float(r["distance_cm"])*10.0 for r in ranges if int(r["valid"])==1]
    if not valid_range:
        raise RuntimeError(f"{root}: no valid range")

    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),square_mm/1000.0,marker_mm/1000.0,dic)
    det=cv2.aruco.CharucoDetector(board)
    obj_all=np.asarray(board.getChessboardCorners(),dtype=np.float64)
    K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],dtype=np.float64)

    hs=[]
    errs=[]
    used=0
    with (root/"frames.mjpg").open("rb") as mj:
        for i,row in enumerate(frames):
            if i%stride: continue
            im=decode(mj,row)
            if im is None: continue
            cc,ci,_,_=det.detectBoard(im)
            if ci is None or cc is None or len(ci)<8: continue
            ids=np.asarray(ci,dtype=np.int32).reshape(-1)
            pts=np.asarray(cc,dtype=np.float64).reshape(-1,2)
            obj=obj_all[ids].reshape(-1,3)
            ok,rvec,tvec=cv2.solvePnP(obj,pts,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: continue
            R,_=cv2.Rodrigues(rvec)
            C=(-R.T@tvec.reshape(3,1)).reshape(3)
            h=abs(float(C[2]))*1000.0
            proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
            e=np.linalg.norm(proj.reshape(-1,2)-pts,axis=1)
            hs.append(h); errs.append(float(np.sqrt(np.mean(e*e)))); used+=1

    if not hs:
        raise RuntimeError(f"{root}: no usable Charuco PnP")

    luna=statistics.median(valid_range)
    cam=statistics.median(hs)
    return dict(root=root,luna=luna,cam=cam,offset=luna-cam,
                range_mean=statistics.mean(valid_range),
                nrange=len(valid_range),ncam=used,
                rms=statistics.median(errs))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",nargs="+")
    ap.add_argument("--square-mm",type=float,default=26.47)
    ap.add_argument("--marker-mm",type=float,default=19.411)
    ap.add_argument("--stride",type=int,default=10)
    a=ap.parse_args()

    print("="*110)
    print("R11 — STATIC TF-LUNA ↔ OV9281 OPTICAL-CENTER HEIGHT OFFSET")
    print("="*110)
    results=[]
    for p in a.runs:
        r=analyze(Path(p),a.square_mm,a.marker_mm,a.stride)
        results.append(r)
        print(f"{r['root']}:")
        print(f"  Luna median = {r['luna']:.2f} mm  (n={r['nrange']})")
        print(f"  Camera PnP  = {r['cam']:.2f} mm  (n={r['ncam']}, reproj_med={r['rms']:.3f}px)")
        print(f"  Luna-camera offset = {r['offset']:+.2f} mm")

    if len(results)>=2:
        offs=[r["offset"] for r in results]
        cams=[r["cam"] for r in results]
        lunas=[r["luna"] for r in results]
        print("\nCROSS-HEIGHT CONSISTENCY")
        print("-"*110)
        print(f"offset median={statistics.median(offs):+.2f} mm range=[{min(offs):+.2f},{max(offs):+.2f}] span={max(offs)-min(offs):.2f} mm")
        print(f"camera-height span={max(cams)-min(cams):.2f} mm; Luna span={max(lunas)-min(lunas):.2f} mm")
        if max(offs)-min(offs)<=5.0:
            print("VERDICT: APPROXIMATELY CONSTANT SENSOR OFFSET — use camera optical-center height, not raw Luna distance, for planar metric projection.")
        else:
            print("VERDICT: OFFSET NOT CONSTANT — inspect tilt, plane geometry, Luna alignment, or PnP model before adopting a fixed correction.")
    print("="*110)

if __name__=="__main__":
    main()
