#!/usr/bin/env python3
import argparse,csv,math,re,statistics
from pathlib import Path
import cv2
import numpy as np

def read_frames(root):
    with (root/"frames.csv").open(newline="") as f:
        return list(csv.DictReader(f))

def read_events(root):
    with (root/"events.csv").open(newline="") as f:
        return list(csv.DictReader(f))

def read_payload(mj,row):
    mj.seek(int(row["offset"]))
    b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): raise RuntimeError("short MJPEG read")
    im=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)
    if im is None: raise RuntimeError("JPEG decode failed")
    return im

def read_intrinsics(path):
    txt=Path(path).read_text()
    mi=re.search(r'intrinsics:\\s*\\[([^\\]]+)\\]',txt)
    md=re.search(r'distortion_coefficients:\\s*\\[([^\\]]+)\\]',txt)
    if not (mi and md):
        raise RuntimeError("camera yaml parse failed")
    intr=[float(x.strip()) for x in mi.group(1).split(",")]
    d=[float(x.strip()) for x in md.group(1).split(",")]
    if len(intr)!=4 or len(d)<4:
        raise RuntimeError("unexpected camera yaml values")
    return intr,d

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--square-mm",type=float,default=26.47)
    ap.add_argument("--marker-mm",type=float,default=19.411)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    ap.add_argument("--max-frames",type=int,default=40)
    a=ap.parse_args()

    root=Path(a.run).expanduser()
    rows=read_frames(root); ev=read_events(root)
    starts=[int(x["recv_mono_ns"]) for x in ev if x["event"]=="MOVE_START"]
    if len(starts)!=1: raise RuntimeError("need exactly one MOVE_START")
    tstart=starts[0]
    pre=[r for r in rows if int(r["recv_mono_ns"])<tstart]
    if not pre: raise RuntimeError("no pre-move frames")

    # Spread samples through the whole stationary pre-move interval.
    n=min(a.max_frames,len(pre))
    idx=np.linspace(0,len(pre)-1,n,dtype=int)
    samples=[pre[i] for i in idx]

    intr,D=read_intrinsics(Path(a.camera_yaml))
    fx,fy,cx,cy=intr
    D=np.asarray(D,dtype=np.float64).reshape(-1,1)

    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),a.square_mm/1000.0,a.marker_mm/1000.0,dictionary)
    detector=cv2.aruco.CharucoDetector(board)
    chess=np.asarray(board.getChessboardCorners(),dtype=np.float64)

    detections=[]
    with (root/"frames.mjpg").open("rb") as mj:
        for r in samples:
            im=read_payload(mj,r)
            cc,ci,mc,mi=detector.detectBoard(im)
            if ci is None or cc is None or len(ci)<8: continue
            ids=np.asarray(ci,dtype=np.int32).reshape(-1)
            pts=np.asarray(cc,dtype=np.float64).reshape(-1,2)
            obj=chess[ids].reshape(-1,3)
            detections.append((int(r["frame_id"]),obj,pts))

    print("="*118)
    print("R1 — SAME-DATASET PRE-MOVE CHARUCO EFFECTIVE-FOCAL CHECK")
    print("="*118)
    print(f"run={root}")
    print(f"pre_move_frames={len(pre)} sampled={len(samples)} detections={len(detections)}")
    print(f"actual board square={a.square_mm:.3f} mm marker={a.marker_mm:.3f} mm physical_height={a.physical_height_mm:.3f} mm")
    print(f"stored fx/fy={fx:.6f}/{fy:.6f}")

    if len(detections)<3:
        print("RESULT: INSUFFICIENT CHARUCO DETECTIONS")
        print("Board is not sufficiently visible in the sampled pre-move frames.")
        print("="*118)
        return

    out=[]
    for k in np.arange(0.95,1.2001,0.0025):
        K=np.array([[fx*k,0,cx],[0,fy*k,cy],[0,0,1]],dtype=np.float64)
        hs=[]; rms=[]
        for fid,obj,img in detections:
            ok,rvec,tvec=cv2.solvePnP(obj,img,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: continue
            Rm,_=cv2.Rodrigues(rvec)
            C=-Rm.T@tvec.reshape(3,1)
            h=abs(float(C[2,0]))*1000.0
            proj,_=cv2.projectPoints(obj,rvec,tvec,K,D)
            e=np.linalg.norm(proj.reshape(-1,2)-img,axis=1)
            hs.append(h); rms.append(float(np.sqrt(np.mean(e*e))))
        if hs:
            mh=statistics.median(hs); mr=statistics.mean(rms)
            herr=(mh-a.physical_height_mm)/a.physical_height_mm*100.0
            out.append((abs(herr),mr,k,mh,herr,len(hs)))

    by_h=sorted(out,key=lambda x:(x[0],x[1]))[:12]
    by_r=sorted(out,key=lambda x:(x[1],x[0]))[:12]

    print("\nBEST PHYSICAL-HEIGHT MATCH")
    print("-"*118)
    for _,rms,k,h,he,n in by_h:
        print(f"k={k:7.4f}  h={h:8.3f} mm  h_err={he:+7.3f}%  rms={rms:6.3f}px  frames={n}")

    print("\nBEST REPROJECTION")
    print("-"*118)
    for _,rms,k,h,he,n in by_r:
        print(f"k={k:7.4f}  rms={rms:6.3f}px  h={h:8.3f} mm  h_err={he:+7.3f}%  frames={n}")

    best=by_h[0]
    _,rms,k,h,he,n=best
    motion_k=552.805/500.0
    print("\nCROSS-CHECK")
    print("-"*118)
    print(f"same-dataset ChArUco height-match k = {k:.6f}")
    print(f"MOVE500 metric residual equivalent k = {motion_k:.6f}")
    print(f"delta = {k-motion_k:+.6f} ({(k/motion_k-1)*100:+.3f}%)")
    print(f"stored-K predicted board height at k=1.0 is reported in the sweep above if present.")
    if abs(k-motion_k)<=0.02:
        print("VERDICT: PASS — known-board geometry and MOVE500 independently support the same ~effective-focal multiplier.")
    else:
        print("VERDICT: MISMATCH — board geometry does not support the MOVE500 equivalent focal scale.")
    print("No production K is changed by this diagnostic.")
    print("="*118)

if __name__=="__main__":
    main()
