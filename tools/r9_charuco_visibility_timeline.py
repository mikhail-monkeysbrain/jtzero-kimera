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

def read_frame(mj,row):
    mj.seek(int(row["offset"]))
    b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): return None
    return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

def median_xy(vals):
    if not vals: return None
    return (statistics.median(v[0] for v in vals),statistics.median(v[1] for v in vals))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--stride",type=int,default=10)
    ap.add_argument("--square-mm",type=float,default=26.47)
    ap.add_argument("--marker-mm",type=float,default=19.411)
    a=ap.parse_args()
    root=Path(a.run)
    frames=read_csv(root/"frames.csv"); events=read_csv(root/"events.csv")
    starts=[int(x["recv_mono_ns"]) for x in events if x["event"]=="MOVE_START"]
    ends=[int(x["recv_mono_ns"]) for x in events if x["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1: raise SystemExit("ERROR: need one MOVE_START and one MOVE_END")
    t0,t1=starts[0],ends[0]

    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((7,5),a.square_mm/1000.0,a.marker_mm/1000.0,dic)
    det=cv2.aruco.CharucoDetector(board)
    obj_all=np.asarray(board.getChessboardCorners(),dtype=np.float64)
    K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],dtype=np.float64)

    buckets={k:[] for k in ("pre","move","post")}
    counts={k:0 for k in buckets}
    sampled={k:0 for k in buckets}

    with (root/"frames.mjpg").open("rb") as mj:
        for i,row in enumerate(frames):
            if i%a.stride: continue
            t=int(row["recv_mono_ns"])
            phase="pre" if t<t0 else ("move" if t<=t1 else "post")
            sampled[phase]+=1
            im=read_frame(mj,row)
            if im is None: continue
            cc,ci,_,_=det.detectBoard(im)
            n=0 if ci is None or cc is None else len(ci)
            if n<8: continue
            counts[phase]+=1
            ids=np.asarray(ci,dtype=np.int32).reshape(-1)
            pts=np.asarray(cc,dtype=np.float64).reshape(-1,2)
            obj=obj_all[ids].reshape(-1,3)
            ok,rvec,tvec=cv2.solvePnP(obj,pts,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: continue
            R,_=cv2.Rodrigues(rvec)
            C=(-R.T@tvec.reshape(3,1)).reshape(3)
            buckets[phase].append((float(C[0]),float(C[1]),float(C[2]),n,int(row["frame_id"])))

    print("="*110)
    print("R9 — FULL R1 CHARUCO VISIBILITY / BOARD-RELATIVE POSE FEASIBILITY")
    print("="*110)
    print(f"run={root} stride={a.stride}")
    for phase in ("pre","move","post"):
        vals=buckets[phase]
        print(f"{phase.upper():5s}: sampled={sampled[phase]:4d} detections>=8={counts[phase]:4d} pnp={len(vals):4d}")
        if vals:
            mx=statistics.median(v[0] for v in vals); my=statistics.median(v[1] for v in vals); mz=statistics.median(v[2] for v in vals)
            print(f"       median camera center wrt board = ({mx*1000:+.1f},{my*1000:+.1f},{mz*1000:+.1f}) mm")
            print(f"       corner_count median={statistics.median(v[3] for v in vals):.1f}")

    pre=buckets["pre"]; post=buckets["post"]
    print("\nDIRECT PRE→POST BOARD-RELATIVE CHECK")
    print("-"*110)
    if len(pre)>=3 and len(post)>=3:
        p=np.array([median_xy([(v[0],v[1]) for v in pre])])
        q=np.array([median_xy([(v[0],v[1]) for v in post])])
        dx=(q[0,0]-p[0,0])*1000; dy=(q[0,1]-p[0,1])*1000
        d=(dx*dx+dy*dy)**0.5
        print(f"dx={dx:+.2f} mm dy={dy:+.2f} mm planar={d:.2f} mm")
        print("VERDICT: BOARD IS VISIBLE PRE AND POST — direct metric cross-check is feasible.")
    else:
        print("VERDICT: BOARD NOT SUFFICIENTLY VISIBLE PRE AND POST — direct ChArUco 500-mm cross-check is not feasible.")
        print("NEXT: use raw-frame planar motion / homography with FC attitude and TF-Luna, independent of Kimera.")
    print("="*110)

if __name__=="__main__": main()
