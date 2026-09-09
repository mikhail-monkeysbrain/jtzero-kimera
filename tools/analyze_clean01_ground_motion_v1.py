#!/usr/bin/env python3
import argparse,csv,math,re,statistics
from pathlib import Path
import cv2
import numpy as np

def load_csv(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)

def parse_intrinsics(p):
    txt=Path(p).read_text()
    m=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt,re.S)
    if not m:raise RuntimeError("intrinsics not found")
    v=[float(x.strip()) for x in m.group(1).split(',')]
    if len(v)<4:raise RuntimeError("bad intrinsics")
    return v[:4]

def read_frame(mj,r):
    mj.seek(int(r["offset"]))
    b=mj.read(int(r["bytes"]))
    if len(b)!=int(r["bytes"]):raise RuntimeError("short MJPEG read")
    g=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)
    if g is None:raise RuntimeError("JPEG decode failed")
    return g

def analyze(root,fx,fy,h_m):
    ev=load_csv(root/"events.csv")
    fr=load_csv(root/"selected_frames.csv")
    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    t0=I(s["state_timestamp_ns"]);t1=I(e["state_timestamp_ns"])
    seq=[r for r in fr if t0<=I(r["timestamp_ns"])<=t1]
    if len(seq)<3:raise RuntimeError(f"{root}: too few MOVE frames")

    dxs=[];dys=[];inliers=[];resids=[];bad=0
    with (root/"selected.mjpg").open("rb") as mj:
        prev=read_frame(mj,seq[0])
        hh,ww=prev.shape[:2]
        center=np.array([ww*.5,hh*.5,1.0],dtype=np.float64)

        for rec in seq[1:]:
            cur=read_frame(mj,rec)
            p0=cv2.goodFeaturesToTrack(prev,maxCorners=900,qualityLevel=.01,minDistance=7,blockSize=7)
            if p0 is None or len(p0)<30:
                bad+=1;prev=cur;continue
            p1,st,_=cv2.calcOpticalFlowPyrLK(prev,cur,p0,None,winSize=(21,21),maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
            ok=st.reshape(-1).astype(bool)
            a=p0.reshape(-1,2)[ok];b=p1.reshape(-1,2)[ok]
            if len(a)<20:
                bad+=1;prev=cur;continue
            H,mask=cv2.findHomography(a,b,cv2.RANSAC,2.0)
            if H is None or mask is None:
                bad+=1;prev=cur;continue
            mask=mask.reshape(-1).astype(bool)
            if int(mask.sum())<15:
                bad+=1;prev=cur;continue

            q=H@center;q=q[:2]/q[2]
            d=q-center[:2]
            dxs.append(float(d[0]));dys.append(float(d[1]))
            inliers.append(int(mask.sum()))

            ah=np.c_[a[mask],np.ones(int(mask.sum()))]
            pred=(H@ah.T).T
            pred=pred[:,:2]/pred[:,2:3]
            resids.append(float(np.median(np.linalg.norm(pred-b[mask],axis=1))))
            prev=cur

    mx=[-dx*h_m/fx for dx in dxs] # image motion opposite camera motion
    my=[-dy*h_m/fy for dy in dys]
    path=sum(math.hypot(x,y) for x,y in zip(mx,my))
    netx=sum(mx);nety=sum(my);net=math.hypot(netx,nety)
    return {
      "name":root.name,"frames":len(seq),"good":len(mx),"bad":bad,
      "path_mm":path*1000,"net_mm":net*1000,
      "x_mm":netx*1000,"y_mm":nety*1000,
      "inl":statistics.median(inliers) if inliers else float("nan"),
      "res":statistics.median(resids) if resids else float("nan")}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",nargs="+")
    ap.add_argument("--camera-yaml",required=True)
    ap.add_argument("--height-mm",type=float,required=True)
    ap.add_argument("--truth-mm",type=float,default=500.0)
    a=ap.parse_args()
    fx,fy,cx,cy=parse_intrinsics(a.camera_yaml)
    h=a.height_mm/1000.0
    out=[analyze(Path(r),fx,fy,h) for r in a.runs]

    print("="*136)
    print("JT-ZERO GROUND MOTION V1 — RAW CAMERA + KNOWN HEIGHT, NO KIMERA / NO IMU SCALE")
    print("="*136)
    print(f"fx={fx:.3f} fy={fy:.3f} height={a.height_mm:.2f}mm truth={a.truth_mm:.1f}mm")
    print("NOTE: v1 does NOT compensate camera rotation/attitude yet.")
    print()
    print(f"{'RUN':42s} {'pairs':>9s} {'path mm':>10s} {'net mm':>10s} {'net X':>10s} {'net Y':>10s} {'err(path)':>11s} {'res px':>8s}")
    print("-"*136)
    for r in out:
        print(f"{r['name']:42s} {r['good']:4d}/{r['good']+r['bad']:<4d} "
              f"{r['path_mm']:10.2f} {r['net_mm']:10.2f} {r['x_mm']:+10.2f} {r['y_mm']:+10.2f} "
              f"{r['path_mm']-a.truth_mm:+11.2f} {r['res']:8.3f}")
    vals=[r["path_mm"] for r in out]
    print()
    print("REPEATABILITY")
    print("-"*136)
    print(f"path median={statistics.median(vals):.2f}mm min={min(vals):.2f}mm max={max(vals):.2f}mm range={max(vals)-min(vals):.2f}mm")
    if len(vals)>=2:print(f"sample_std={statistics.stdev(vals):.2f}mm")
    print()
    print("INTERPRETATION")
    print("-"*136)
    print("This v1 is a deliberately simple geometric estimator. Do not tune height to force 500 mm.")
    print("If BAD/GOOD become mutually consistent here, the camera+known-height architecture is reproducible even where MonoVIO is not.")
    print("Absolute residual error is addressed next by attitude/rotation compensation and live TF-Luna height.")
    print("="*136)

if __name__=="__main__":main()
