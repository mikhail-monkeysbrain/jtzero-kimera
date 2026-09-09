#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path
import cv2
import numpy as np

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)

def read_frame(mj,r):
    mj.seek(int(r["offset"]))
    b=mj.read(int(r["bytes"]))
    if len(b)!=int(r["bytes"]):raise RuntimeError("short mjpeg read")
    g=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)
    if g is None:raise RuntimeError("decode failed")
    return g

def nearest(rows,key,t):
    return min(rows,key=lambda r:abs(I(r[key])-t))

def analyze(root):
    ev=load(root/"events.csv")
    sf=load(root/"selected_frames.csv")
    be=load(root/"backend.csv")
    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    ts0,ts1=I(s["state_timestamp_ns"]),I(e["state_timestamp_ns"])
    fr=[r for r in sf if ts0<=I(r["timestamp_ns"])<=ts1]
    if len(fr)<3:raise RuntimeError("too few frames")

    ax,ay=F(s["px"]),F(s["py"])
    move_be=[r for r in be if I(s["wall_ns"])<=I(r["callback_wall_ns"])<=I(e["wall_ns"])]
    peak=max(move_be,key=lambda r:(F(r["px"])-ax)**2+(F(r["py"])-ay)**2)
    ux,uy=F(peak["px"])-ax,F(peak["py"])-ay
    un=math.hypot(ux,uy);ux/=un;uy/=un

    out=[]
    cum=0.0
    with (root/"selected.mjpg").open("rb") as mj:
        prev=read_frame(mj,fr[0])
        out.append((I(fr[0]["timestamp_ns"]),0.0))
        for r in fr[1:]:
            cur=read_frame(mj,r)
            p0=cv2.goodFeaturesToTrack(prev,maxCorners=900,qualityLevel=0.01,minDistance=7,blockSize=7)
            step=0.0
            if p0 is not None and len(p0)>=30:
                p1,st,_=cv2.calcOpticalFlowPyrLK(prev,cur,p0,None,winSize=(21,21),maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
                ok=st.reshape(-1).astype(bool)
                a=p0.reshape(-1,2)[ok];b=p1.reshape(-1,2)[ok]
                if len(a)>=20:
                    H,m=cv2.findHomography(a,b,cv2.RANSAC,2.0)
                    if H is not None and m is not None and int(m.sum())>=15:
                        h,w=prev.shape[:2]
                        c=np.array([w*.5,h*.5,1.0])
                        q=H@c;q=q[:2]/q[2]
                        step=float(np.linalg.norm(q-c[:2]))
            cum+=step
            out.append((I(r["timestamp_ns"]),cum))
            prev=cur

    total=cum
    samples=[]
    for ts,cp in out:
        b=nearest(be,"timestamp_ns",ts)
        x,y=F(b["px"])-ax,F(b["py"])-ay
        prog=(x*ux+y*uy)*1000
        samples.append((cp/total if total>0 else 0.0,prog,I(b["keyframe"])))
    return samples,total,un*1000

def interp(samples,q):
    if q<=samples[0][0]:return samples[0][1]
    for a,b in zip(samples,samples[1:]):
        if a[0]<=q<=b[0]:
            if b[0]==a[0]:return b[1]
            u=(q-a[0])/(b[0]-a[0])
            return a[1]+u*(b[1]-a[1])
    return samples[-1][1]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",nargs="+")
    a=ap.parse_args()
    data=[]
    for p in a.runs:
        root=Path(p);s,total,peak=analyze(root)
        data.append((root.name,s,total,peak))

    qs=[i/10 for i in range(11)]
    print("="*142)
    print("CLEAN-01 — BACKEND PROGRESS ALIGNED BY RAW VISUAL PATH FRACTION")
    print("="*142)
    print("Raw-image path fraction is used as the alignment coordinate; no metric camera model or IMU is used.")
    print()
    hdr=f"{'raw%':>6}"
    for name,_,_,_ in data:hdr+=f" {name[:18]:>18}"
    print(hdr)
    print("-"*142)
    for q in qs:
        line=f"{q*100:5.0f}%"
        for _,s,_,_ in data:
            line+=f" {interp(s,q):18.2f}"
        print(line)

    print()
    print("RUN TOTALS")
    print("-"*142)
    for name,_,total,peak in data:
        print(f"{name:42s} raw_path={total:8.1f}px backend_peak={peak:8.2f}mm")
    print()
    print("INTERPRETATION")
    print("-"*142)
    print("If BAD backend progress separates early at the same raw-image path fraction, the failure is accumulated during fusion, not caused only by the final stop.")
    print("If trajectories track together until late, focus on the stopping/rollback phase.")
    print("="*142)

if __name__=="__main__":main()
