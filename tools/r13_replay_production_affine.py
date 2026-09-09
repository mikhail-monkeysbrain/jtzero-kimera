#!/usr/bin/env python3
import argparse,csv,math,re,statistics
from pathlib import Path
import cv2
import numpy as np

FX=568.53170752165227
FY=569.68005562865858
CX=315.98271077441063
CY=239.88148589100641

def read_csv(path):
    with path.open(newline="") as f: return list(csv.DictReader(f))

def interp(rows,t,key):
    ts=[int(r["recv_mono_ns"]) for r in rows]
    if t<=ts[0]: return float(rows[0][key])
    if t>=ts[-1]: return float(rows[-1][key])
    lo,hi=0,len(ts)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if ts[m]<=t: lo=m
        else: hi=m
    a=(t-ts[lo])/(ts[hi]-ts[lo])
    return float(rows[lo][key])*(1-a)+float(rows[hi][key])*a

def range_m(rows,t):
    valid=[r for r in rows if int(r["valid"])==1]
    return interp(valid,t,"distance_cm")/100.0

def decode(mj,row):
    mj.seek(int(row["offset"]))
    b=mj.read(int(row["bytes"]))
    if len(b)!=int(row["bytes"]): return None
    return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

def production_pair(prev,cur,h):
    p0=cv2.goodFeaturesToTrack(prev,350,0.01,7.0,blockSize=7,useHarrisDetector=False,k=0.04)
    if p0 is None or len(p0)<40: return None
    p1,st,err=cv2.calcOpticalFlowPyrLK(prev,cur,p0,None,winSize=(21,21),maxLevel=3)
    if p1 is None: return None
    a=[]; b=[]
    p0r=p0.reshape(-1,2); p1r=p1.reshape(-1,2)
    for i in range(len(p0r)):
        if not st[i]: continue
        if err[i]>30.0: continue
        dx=p1r[i,0]-p0r[i,0]; dy=p1r[i,1]-p0r[i,1]
        if math.hypot(dx,dy)>80.0: continue
        a.append((p0r[i,0]-CX,p0r[i,1]-CY))
        b.append((p1r[i,0]-CX,p1r[i,1]-CY))
    if len(a)<30: return None
    a=np.asarray(a,np.float32); b=np.asarray(b,np.float32)
    A,inl=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=2.0,maxIters=2000,confidence=0.99,refineIters=10)
    if A is None or inl is None: return None
    nin=int(np.count_nonzero(inl))
    if nin<20: return None
    tx=float(A[0,2]); ty=float(A[1,2])
    aa=float(A[0,0]); bb=float(A[1,0])
    rot=math.degrees(math.atan2(bb,aa))
    scale=math.hypot(aa,bb)
    dx=-tx*h/FX; dy=-ty*h/FY
    step=math.hypot(dx,dy)
    if not math.isfinite(step) or step>0.030: return None
    return dx,dy,step,nin,tx,ty,rot,scale

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--period-ms",type=float,default=35.0)
    ap.add_argument("--offset-mm",type=float,default=-20.0)
    a=ap.parse_args()
    root=Path(a.run)
    frames=read_csv(root/"frames.csv")
    events=read_csv(root/"events.csv")
    ranges=read_csv(root/"range.csv")
    starts=[int(r["recv_mono_ns"]) for r in events if r["event"]=="MOVE_START"]
    ends=[int(r["recv_mono_ns"]) for r in events if r["event"]=="MOVE_END"]
    if len(starts)!=1 or len(ends)!=1: raise SystemExit("need one MOVE_START/MOVE_END")
    t0,t1=starts[0],ends[0]

    sel=[]; last=-10**30; per=int(a.period_ms*1e6)
    for r in frames:
        t=int(r["recv_mono_ns"])
        if t<t0 or t>t1: continue
        if not sel or t-last>=per:
            sel.append(r); last=t

    sx=sy=path=0.0
    good=bad=0
    rots=[]; scales=[]; txs=[]; tys=[]; hs=[]; inls=[]
    with (root/"frames.mjpg").open("rb") as mj:
        prev=None; prevrow=None
        for row in sel:
            cur=decode(mj,row)
            if cur is None: continue
            if prev is None:
                prev,prevrow=cur,row
                continue
            tm=(int(prevrow["recv_mono_ns"])+int(row["recv_mono_ns"]))//2
            h=max(0.05,range_m(ranges,tm)+a.offset_mm/1000.0)
            z=production_pair(prev,cur,h)
            if z is None:
                bad+=1
            else:
                dx,dy,st,nin,tx,ty,rot,sc=z
                sx+=dx; sy+=dy; path+=st; good+=1
                rots.append(rot); scales.append(sc); txs.append(tx); tys.append(ty); hs.append(h); inls.append(nin)
            prev,prevrow=cur,row

    net=math.hypot(sx,sy)
    print("="*118)
    print("R13 — PRODUCTION AFFINE METRIC ESTIMATOR REPLAY ON RAW R1")
    print("="*118)
    print(f"run={root}")
    print(f"period={a.period_ms:.1f}ms offset={a.offset_mm:+.2f}mm")
    print(f"pairs={good}/{good+bad}")
    print(f"height mean/median={statistics.mean(hs)*1000:.2f}/{statistics.median(hs)*1000:.2f}mm")
    print(f"PRODUCTION-AFFINE net={net*1000:.2f}mm path={path*1000:.2f}mm x={sx*1000:+.2f} y={sy*1000:+.2f}")
    print(f"net scale={net/0.5:.5f} path scale={path/0.5:.5f}")
    print(f"inliers median={statistics.median(inls):.0f}")
    print(f"aff_rot |median|/p90/max={statistics.median(abs(x) for x in rots):.4f}/{np.percentile(np.abs(rots),90):.4f}/{max(abs(x) for x in rots):.4f} deg")
    print(f"aff_scale median/p10/p90={statistics.median(scales):.6f}/{np.percentile(scales,10):.6f}/{np.percentile(scales,90):.6f}")
    print(f"sum tx/ty={sum(txs):+.3f}/{sum(tys):+.3f}px")
    print("\nREFERENCE FROM R10 SAME RAW DATA")
    print("-"*118)
    print("ground-plane + FC attitude + Luna-20mm: net=486.52mm path=499.63mm")
    print("\nDIFFERENCE")
    print("-"*118)
    print(f"affine net - R10 net = {net*1000-486.52:+.2f}mm")
    print(f"affine path - R10 path = {path*1000-499.63:+.2f}mm")
    if abs(path*1000-499.63)>15:
        print("VERDICT: PRODUCTION AFFINE MODEL INTRODUCES MATERIAL METRIC BIAS RELATIVE TO CALIBRATED GROUND-PLANE GEOMETRY.")
    else:
        print("VERDICT: AFFINE AND GROUND-PLANE PATH AGREE; remaining discrepancy is elsewhere.")
    print("="*118)

if __name__=="__main__":
    main()
