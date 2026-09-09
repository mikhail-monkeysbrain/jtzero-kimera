#!/usr/bin/env python3
import argparse,csv,math,re,statistics
from pathlib import Path
import cv2
import numpy as np

def load_csv(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)

def parse_camera(p):
    txt=Path(p).read_text()
    m=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt,re.S)
    if not m:raise RuntimeError("intrinsics not found")
    intr=[float(x.strip()) for x in m.group(1).split(',')]
    t=re.search(r'T_BS:\s*.*?data:\s*\[([^\]]+)\]',txt,re.S)
    if not t:raise RuntimeError("T_BS not found")
    vals=[float(x.strip()) for x in t.group(1).replace("\n"," ").split(',')]
    if len(vals)!=16:raise RuntimeError("bad T_BS")
    T=np.array(vals,dtype=np.float64).reshape(4,4)
    return intr[:4],T

def read_frame(mj,r):
    mj.seek(int(r["offset"]))
    b=mj.read(int(r["bytes"]))
    if len(b)!=int(r["bytes"]):raise RuntimeError("short MJPEG read")
    g=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)
    if g is None:raise RuntimeError("JPEG decode failed")
    return g

def RzRyRx(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([
      [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
      [-sp,   cp*sr,          cp*cr]
    ],dtype=np.float64)

def nearest_att(att,ts):
    return min(att,key=lambda r:abs(I(r["recv_wall_ns"])-ts))

def analyze(root,K,Kinv,B_R_C,fx,fy,h_m):
    ev=load_csv(root/"events.csv")
    fr=load_csv(root/"selected_frames.csv")
    att=load_csv(root/"attitude.csv")
    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    t0=I(s["state_timestamp_ns"]);t1=I(e["state_timestamp_ns"])
    seq=[r for r in fr if t0<=I(r["timestamp_ns"])<=t1]
    if len(seq)<3:raise RuntimeError(f"{root}: too few MOVE frames")

    mx=[];my=[];inliers=[];resids=[];att_dt=[];bad=0
    with (root/"selected.mjpg").open("rb") as mj:
        prev_rec=seq[0]
        prev=read_frame(mj,prev_rec)
        hh,ww=prev.shape[:2]
        center=np.array([ww*.5,hh*.5,1.0],dtype=np.float64)
        a0=nearest_att(att,I(prev_rec["timestamp_ns"]))

        for rec in seq[1:]:
            cur=read_frame(mj,rec)
            a1=nearest_att(att,I(rec["timestamp_ns"]))
            att_dt.append(abs(I(a1["recv_wall_ns"])-I(rec["timestamp_ns"]))/1e6)

            p0=cv2.goodFeaturesToTrack(prev,maxCorners=900,qualityLevel=.01,minDistance=7,blockSize=7)
            if p0 is None or len(p0)<30:
                bad+=1;prev=cur;prev_rec=rec;a0=a1;continue
            p1,st,_=cv2.calcOpticalFlowPyrLK(prev,cur,p0,None,winSize=(21,21),maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
            ok=st.reshape(-1).astype(bool)
            pa=p0.reshape(-1,2)[ok];pb=p1.reshape(-1,2)[ok]
            if len(pa)<20:
                bad+=1;prev=cur;prev_rec=rec;a0=a1;continue
            H,mask=cv2.findHomography(pa,pb,cv2.RANSAC,2.0)
            if H is None or mask is None:
                bad+=1;prev=cur;prev_rec=rec;a0=a1;continue
            mask=mask.reshape(-1).astype(bool)
            if int(mask.sum())<15:
                bad+=1;prev=cur;prev_rec=rec;a0=a1;continue

            W_R_B0=RzRyRx(F(a0["roll"]),F(a0["pitch"]),F(a0["yaw"]))
            W_R_B1=RzRyRx(F(a1["roll"]),F(a1["pitch"]),F(a1["yaw"]))
            W_R_C0=W_R_B0@B_R_C
            W_R_C1=W_R_B1@B_R_C
            C1_R_C0=W_R_C1.T@W_R_C0
            Hrot=K@C1_R_C0@Kinv

            # Remove camera rotation from the observed prev->current homography.
            Hder=np.linalg.inv(Hrot)@H
            Hder=Hder/Hder[2,2]

            q=Hder@center
            q=q[:2]/q[2]
            d=q-center[:2]

            # Image motion is opposite camera translation.
            mx.append(-float(d[0])*h_m/fx)
            my.append(-float(d[1])*h_m/fy)
            inliers.append(int(mask.sum()))

            ah=np.c_[pa[mask],np.ones(int(mask.sum()))]
            pred=(H@ah.T).T
            pred=pred[:,:2]/pred[:,2:3]
            resids.append(float(np.median(np.linalg.norm(pred-pb[mask],axis=1))))

            prev=cur;prev_rec=rec;a0=a1

    path=sum(math.hypot(x,y) for x,y in zip(mx,my))
    netx=sum(mx);nety=sum(my);net=math.hypot(netx,nety)
    return {
      "name":root.name,"frames":len(seq),"good":len(mx),"bad":bad,
      "path_mm":path*1000,"net_mm":net*1000,
      "x_mm":netx*1000,"y_mm":nety*1000,
      "inl":statistics.median(inliers) if inliers else float("nan"),
      "res":statistics.median(resids) if resids else float("nan"),
      "att_med":statistics.median(att_dt) if att_dt else float("nan"),
      "att_max":max(att_dt) if att_dt else float("nan")}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs",nargs="+")
    ap.add_argument("--camera-yaml",required=True)
    ap.add_argument("--height-mm",type=float,required=True)
    ap.add_argument("--truth-mm",type=float,default=500.0)
    a=ap.parse_args()

    (fx,fy,cx,cy),T=parse_camera(a.camera_yaml)
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=np.float64)
    Kinv=np.linalg.inv(K)
    B_R_C=T[:3,:3]
    h=a.height_mm/1000.0
    out=[analyze(Path(r),K,Kinv,B_R_C,fx,fy,h) for r in a.runs]

    print("="*150)
    print("JT-ZERO GROUND MOTION V2 — CAMERA + FC ATTITUDE ROTATION COMPENSATION + KNOWN HEIGHT")
    print("="*150)
    print(f"fx={fx:.3f} fy={fy:.3f} height={a.height_mm:.2f}mm truth={a.truth_mm:.1f}mm")
    print("No Kimera state, no IMU scale estimation, no height fitting.")
    print()
    print(f"{'RUN':42s} {'pairs':>9s} {'path mm':>10s} {'net mm':>10s} {'net X':>10s} {'net Y':>10s} {'err(path)':>11s} {'att dt med/max':>18s}")
    print("-"*150)
    for r in out:
        print(f"{r['name']:42s} {r['good']:4d}/{r['good']+r['bad']:<4d} "
              f"{r['path_mm']:10.2f} {r['net_mm']:10.2f} {r['x_mm']:+10.2f} {r['y_mm']:+10.2f} "
              f"{r['path_mm']-a.truth_mm:+11.2f} {r['att_med']:7.2f}/{r['att_max']:7.2f}ms")
    vals=[r["path_mm"] for r in out]
    print()
    print("REPEATABILITY")
    print("-"*150)
    print(f"path median={statistics.median(vals):.2f}mm min={min(vals):.2f}mm max={max(vals):.2f}mm range={max(vals)-min(vals):.2f}mm")
    if len(vals)>=2:print(f"sample_std={statistics.stdev(vals):.2f}mm")
    print()
    print("INTERPRETATION")
    print("-"*150)
    print("Compare v2 against v1 at the SAME fixed height. Improvement is accepted only if absolute error improves without harming repeatability.")
    print("Do NOT retune height from these results. The next stage, if v2 is sound, is live TF-Luna height.")
    print("="*150)

if __name__=="__main__":main()
