#!/usr/bin/env python3
import argparse,csv,glob,math,statistics
from pathlib import Path
try:
    import cv2
    import numpy as np
except ModuleNotFoundError as e:
    raise SystemExit("ERROR: run with /usr/bin/python3 (cv2 required)") from e

FX=568.53170752165227; FY=569.68005562865858
CX=315.98271077441063; CY=239.88148589100641
D0=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],dtype=np.float64)
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
RBC=[
 [0.012724080, 0.995473080, 0.094188300],
 [0.998083560,-0.006939440,-0.061490300],
 [-0.060558320,0.094790200,-0.993653620],
]

def make_board(square_mm, marker_mm):
    if hasattr(cv2.aruco,"CharucoBoard"):
        try: return cv2.aruco.CharucoBoard((7,5),square_mm,marker_mm,DICT)
        except TypeError: pass
    return cv2.aruco.CharucoBoard_create(7,5,square_mm,marker_mm,DICT)

def board_corners(board):
    return np.asarray(board.getChessboardCorners() if hasattr(board,"getChessboardCorners") else board.chessboardCorners,dtype=np.float64)

def detect_charuco(gray,board):
    if hasattr(cv2.aruco,"ArucoDetector"):
        det=cv2.aruco.ArucoDetector(DICT,cv2.aruco.DetectorParameters())
        mc,mi,_=det.detectMarkers(gray)
    else:
        mc,mi,_=cv2.aruco.detectMarkers(gray,DICT)
    if mi is None or len(mi)<4: return None
    n,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,gray,board)
    if cc is None or ci is None or int(n)<6: return None
    return cc.reshape(-1,2).astype(np.float64),ci.flatten().astype(int)

def aggregate_start(images,board):
    per={}; used=0
    for fn in images:
        g=cv2.imread(str(fn),cv2.IMREAD_GRAYSCALE)
        if g is None: continue
        d=detect_charuco(g,board)
        if d is None: continue
        pts,ids=d; used+=1
        for p,i in zip(pts,ids): per.setdefault(int(i),[]).append(p)
    stable={i:np.median(np.stack(v),axis=0) for i,v in per.items() if len(v)>=max(3,used//2)}
    return used,stable

def solve_start_k(images,square_mm,marker_mm,physical_h):
    board=make_board(square_mm,marker_mm); bc=board_corners(board)
    used,stable=aggregate_start(images,board)
    if len(stable)<8: raise SystemExit(f"start-anchor: too few stable ChArUco corners: {len(stable)}")
    ids=np.array(sorted(stable),dtype=int)
    img=np.array([stable[int(i)] for i in ids],dtype=np.float64)
    obj=bc[ids].astype(np.float64)
    cand=[]
    for k in np.linspace(0.90,1.20,601):
        K=np.array([[FX*k,0,CX],[0,FY*k,CY],[0,0,1]],dtype=np.float64)
        # V44.8 showed distortion scale was not needed for the height closure; keep calibrated D disabled here
        # to avoid trading focal against distortion on a single start plane.
        D=np.zeros(5,dtype=np.float64)
        ok,rv,tv=cv2.solvePnP(obj,img,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: continue
        R,_=cv2.Rodrigues(rv); C=-R.T@tv.reshape(3,1); h=abs(float(C[2,0]))
        proj,_=cv2.projectPoints(obj,rv,tv,K,D); proj=proj.reshape(-1,2)
        rms=math.sqrt(float(np.mean(np.sum((proj-img)**2,axis=1))))
        cand.append((abs(h-physical_h),rms,float(k),h,len(ids),used))
    cand.sort(key=lambda x:(x[0],x[1]))
    return cand[0]

def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mt(A): return [list(x) for x in zip(*A)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def rx(a):
    c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def ry(a):
    c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def rz(a):
    c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def rpy(r,p,y): return mm(rz(y),mm(ry(p),rx(r)))
def interp(xs,ys,x):
    if x<=xs[0]: return ys[0]
    if x>=xs[-1]: return ys[-1]
    lo,hi=0,len(xs)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if xs[m]<=x: lo=m
        else: hi=m
    t=(x-xs[lo])/(xs[hi]-xs[lo])
    return ys[lo]+t*(ys[hi]-ys[lo])
def unwrap_deg(v):
    out=[v[0]]
    for x in v[1:]:
        y=x
        while y-out[-1]>180:y-=360
        while y-out[-1]<-180:y+=360
        out.append(y)
    return out

def rotation_corrected_pixels(run,forensic):
    events=list(csv.DictReader((run/"jtzero_500mm_v25_events.csv").open()))
    att=list(csv.reader((run/"jtzero_500mm_v25_attitude.csv").open()))
    if len(events)<2 or not att: raise SystemExit("missing events/attitude")
    header=None
    try: int(att[0][0]); data=att
    except: header=att[0]; data=att[1:]
    if header:
        idx={k:i for i,k in enumerate(header)}
        ti=idx.get("recv_ns",0);ri=idx.get("roll_deg",2);pi=idx.get("pitch_deg",3);yi=idx.get("yaw_deg",4)
    else: ti,ri,pi,yi=0,2,3,4
    T=[float(r[ti]) for r in data]; RR=[float(r[ri]) for r in data]; PP=[float(r[pi]) for r in data]; YY=unwrap_deg([float(r[yi]) for r in data])
    start=float(events[0]["event_wall_ns"]); end=float(events[1]["event_wall_ns"])
    N=len(forensic); bounds=[start+(end-start)*i/N for i in range(N+1)]
    obsx=obsy=corrx=corry=rotx=roty=0.0
    for i,row in enumerate(forensic):
        def pose(t):
            rr=math.radians(interp(T,RR,t)); pp=math.radians(-interp(T,PP,t)); yy=math.radians(-interp(T,YY,t))
            return rpy(rr,pp,yy)
        Rw0=pose(bounds[i]); Rw1=pose(bounds[i+1])
        Rrel=mm(mt(RBC),mm(mt(Rw1),mm(Rw0,RBC)))
        u=float(row["c0x"]);v=float(row["c0y"])
        ray=[(u-CX)/FX,(v-CY)/FY,1.0]; q=mv(Rrel,ray)
        if abs(q[2])<1e-9: continue
        ur=FX*q[0]/q[2]+CX; vr=FY*q[1]/q[2]+CY
        rfx=ur-u; rfy=vr-v
        ofx=float(row["med_flow_x_px"]); ofy=float(row["med_flow_y_px"])
        obsx+=ofx;obsy+=ofy;rotx+=rfx;roty+=rfy;corrx+=ofx-rfx;corry+=ofy-rfy
    return obsx,obsy,rotx,roty,corrx,corry,(end-start)/1e9

def metric_from_px(px,py,h,k):
    return math.hypot(px*h/(FX*k),py*h/(FY*k))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--start-glob",required=True)
    ap.add_argument("--square-mm",type=float,default=26.47)
    ap.add_argument("--marker-mm",type=float,default=None)
    ap.add_argument("--height-mm",type=float,default=185.5)
    ap.add_argument("--truth-mm",type=float,default=500.0)
    a=ap.parse_args()
    if a.marker_mm is None: a.marker_mm=a.square_mm*(22.0/30.0)
    run=Path(a.run); images=sorted(glob.glob(a.start_glob))
    if not images: raise SystemExit("no start-anchor images")
    dh,rms,kstart,hfit,ncorn,nframes=solve_start_k(images,a.square_mm,a.marker_mm,a.height_mm)
    forensic=list(csv.DictReader((run/"jtzero_v43_camera_forensic.csv").open()))
    if not forensic: raise SystemExit("empty forensic log")
    obsx,obsy,rotx,roty,cx,cy,dur=rotation_corrected_pixels(run,forensic)
    truth=a.truth_mm/1000.0; h=a.height_mm/1000.0
    raw_stored=metric_from_px(obsx,obsy,h,1.0)
    corr_stored=metric_from_px(cx,cy,h,1.0)
    raw_anchor=metric_from_px(obsx,obsy,h,kstart)
    corr_anchor=metric_from_px(cx,cy,h,kstart)

    # Affine estimator path/net under the same fixed physical height and anchor k.
    ax=ay=apath=0.0
    for r in forensic:
        dx=-float(r["tx_px"])*h/(FX*kstart); dy=-float(r["ty_px"])*h/(FY*kstart)
        ax+=dx; ay+=dy; apath+=math.hypot(dx,dy)
    anet=math.hypot(ax,ay)

    print("="*120)
    print("V44.10 — START-BOARD ANCHORED SINGLE 500MM PASS")
    print("="*120)
    print(f"run={run} duration={dur:.3f}s rows={len(forensic)} truth={a.truth_mm:.1f}mm")
    print(f"start ChArUco: frames={nframes} stable_corners={ncorn} square={a.square_mm:.3f}mm marker={a.marker_mm:.3f}mm")
    print(f"start anchor focal_k={kstart:.5f} -> fx={FX*kstart:.2f} fy={FY*kstart:.2f} h_fit={hfit:.2f}mm target_h={a.height_mm:.2f}mm rms={rms:.3f}px")
    print()
    print("PIXEL BUDGET")
    print("-"*120)
    print(f"observed flow sum=({obsx:+.2f},{obsy:+.2f})px")
    print(f"rotation-only sum=({rotx:+.2f},{roty:+.2f})px")
    print(f"rotation-corrected=({cx:+.2f},{cy:+.2f})px")
    print()
    print("500MM RECONCILIATION AT FIXED PHYSICAL HEIGHT")
    print("-"*120)
    for name,v in [
        ("raw + stored focal",raw_stored),
        ("rotation + stored focal",corr_stored),
        ("raw + start-anchor focal",raw_anchor),
        ("rotation + start-anchor focal",corr_anchor),
        ("affine net + start-anchor focal",anet),
    ]:
        print(f"{name:34s}: {v*1000:7.2f}mm error={(v/truth-1)*100:+6.2f}%")
    print(f"affine path + start-anchor focal     : {apath*1000:7.2f}mm path/net={apath/anet if anet else float('nan'):.6f}")
    print()
    print("DECISION")
    print("-"*120)
    print("- The ChArUco board is required ONLY at A/start. It may disappear immediately after movement begins.")
    print("- If start-anchor focal + rotation closes 500mm, runtime projection scale is a causal source.")
    print("- If start-anchor focal under/overshoots while stored focal is closer, the single-plane anchor is not transferable to motion scale.")
    print("- If affine and median-flow disagree materially after the same anchor, estimator/model-fit bias is causal.")
    print("="*120)

if __name__=="__main__":
    main()
