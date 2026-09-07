#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
video_path=run/"ov9281_ruler_pass.avi"
frames_csv=run/"p11_ruler_pass_frames.csv"
calib_path=repo/"calibration"/"ov9281_intrinsics.yaml"

print("================ P11 CALIBRATED ROTATION CONSISTENCY ================")
print("run:",run)
print("Цель: проверить, есть ли согласованная вращательная составляющая кадр->кадр")
print("и исчезает ли она на неподвижных A/B plateau.")
print("Это model-consistency diagnostic, НЕ окончательный физический угол.")

fs=cv2.FileStorage(str(calib_path),cv2.FILE_STORAGE_READ)
K=fs.getNode("camera_matrix").mat()
D=fs.getNode("distortion_coefficients").mat()
fs.release()
if K is None or D is None:
    raise SystemExit("FAIL: calibration missing")
Ki=np.linalg.inv(K)

rows=[]
with frames_csv.open(newline="") as f:
    for r in csv.DictReader(f):
        rows.append({
            "frame_id":int(r["frame_id"]),
            "steady_ns":int(r["steady_ns"]),
            "state":r["state"],
        })

cap=cv2.VideoCapture(str(video_path))
ims=[]
while True:
    ok,im=cap.read()
    if not ok: break
    ims.append(im)
cap.release()

n=min(len(rows),len(ims))
rows=rows[:n]; ims=ims[:n]
if n<20: raise SystemExit("FAIL: too few frames")

# ~25 Hz using real timestamps.
sel=[0]; last=rows[0]["steady_ns"]
for i in range(1,n):
    if rows[i]["steady_ns"]-last>=35_000_000:
        sel.append(i); last=rows[i]["steady_ns"]
if sel[-1]!=n-1: sel.append(n-1)

clahe=cv2.createCLAHE(2.0,(8,8))
def prep(im):
    return clahe.apply(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))

def pair_data(im1,im2):
    g1=prep(im1); g2=prep(im2)
    p0=cv2.goodFeaturesToTrack(g1,700,0.01,6,blockSize=7)
    if p0 is None or len(p0)<20:return None
    p1,s1,_=cv2.calcOpticalFlowPyrLK(g1,g2,p0,None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
    p0b,s2,_=cv2.calcOpticalFlowPyrLK(g2,g1,p1,None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
    if p1 is None or p0b is None:return None
    a=p0.reshape(-1,2); b=p1.reshape(-1,2); bb=p0b.reshape(-1,2)
    good=(s1.ravel()==1)&(s2.ravel()==1)&(np.linalg.norm(a-bb,axis=1)<1.0)
    a=a[good]; b=b[good]
    if len(a)<12:return None
    H,m=cv2.findHomography(a,b,cv2.RANSAC,2.0)
    if H is None or m is None:return None
    inl=m.ravel().astype(bool); a=a[inl]; b=b[inl]
    if len(a)<8:return None
    H=H/H[2,2]
    return H,a,b

def closest_rotation(H):
    # Normalize homography with camera intrinsics, remove arbitrary projective scale,
    # then project to nearest SO(3) via SVD. This is only a rotation-like component.
    A=Ki@H@K
    det=np.linalg.det(A)
    if abs(det)<1e-12:return None
    s=np.cbrt(det)
    A=A/s
    U,_,Vt=np.linalg.svd(A)
    R=U@Vt
    if np.linalg.det(R)<0:
        U[:,-1]*=-1
        R=U@Vt
    # best scalar residual between A and R
    alpha=float(np.sum(A*R)/np.sum(R*R))
    resid=np.linalg.norm(A-alpha*R,"fro")/np.linalg.norm(A,"fro")
    tr=np.clip((np.trace(R)-1)/2,-1,1)
    ang=math.degrees(math.acos(tr))
    rv,_=cv2.Rodrigues(R)
    rv=rv.reshape(3)
    return R,ang,rv,resid

def subset_rotations(a,b):
    # Spatial robustness: estimate local homographies in 4 image quadrants.
    outs=[]
    quads=[
        (0,320,0,240),(320,640,0,240),
        (0,320,240,480),(320,640,240,480)
    ]
    for x0,x1,y0,y1 in quads:
        mask=(a[:,0]>=x0)&(a[:,0]<x1)&(a[:,1]>=y0)&(a[:,1]<y1)
        aa=a[mask]; bb=b[mask]
        if len(aa)<12: continue
        H,m=cv2.findHomography(aa,bb,cv2.RANSAC,2.0)
        if H is None or m is None or int(m.sum())<8:continue
        rr=closest_rotation(H/H[2,2])
        if rr is not None:outs.append(rr)
    return outs

records=[]
Rcum=np.eye(3)
for j in range(1,len(sel)):
    i0,i1=sel[j-1],sel[j]
    pd=pair_data(ims[i0],ims[i1])
    if pd is None:
        records.append({"state":rows[i1]["state"],"ok":False})
        continue
    H,a,b=pd
    cr=closest_rotation(H)
    if cr is None:
        records.append({"state":rows[i1]["state"],"ok":False})
        continue
    R,ang,rv,resid=cr
    subs=subset_rotations(a,b)
    subvec=[]
    for Rs,angs,rvs,resids in subs:
        subvec.append(rvs)
    spread=None
    if len(subvec)>=2:
        subvec=np.asarray(subvec)
        mu=subvec.mean(axis=0)
        spread=float(np.median(np.linalg.norm(subvec-mu,axis=1))*180/math.pi)
    if rows[i1]["state"]=="MOVE_A_TO_B":
        Rcum=R@Rcum
    records.append({
        "state":rows[i1]["state"],"ok":True,
        "angle":ang,"rv":rv,"resid":resid,
        "subset_n":len(subs),"subset_spread":spread
    })

def summarize(st):
    xs=[r for r in records if r.get("ok") and r["state"]==st]
    total=sum(r["state"]==st for r in records)
    print(f"\n{st}: valid={len(xs)}/{total}")
    if not xs:return None
    ang=[r["angle"] for r in xs]
    res=[r["resid"] for r in xs]
    spreads=[r["subset_spread"] for r in xs if r["subset_spread"] is not None]
    rv=np.stack([r["rv"] for r in xs])
    print(f"  rotation-like increment median/p90={statistics.median(ang):.4f}/{np.percentile(ang,90):.4f} deg")
    print(f"  normalized pure-rotation residual median/p90={statistics.median(res):.5f}/{np.percentile(res,90):.5f}")
    if spreads:
        print(f"  spatial-subset rotation spread median/p90={statistics.median(spreads):.4f}/{np.percentile(spreads,90):.4f} deg")
    print(f"  mean rotvec [deg]=[{np.mean(rv[:,0])*180/math.pi:+.4f},{np.mean(rv[:,1])*180/math.pi:+.4f},{np.mean(rv[:,2])*180/math.pi:+.4f}]")
    return {"ang":ang,"res":res,"spreads":spreads}

print(f"selected pair count: {len(records)}")
sa=summarize("PRE_STILL_A")
sm=summarize("MOVE_A_TO_B")
sb=summarize("POST_STILL_B")

# Integrated candidate over MOVE only.
tr=np.clip((np.trace(Rcum)-1)/2,-1,1)
cumang=math.degrees(math.acos(tr))
rv,_=cv2.Rodrigues(Rcum); rv=rv.reshape(3)*180/math.pi
print("\n================ MOVE INTEGRATED ROTATION-LIKE CANDIDATE ================")
print(f"cumulative nearest-rotation angle = {cumang:.3f} deg")
print(f"cumulative rotvec [deg] = [{rv[0]:+.3f},{rv[1]:+.3f},{rv[2]:+.3f}]")

# Conservative gate: still increments must be tiny, move larger, residual and subset spread bounded.
def med(x,default=999): return statistics.median(x) if x else default
still_ok=(sa is not None and sb is not None and
          med(sa["ang"])<=0.08 and med(sb["ang"])<=0.08)
move_signal=(sm is not None and med(sm["ang"])>=0.03)
resid_ok=(sm is not None and np.percentile(sm["res"],90)<=0.03)
spread_ok=(sm is not None and sm["spreads"] and np.percentile(sm["spreads"],90)<=0.20)
passed=bool(still_ok and move_signal and resid_ok and spread_ok)

print("\n================ CONSISTENCY GATE ================")
print("PASS" if passed else "FAIL")
print(f"still_ok={still_ok} move_signal={move_signal} residual_ok={resid_ok} spatial_robustness_ok={spread_ok}")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- PASS означает, что rotation-like component мал на STILL, появляется на MOVE,")
print("  хорошо объясняется calibrated rotation model и устойчив к пространственным subset.")
print("- FAIL означает: накопленный кандидат нельзя считать физическим изменением ориентации.")
print("- Даже PASS не доказывает причину P11: plane translation может частично загрязнять homography.")
print("- Сравнивать cumulative candidate с IMU имеет смысл только после PASS.")
