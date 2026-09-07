#!/usr/bin/env python3
import sys, csv, cv2, statistics, math
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
calib=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 RECTIFIED X-RESIDUAL FORENSIC ================")
print("run:",run)
print("Цель: понять природу остаточного x-рассогласования после vertical rectification.")
print("A/B normals на этом шаге НЕ сравниваются.")

fs=cv2.FileStorage(str(calib),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat(); D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat(); D2=fs.getNode("D2").mat()
R1=fs.getNode("R1").mat(); R2=fs.getNode("R2").mat()
P1=fs.getNode("P1").mat(); P2=fs.getNode("P2").mat()
fs.release()

size=(640,480)
m1x,m1y=cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
m2x,m2y=cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

with pairs_csv.open(newline="") as f:
    rows=list(csv.DictReader(f))

det=cv2.SIFT_create(nfeatures=2500,contrastThreshold=0.008,edgeThreshold=15)

X_CORRIDOR=8.0
DISP_MIN=220.0
DISP_MAX=380.0
RATIO=0.82

def guided(kL,dL,kR,dR):
    if dL is None or dR is None:return []
    rpts=np.array([k.pt for k in kR],dtype=np.float32)
    out=[]
    used=set()
    for i,k in enumerate(kL):
        xL,yL=k.pt
        mask=(np.abs(rpts[:,0]-xL)<=X_CORRIDOR)&((yL-rpts[:,1])>=DISP_MIN)&((yL-rpts[:,1])<=DISP_MAX)
        inds=np.where(mask)[0]
        if len(inds)<2:continue
        ds=np.sqrt(np.sum((dR[inds]-dL[i])**2,axis=1))
        order=np.argsort(ds)
        if ds[order[0]]>=RATIO*ds[order[1]]:continue
        rid=int(inds[order[0]])
        if rid in used:continue
        used.add(rid)
        out.append((i,rid))
    return out

stage=defaultdict(lambda:defaultdict(list))
all_points=[]

for row in rows:
    st=row["stage"]
    L0=cv2.imread(str(run/row["left_file"]),0)
    R0=cv2.imread(str(run/row["right_file"]),0)
    if L0 is None or R0 is None:continue
    L=cv2.remap(L0,m1x,m1y,cv2.INTER_LINEAR)
    R=cv2.remap(R0,m2x,m2y,cv2.INTER_LINEAR)

    kL,dL=det.detectAndCompute(L,None)
    kR,dR=det.detectAndCompute(R,None)
    gm=guided(kL,dL,kR,dR)
    if not gm:continue

    for i,j in gm:
        xL,yL=kL[i].pt; xR,yR=kR[j].pt
        dx=float(xL-xR)
        disp=float(yL-yR)
        if not (DISP_MIN<=disp<=DISP_MAX):continue

        stage[st]["dx"].append(dx)
        stage[st]["x"].append((xL+xR)*0.5)
        stage[st]["y"].append((yL+yR)*0.5)
        stage[st]["disp"].append(disp)
        all_points.append((st,dx,(xL+xR)*0.5,(yL+yR)*0.5,disp))

def linfit(x,y):
    x=np.asarray(x,float); y=np.asarray(y,float)
    if len(x)<3:return (float("nan"),float("nan"),float("nan"))
    A=np.column_stack([x,np.ones_like(x)])
    a,b=np.linalg.lstsq(A,y,rcond=None)[0]
    pred=a*x+b
    ssr=np.sum((y-pred)**2); sst=np.sum((y-y.mean())**2)
    r2=1-ssr/sst if sst>1e-12 else 0.0
    return float(a),float(b),float(r2)

print("\n================ STAGE SIGNED DX ================")
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    s=stage[st]
    dx=s["dx"]
    print(f"\n{st}: matches={len(dx)}")
    if not dx:continue
    print(f"  dx median/mean/std={statistics.median(dx):+.3f}/{np.mean(dx):+.3f}/{np.std(dx):.3f} px")
    print(f"  |dx| median/p90={np.median(np.abs(dx)):.3f}/{np.percentile(np.abs(dx),90):.3f} px")
    ax,bx,r2x=linfit(s["x"],dx)
    ay,by,r2y=linfit(s["y"],dx)
    ad,bd,r2d=linfit(s["disp"],dx)
    print(f"  dx vs x: slope={ax:+.5f} px/px  R2={r2x:.3f}")
    print(f"  dx vs y: slope={ay:+.5f} px/px  R2={r2y:.3f}")
    print(f"  dx vs disparity: slope={ad:+.5f}  R2={r2d:.3f}")

# Position-level comparison A vs B
A=[p for p in all_points if p[0].startswith("A")]
B=[p for p in all_points if p[0].startswith("B")]
print("\n================ POSITION COMPARISON ================")
for name,pts in (("A",A),("B",B)):
    if not pts:continue
    dx=np.array([p[1] for p in pts])
    print(f"{name}: n={len(dx)} median dx={np.median(dx):+.3f} mean={np.mean(dx):+.3f} std={np.std(dx):.3f} px")

# Constant-offset test
all_dx=np.array([p[1] for p in all_points],float)
if len(all_dx):
    off=float(np.median(all_dx))
    residual=all_dx-off
    print("\n================ CONSTANT-OFFSET TEST ================")
    print(f"global median offset={off:+.3f} px")
    print(f"after subtracting offset: |residual| median/p90={np.median(np.abs(residual)):.3f}/{np.percentile(np.abs(residual),90):.3f} px")

# Global trend models
if all_points:
    arr=np.array([[p[2],p[3],p[4],1.0] for p in all_points],float)
    y=np.array([p[1] for p in all_points],float)
    coef=np.linalg.lstsq(arr,y,rcond=None)[0]
    pred=arr@coef
    res=y-pred
    ssr=np.sum(res**2); sst=np.sum((y-y.mean())**2)
    r2=1-ssr/sst if sst>1e-12 else 0.0
    print("\n================ GLOBAL AFFINE RESIDUAL MODEL ================")
    print(f"dx ~= a*x + b*y + c*disp + d")
    print(f"a={coef[0]:+.6f} b={coef[1]:+.6f} c={coef[2]:+.6f} d={coef[3]:+.3f}  R2={r2:.3f}")
    print(f"model residual |dx| median/p90={np.median(np.abs(res)):.3f}/{np.percentile(np.abs(res),90):.3f} px")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- Почти постоянный signed dx во всех stages -> вероятен систематический rectification offset.")
print("- Сильная зависимость dx от x/y -> возможен residual rotation/distortion mismatch.")
print("- Сильная зависимость dx от disparity -> возможна stereo-geometry/scale mismatch.")
print("- Отличающийся A vs B signed dx -> особенно опасен: он может смещать будущую A/B normal comparison.")
print("- Пока этот diagnostic не объяснён, plane normals сравнивать нельзя.")
