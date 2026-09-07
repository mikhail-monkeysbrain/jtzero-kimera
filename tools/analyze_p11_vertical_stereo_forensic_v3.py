#!/usr/bin/env python3
import sys,csv,cv2,statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
cal=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 VERTICAL STEREO FORENSIC V3 ================")
print("run:",run)
print("Цель: локализовать оставшийся systematic FAIL после исправления оси stereo.")
print("На этом шаге ничего не считаем про A/B orientation.")

fs=cv2.FileStorage(str(cal),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat();D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat();D2=fs.getNode("D2").mat()
R1=fs.getNode("R1").mat();R2=fs.getNode("R2").mat()
P1=fs.getNode("P1").mat();P2=fs.getNode("P2").mat()
fs.release()

print(f"P2 translation components: tx={P2[0,3]:+.6f} ty={P2[1,3]:+.6f}")
print("Expected rectified axis: VERTICAL")

size=(640,480)
m1x,m1y=cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
m2x,m2y=cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

with (run/"p11_stereo_pairs.csv").open(newline="") as f:
    rows=list(csv.DictReader(f))

det=cv2.SIFT_create(nfeatures=2500,contrastThreshold=0.008,edgeThreshold=15)
bf=cv2.BFMatcher(cv2.NORM_L2)

S=defaultdict(lambda:defaultdict(list))
diag=run/"stereo_forensic_v3"
diag.mkdir(exist_ok=True)

for ri,row in enumerate(rows):
    st=row["stage"]
    L0=cv2.imread(str(run/row["left_file"]),0)
    R0=cv2.imread(str(run/row["right_file"]),0)
    if L0 is None or R0 is None:
        S[st]["read_fail"].append(1); continue
    L=cv2.remap(L0,m1x,m1y,cv2.INTER_LINEAR)
    R=cv2.remap(R0,m2x,m2y,cv2.INTER_LINEAR)
    kL,dL=det.detectAndCompute(L,None)
    kR,dR=det.detectAndCompute(R,None)
    S[st]["kpL"].append(len(kL));S[st]["kpR"].append(len(kR))
    if dL is None or dR is None: continue
    knn=bf.knnMatch(dL,dR,k=2)
    good=[]
    for ms in knn:
        if len(ms)==2 and ms[0].distance < 0.80*ms[1].distance:
            good.append(ms[0])
    S[st]["ratio"].append(len(good))
    if not good: continue

    dx=[];dy=[];records=[]
    for m in good:
        p=kL[m.queryIdx].pt;q=kR[m.trainIdx].pt
        ddx=float(p[0]-q[0]); ddy=float(p[1]-q[1])
        dx.append(ddx);dy.append(ddy);records.append((m,p,q))
    S[st]["absdx_all"].extend(np.abs(dx).tolist())
    S[st]["dy_all"].extend(dy)

    for epi in (1.5,2.5,5.0,10.0):
        mask=np.abs(dx)<=epi
        S[st][f"epi{epi}"].append(int(mask.sum()))
        if mask.any():
            S[st][f"dy_epi{epi}"].extend(np.array(dy)[mask].tolist())

    mask25=np.abs(dx)<=2.5
    dy25=np.array(dy)[mask25]
    S[st]["pos8"].append(int(np.sum(dy25>=8)))
    S[st]["neg8"].append(int(np.sum(dy25<=-8)))
    S[st]["abs8"].append(int(np.sum(np.abs(dy25)>=8)))

    # Triangulate both signs without pre-selecting sign, then count positive Z.
    inds=np.where(mask25 & (np.abs(np.array(dy))>=2.0))[0]
    if len(inds):
        pL=np.array([records[i][1] for i in inds],dtype=np.float64)
        pR=np.array([records[i][2] for i in inds],dtype=np.float64)
        Xh=cv2.triangulatePoints(P1,P2,pL.T,pR.T)
        ok=np.abs(Xh[3])>1e-9
        X=np.full((len(inds),3),np.nan)
        X[ok]=(Xh[:3,ok]/Xh[3,ok]).T
        finite=np.isfinite(X).all(axis=1)
        z=X[:,2]
        S[st]["tri_total"].append(int(finite.sum()))
        S[st]["tri_posz"].append(int(np.sum(finite & (z>0.05)&(z<3.0))))
        S[st]["tri_negz"].append(int(np.sum(finite & (z<0))))
        zp=z[finite & (z>0.05)&(z<3.0)]
        if len(zp): S[st]["depth"].extend(zp.tolist())
    else:
        S[st]["tri_total"].append(0);S[st]["tri_posz"].append(0);S[st]["tri_negz"].append(0)

    if int(row["pair_index"])==10:
        vis=cv2.drawMatches(L,kL,R,kR,good[:120],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imwrite(str(diag/f"{st}_ratio_matches.png"),vis)

def med(x):
    return statistics.median(x) if x else float("nan")
def pct(x,p):
    return float(np.percentile(x,p)) if x else float("nan")

print("\n================ FILTER BREAKDOWN ================")
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    x=S[st]
    print(f"\n{st}:")
    print(f"  keypoints L/R median={med(x['kpL']):.0f}/{med(x['kpR']):.0f}")
    print(f"  ratio matches median={med(x['ratio']):.0f}")
    print(f"  raw |dx| median/p90={med(x['absdx_all']):.2f}/{pct(x['absdx_all'],90):.2f} px")
    print(f"  pass |dx|<=1.5/2.5/5/10 median={med(x['epi1.5']):.0f}/{med(x['epi2.5']):.0f}/{med(x['epi5.0']):.0f}/{med(x['epi10.0']):.0f}")
    if x["dy_epi2.5"]:
        print(f"  yL-yR after |dx|<=2.5 median/p10/p90={med(x['dy_epi2.5']):+.2f}/{pct(x['dy_epi2.5'],10):+.2f}/{pct(x['dy_epi2.5'],90):+.2f} px")
    print(f"  after epi: dy>=+8 / dy<=-8 / |dy|>=8 median={med(x['pos8']):.0f}/{med(x['neg8']):.0f}/{med(x['abs8']):.0f}")
    print(f"  triangulated finite / positive-depth / negative-depth median={med(x['tri_total']):.0f}/{med(x['tri_posz']):.0f}/{med(x['tri_negz']):.0f}")
    if x["depth"]:
        print(f"  positive depth median/p10/p90={med(x['depth']):.3f}/{pct(x['depth'],10):.3f}/{pct(x['depth'],90):.3f} m")

print("\n================ DECISION HELP ================")
print("- Если |dx| после rectification большой: rectification/calibration/order остаются подозреваемыми.")
print("- Если |dx| малый, но usable убивает знак y-disparity: исправляем только sign gate.")
print("- Если triangulation даёт positive depth при противоположном знаке disparity: это прямое подтверждение sign-gate bug.")
print("- Если positive-depth points мало даже без sign gate: проблема глубже, и plane estimator писать нельзя.")
print("Diagnostics:",diag)
