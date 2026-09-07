#!/usr/bin/env python3
import sys,csv,cv2,statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
cal=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 RECTIFIED DX TIME/POSITION TEST ================")
print("run:",run)
print("Цель: проверить, связан ли signed dx с позицией A/B или дрейфует со временем.")
print("Нормали плоскости здесь НЕ считаются.")

fs=cv2.FileStorage(str(cal),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat();D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat();D2=fs.getNode("D2").mat()
R1=fs.getNode("R1").mat();R2=fs.getNode("R2").mat()
P1=fs.getNode("P1").mat();P2=fs.getNode("P2").mat()
fs.release()

size=(640,480)
m1x,m1y=cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
m2x,m2y=cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

rows=[]
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
    out=[]; used=set()
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

pair_stats=[]
for global_idx,row in enumerate(rows):
    L0=cv2.imread(str(run/row["left_file"]),0)
    R0=cv2.imread(str(run/row["right_file"]),0)
    if L0 is None or R0 is None:continue
    L=cv2.remap(L0,m1x,m1y,cv2.INTER_LINEAR)
    R=cv2.remap(R0,m2x,m2y,cv2.INTER_LINEAR)
    kL,dL=det.detectAndCompute(L,None); kR,dR=det.detectAndCompute(R,None)
    gm=guided(kL,dL,kR,dR)
    dx=[]
    for i,j in gm:
        xL,yL=kL[i].pt; xR,yR=kR[j].pt
        disp=yL-yR
        if DISP_MIN<=disp<=DISP_MAX:
            dx.append(float(xL-xR))
    if len(dx)>=10:
        pair_stats.append({
            "global_idx":global_idx,
            "stage":row["stage"],
            "position":row["position"],
            "pair_index":int(row["pair_index"]),
            "dx_med":float(np.median(dx)),
            "dx_mean":float(np.mean(dx)),
            "dx_std":float(np.std(dx)),
            "n":len(dx)
        })

print(f"usable pair-level summaries: {len(pair_stats)}/{len(rows)}")

by_stage=defaultdict(list)
for r in pair_stats: by_stage[r["stage"]].append(r)

print("\n================ STAGE PAIR-MEDIAN DX ================")
stage_meds={}
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    xs=by_stage.get(st,[])
    vals=[x["dx_med"] for x in xs]
    if not vals:
        print(f"{st}: no data")
        continue
    med=float(np.median(vals)); stage_meds[st]=med
    print(f"{st}: pair-median dx median={med:+.3f} px  spread(p10/p90)={np.percentile(vals,10):+.3f}/{np.percentile(vals,90):+.3f}")

print("\n================ ADJACENT A/B DELTAS ================")
for a,b in [("A1","B1"),("A2","B2"),("A3","B3")]:
    if a in stage_meds and b in stage_meds:
        print(f"{a}->{b}: B-A dx={stage_meds[b]-stage_meds[a]:+.3f} px")

print("\n================ SAME-POSITION DRIFT ================")
avec=[stage_meds[s] for s in ["A1","A2","A3","A4"] if s in stage_meds]
bvec=[stage_meds[s] for s in ["B1","B2","B3"] if s in stage_meds]
if len(avec)>=2:
    print("A sequence:", " -> ".join(f"{v:+.3f}" for v in avec))
    print(f"A span={max(avec)-min(avec):.3f} px")
if len(bvec)>=2:
    print("B sequence:", " -> ".join(f"{v:+.3f}" for v in bvec))
    print(f"B span={max(bvec)-min(bvec):.3f} px")

# Linear trend by acquisition order
if len(pair_stats)>=10:
    x=np.array([r["global_idx"] for r in pair_stats],float)
    y=np.array([r["dx_med"] for r in pair_stats],float)
    A=np.column_stack([x,np.ones_like(x)])
    slope,inter=np.linalg.lstsq(A,y,rcond=None)[0]
    pred=slope*x+inter
    ssr=np.sum((y-pred)**2);sst=np.sum((y-y.mean())**2)
    r2=1-ssr/sst if sst>1e-12 else 0.
    print("\n================ GLOBAL TIME TREND ================")
    print(f"dx_med ~= slope*pair_order + intercept")
    print(f"slope={slope:+.5f} px/pair  R2={r2:.3f}")

# Compare position effect after removing global linear time trend
if len(pair_stats)>=10:
    resid=y-pred
    Ares=[resid[i] for i,r in enumerate(pair_stats) if r["position"]=="A"]
    Bres=[resid[i] for i,r in enumerate(pair_stats) if r["position"]=="B"]
    print("\n================ POSITION EFFECT AFTER LINEAR TIME DETREND ================")
    if Ares and Bres:
        am=float(np.median(Ares)); bm=float(np.median(Bres))
        print(f"A median residual={am:+.3f} px")
        print(f"B median residual={bm:+.3f} px")
        print(f"B-A residual difference={bm-am:+.3f} px")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- Если A1/A2/A3/A4 заметно дрейфуют при одной и той же позиции, dx нельзя считать стабильным position-specific offset.")
print("- Если adjacent B-A меняет знак/величину, position effect слабее гипотезы time/pipeline drift.")
print("- Если после detrend остаётся стабильный B-A residual, позиционная составляющая ещё возможна.")
print("- До этого plane-normal A/B comparison по текущей calibration остаётся заблокирован.")
