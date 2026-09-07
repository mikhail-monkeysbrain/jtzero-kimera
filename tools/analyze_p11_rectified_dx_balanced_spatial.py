#!/usr/bin/env python3
import sys,csv,cv2,statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
cal=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 RECTIFIED DX BALANCED-SPATIAL TEST ================")
print("run:",run)
print("Цель: убрать влияние того, что в разных stages matcher видит разные области кадра.")
print("Сравниваем dx только в одинаковых spatial bins, присутствующих и в A, и в B.")
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

with pairs_csv.open(newline="") as f:
    rows=list(csv.DictReader(f))

det=cv2.SIFT_create(nfeatures=2500,contrastThreshold=0.008,edgeThreshold=15)
X_CORRIDOR=8.0
DISP_MIN=220.0
DISP_MAX=380.0
RATIO=0.82
NX,NY=8,6

def guided(kL,dL,kR,dR):
    if dL is None or dR is None:return []
    rpts=np.array([k.pt for k in kR],dtype=np.float32)
    out=[];used=set()
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

stage_bins=defaultdict(lambda:defaultdict(list))

for row in rows:
    L0=cv2.imread(str(run/row["left_file"]),0)
    R0=cv2.imread(str(run/row["right_file"]),0)
    if L0 is None or R0 is None:continue
    L=cv2.remap(L0,m1x,m1y,cv2.INTER_LINEAR)
    R=cv2.remap(R0,m2x,m2y,cv2.INTER_LINEAR)
    kL,dL=det.detectAndCompute(L,None);kR,dR=det.detectAndCompute(R,None)
    gm=guided(kL,dL,kR,dR)
    st=row["stage"]
    for i,j in gm:
        xL,yL=kL[i].pt;xR,yR=kR[j].pt
        disp=yL-yR
        if not (DISP_MIN<=disp<=DISP_MAX):continue
        x=(xL+xR)*0.5;y=(yL+yR)*0.5
        bx=min(NX-1,max(0,int(x/(640/NX))))
        by=min(NY-1,max(0,int(y/(480/NY))))
        stage_bins[st][(bx,by)].append(float(xL-xR))

# stage-level median per bin, requiring enough support
stage_bin_med=defaultdict(dict)
for st,bins in stage_bins.items():
    for b,vals in bins.items():
        if len(vals)>=20:
            stage_bin_med[st][b]=float(np.median(vals))

A_stages=["A1","A2","A3","A4"]
B_stages=["B1","B2","B3"]

# bins supported in >=3 A stages and >=2 B stages
all_bins={(bx,by) for bx in range(NX) for by in range(NY)}
shared=[]
for b in all_bins:
    na=sum(b in stage_bin_med[s] for s in A_stages)
    nb=sum(b in stage_bin_med[s] for s in B_stages)
    if na>=3 and nb>=2:
        shared.append(b)

print(f"grid: {NX}x{NY}")
print(f"shared bins with sufficient A/B support: {len(shared)}")

print("\n================ SHARED-BIN STAGE MEDIANS ================")
balanced_stage={}
for st in A_stages+B_stages:
    vals=[stage_bin_med[st][b] for b in shared if b in stage_bin_med[st]]
    if vals:
        balanced_stage[st]=float(np.median(vals))
        print(f"{st}: bins={len(vals)} balanced median dx={np.median(vals):+.3f} px  spread={np.percentile(vals,10):+.3f}/{np.percentile(vals,90):+.3f}")
    else:
        print(f"{st}: no balanced data")

print("\n================ BALANCED ADJACENT A/B ================")
for a,b in [("A1","B1"),("A2","B2"),("A3","B3")]:
    if a in balanced_stage and b in balanced_stage:
        print(f"{a}->{b}: B-A={balanced_stage[b]-balanced_stage[a]:+.3f} px")

# Per-bin position effect using stage medians, then aggregate across bins.
bin_effects=[]
print("\n================ PER-BIN POSITION EFFECT ================")
for b in shared:
    av=[stage_bin_med[s][b] for s in A_stages if b in stage_bin_med[s]]
    bv=[stage_bin_med[s][b] for s in B_stages if b in stage_bin_med[s]]
    if len(av)>=3 and len(bv)>=2:
        eff=float(np.median(bv)-np.median(av))
        bin_effects.append(eff)

if bin_effects:
    print(f"bins used={len(bin_effects)}")
    print(f"B-A per-bin effect median={np.median(bin_effects):+.3f} px")
    print(f"B-A per-bin effect p10/p90={np.percentile(bin_effects,10):+.3f}/{np.percentile(bin_effects,90):+.3f} px")
    print(f"fraction same sign as median={np.mean(np.sign(bin_effects)==np.sign(np.median(bin_effects))):.3f}")

# Same-position stage variability within identical bins
A_drift=[];B_drift=[]
for b in shared:
    av=[stage_bin_med[s][b] for s in A_stages if b in stage_bin_med[s]]
    bv=[stage_bin_med[s][b] for s in B_stages if b in stage_bin_med[s]]
    if len(av)>=3:A_drift.append(max(av)-min(av))
    if len(bv)>=2:B_drift.append(max(bv)-min(bv))

print("\n================ SAME-POSITION BIN DRIFT ================")
if A_drift:
    print(f"A bin span median/p90={np.median(A_drift):.3f}/{np.percentile(A_drift,90):.3f} px")
if B_drift:
    print(f"B bin span median/p90={np.median(B_drift):.3f}/{np.percentile(B_drift,90):.3f} px")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- Если balanced B-A близок к нулю, прежняя A/B разница была в основном feature-location bias.")
print("- Если per-bin B-A стабилен по знаку и больше same-position bin drift, позиционный residual ещё реален.")
print("- Если same-position bin drift сопоставим или больше B-A, rectification bias остаётся нестабильным.")
print("- Plane-normal comparison остаётся заблокирован до этого решения.")
