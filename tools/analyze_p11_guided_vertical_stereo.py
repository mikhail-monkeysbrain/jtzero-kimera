#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
calib=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 GUIDED VERTICAL STEREO GATE ================")
print("run:",run)
print("Метод: rectification + epipolar-guided SIFT matching + triangulation + plane RANSAC.")
print("На этом шаге A/B normal difference всё ещё НЕ вычисляется.")

fs=cv2.FileStorage(str(calib),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat(); D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat(); D2=fs.getNode("D2").mat()
R1=fs.getNode("R1").mat(); R2=fs.getNode("R2").mat()
P1=fs.getNode("P1").mat(); P2=fs.getNode("P2").mat()
baseline=float(fs.getNode("baseline_m").real())
fs.release()

size=(640,480)
m1x,m1y=cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
m2x,m2y=cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

with pairs_csv.open(newline="") as f:
    rows=list(csv.DictReader(f))

det=cv2.SIFT_create(nfeatures=2500,contrastThreshold=0.008,edgeThreshold=15)

# Broad physical corridor around the observed ~303 px disparity.
X_CORRIDOR=6.0
DISP_MIN=220.0
DISP_MAX=380.0
RATIO=0.82
PLANE_THRESH=0.008

print(f"baseline={baseline*1000:.3f} mm")
print(f"guided corridor: |dx|<={X_CORRIDOR:.1f} px, yL-yR in [{DISP_MIN:.0f},{DISP_MAX:.0f}] px")

def fit_plane(pts,seed):
    if len(pts)<8:return None
    rng=np.random.default_rng(seed)
    best=None
    for _ in range(600):
        ids=rng.choice(len(pts),3,replace=False)
        a,b,c=pts[ids]
        n=np.cross(b-a,c-a); nn=np.linalg.norm(n)
        if nn<1e-9:continue
        n=n/nn; d=-float(np.dot(n,a))
        dist=np.abs(pts@n+d)
        mask=dist<PLANE_THRESH
        cnt=int(mask.sum())
        med=float(np.median(dist[mask])) if cnt else 999.
        key=(cnt,-med)
        if best is None or key>best[0]:
            best=(key,mask)
    if best is None:return None
    mask=best[1]
    q=pts[mask]
    c=q.mean(axis=0)
    _,_,vt=np.linalg.svd(q-c,full_matrices=False)
    n=vt[-1]; n=n/np.linalg.norm(n)
    if n[2]<0:n=-n
    dist=np.abs((pts-c)@n)
    mask=dist<PLANE_THRESH
    di=dist[mask]
    return dict(normal=n,centroid=c,mask=mask,inliers=int(mask.sum()),
                ratio=float(mask.mean()),
                res_med=float(np.median(di)) if len(di) else 999.,
                res_p90=float(np.percentile(di,90)) if len(di) else 999.)

def guided_matches(kL,dL,kR,dR):
    if dL is None or dR is None:return []
    out=[]
    usedR=set()
    # Build right features sorted by x for simple corridor prefilter.
    rpts=np.array([k.pt for k in kR],dtype=np.float32)
    for i,k in enumerate(kL):
        xL,yL=k.pt
        mask=(np.abs(rpts[:,0]-xL)<=X_CORRIDOR) & ((yL-rpts[:,1])>=DISP_MIN) & ((yL-rpts[:,1])<=DISP_MAX)
        inds=np.where(mask)[0]
        if len(inds)<2:continue
        desc=dR[inds]
        diff=desc-dL[i]
        ds=np.sqrt(np.sum(diff*diff,axis=1))
        order=np.argsort(ds)
        j1,j2=order[0],order[1]
        if ds[j1] >= RATIO*ds[j2]:
            continue
        rid=int(inds[j1])
        if rid in usedR:
            continue
        # Mutual uniqueness inside the same geometric corridor.
        xr,yr=kR[rid].pt
        lpts=np.array([kk.pt for kk in kL],dtype=np.float32)
        lmask=(np.abs(lpts[:,0]-xr)<=X_CORRIDOR) & ((lpts[:,1]-yr)>=DISP_MIN) & ((lpts[:,1]-yr)<=DISP_MAX)
        linds=np.where(lmask)[0]
        if len(linds)<1:continue
        ld=dL[linds]
        dd=ld-dR[rid]
        dback=np.sqrt(np.sum(dd*dd,axis=1))
        best_back=int(linds[int(np.argmin(dback))])
        if best_back!=i:
            continue
        usedR.add(rid)
        out.append((i,rid,float(ds[j1])))
    return out

def analyze(row,seed):
    L0=cv2.imread(str(run/row["left_file"]),0)
    R0=cv2.imread(str(run/row["right_file"]),0)
    if L0 is None or R0 is None:return None
    L=cv2.remap(L0,m1x,m1y,cv2.INTER_LINEAR)
    R=cv2.remap(R0,m2x,m2y,cv2.INTER_LINEAR)
    kL,dL=det.detectAndCompute(L,None); kR,dR=det.detectAndCompute(R,None)
    gm=guided_matches(kL,dL,kR,dR)
    if len(gm)<8:
        return dict(reason="guided_matches",L=L,R=R,kL=kL,kR=kR,gm=gm)
    pL=np.array([kL[i].pt for i,j,d in gm],dtype=np.float64)
    pR=np.array([kR[j].pt for i,j,d in gm],dtype=np.float64)
    Xh=cv2.triangulatePoints(P1,P2,pL.T,pR.T)
    ok=np.abs(Xh[3])>1e-9
    X=np.full((len(gm),3),np.nan)
    X[ok]=(Xh[:3,ok]/Xh[3,ok]).T
    finite=np.isfinite(X).all(axis=1)
    finite &= (X[:,2]>0.05)&(X[:,2]<0.50)
    Xv=X[finite]
    gmv=[g for g,v in zip(gm,finite) if v]
    if len(Xv)<8:
        return dict(reason="positive_depth",L=L,R=R,kL=kL,kR=kR,gm=gm,
                    positive_depth=len(Xv))
    plane=fit_plane(Xv,seed)
    if plane is None:
        return dict(reason="plane",L=L,R=R,kL=kL,kR=kR,gm=gm,
                    positive_depth=len(Xv))
    dx=np.abs(pL[finite,0]-pR[finite,0])
    disp=pL[finite,1]-pR[finite,1]
    return dict(reason="ok",L=L,R=R,kL=kL,kR=kR,gm=gm,gmv=gmv,
                pts=Xv,plane=plane,matches=len(Xv),
                dx_med=float(np.median(dx)),dx_p90=float(np.percentile(dx,90)),
                disp_med=float(np.median(disp)),
                depth_med=float(np.median(Xv[:,2])))

groups=defaultdict(list)
diag=run/"stereo_guided_gate"
diag.mkdir(exist_ok=True)

for i,row in enumerate(rows):
    r=analyze(row,10000+i)
    groups[row["stage"]].append((row,r))

print("\n================ STAGE QUALITY ================")
summary={}
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    xs=groups[st]
    ok=[r for row,r in xs if r and r.get("reason")=="ok"]
    print(f"\n{st}: pairs={len(xs)} usable={len(ok)}")
    reasons=defaultdict(int)
    for row,r in xs:
        reasons["none" if r is None else r.get("reason","?")]+=1
    print("  reasons:",dict(reasons))
    if not ok:continue
    matches=[r["matches"] for r in ok]
    ratios=[r["plane"]["ratio"] for r in ok]
    pres=[r["plane"]["res_med"] for r in ok]
    dx=[r["dx_med"] for r in ok]
    dep=[r["depth_med"] for r in ok]
    print(f"  guided matches median/p10={statistics.median(matches):.0f}/{np.percentile(matches,10):.0f}")
    print(f"  |dx| median-of-pairs={statistics.median(dx):.3f} px")
    print(f"  depth median-of-pairs={statistics.median(dep):.3f} m")
    print(f"  plane inlier ratio median/p10={statistics.median(ratios):.3f}/{np.percentile(ratios,10):.3f}")
    print(f"  plane residual median={statistics.median(pres)*1000:.2f} mm")
    summary[st]=dict(usable=len(ok),matches_med=statistics.median(matches),
                     ratio_p10=float(np.percentile(ratios,10)),
                     res_med=statistics.median(pres),
                     dx_med=statistics.median(dx))
    # Representative overlay
    r=ok[len(ok)//2]
    vis=cv2.cvtColor(r["L"],cv2.COLOR_GRAY2BGR)
    mask=r["plane"]["mask"]
    for (g,isin) in zip(r["gmv"],mask):
        p=tuple(np.int32(r["kL"][g[0]].pt))
        cv2.circle(vis,p,3,(0,255,0) if isin else (0,0,255),-1)
    cv2.putText(vis,f"{st} guided={r['matches']} plane={r['plane']['inliers']}",
                (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
    cv2.imwrite(str(diag/f"{st}.png"),vis)

def pass_stage(st):
    if st not in summary:return False
    x=summary[st]
    return (x["usable"]>=15 and x["matches_med"]>=15 and
            x["ratio_p10"]>=0.45 and x["res_med"]<=0.006 and
            x["dx_med"]<=3.0)

passes={st:pass_stage(st) for st in ["A1","B1","A2","B2","A3","B3","A4"]}

print("\n================ STAGE GATE ================")
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    print(f"{st}: {'PASS' if passes[st] else 'FAIL'}")
print("\n================ OVERALL GATE ================")
print("PASS" if all(passes.values()) else "FAIL")
print("Diagnostics:",diag)
print("\nИНТЕРПРЕТАЦИЯ:")
print("- PASS означает: guided stereo correspondences и 3-D plane fit достаточно стабильны во всех stages.")
print("- Это ещё НЕ A/B tilt result.")
print("- При PASS следующий шаг: repeatability normals внутри A и B, затем pairwise A/B comparison.")
print("- При FAIL новый физический run не нужен автоматически: сначала смотрим reasons/diagnostics.")
