#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run = Path(sys.argv[1])
repo = Path(__file__).resolve().parents[1]
pairs_csv = run / "p11_stereo_pairs.csv"
calib_path = repo / "calibration" / "stereo_ov9281_ov5647_final.yaml"

print("================ P11 STEREO OBSERVABILITY GATE ================")
print("run:", run)
print("Цель: проверить качество stereo-геометрии отдельно в A и B.")
print("На этом шаге угол A/B НЕ вычисляется.")

if not pairs_csv.exists():
    raise SystemExit("FAIL: p11_stereo_pairs.csv not found")

fs = cv2.FileStorage(str(calib_path), cv2.FILE_STORAGE_READ)
K1 = fs.getNode("K1").mat()
D1 = fs.getNode("D1").mat()
K2 = fs.getNode("K2").mat()
D2 = fs.getNode("D2").mat()
R1 = fs.getNode("R1").mat()
R2 = fs.getNode("R2").mat()
P1 = fs.getNode("P1").mat()
P2 = fs.getNode("P2").mat()
baseline = float(fs.getNode("baseline_m").real())
fs.release()

if any(x is None for x in (K1,D1,K2,D2,R1,R2,P1,P2)):
    raise SystemExit("FAIL: stereo calibration incomplete")

size = (640,480)
map1x,map1y = cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
map2x,map2y = cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

rows=[]
with pairs_csv.open(newline="") as f:
    for r in csv.DictReader(f):
        rows.append(r)

print(f"pairs in CSV: {len(rows)}")
print(f"baseline: {baseline*1000:.3f} mm")

try:
    detector = cv2.SIFT_create(nfeatures=1600, contrastThreshold=0.02, edgeThreshold=12)
    norm_type = cv2.NORM_L2
    detector_name = "SIFT"
except Exception:
    detector = cv2.ORB_create(nfeatures=1800, fastThreshold=8)
    norm_type = cv2.NORM_HAMMING
    detector_name = "ORB"

matcher = cv2.BFMatcher(norm_type, crossCheck=False)
print("feature matcher:", detector_name)

RANSAC_PLANE_THRESH_M = 0.008
EPI_Y_MAX_PX = 2.5
MIN_DISP_PX = 8.0
MAX_DISP_PX = 620.0

def fit_plane_ransac(pts, seed):
    if len(pts) < 6:
        return None
    rng = np.random.default_rng(seed)
    best = None
    n = len(pts)
    for _ in range(500):
        ids = rng.choice(n,3,replace=False)
        a,b,c = pts[ids]
        normal = np.cross(b-a,c-a)
        nn = np.linalg.norm(normal)
        if nn < 1e-9:
            continue
        normal = normal/nn
        d = -float(np.dot(normal,a))
        dist = np.abs(pts@normal + d)
        mask = dist < RANSAC_PLANE_THRESH_M
        count = int(mask.sum())
        med = float(np.median(dist[mask])) if count else 999.0
        key = (count,-med)
        if best is None or key > best[0]:
            best = (key,mask)
    if best is None:
        return None

    mask = best[1]
    inpts = pts[mask]
    centroid = inpts.mean(axis=0)
    _,_,vt = np.linalg.svd(inpts-centroid,full_matrices=False)
    normal = vt[-1]
    normal = normal/np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    dist = np.abs((pts-centroid)@normal)
    mask = dist < RANSAC_PLANE_THRESH_M
    in_dist = dist[mask]
    return {
        "normal": normal,
        "centroid": centroid,
        "mask": mask,
        "inliers": int(mask.sum()),
        "ratio": float(mask.mean()),
        "res_med": float(np.median(in_dist)) if len(in_dist) else 999.0,
        "res_p90": float(np.percentile(in_dist,90)) if len(in_dist) else 999.0,
    }

def analyze_pair(left_path,right_path,seed):
    left = cv2.imread(str(left_path),cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(right_path),cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        return None

    l = cv2.remap(left,map1x,map1y,cv2.INTER_LINEAR)
    r = cv2.remap(right,map2x,map2y,cv2.INTER_LINEAR)

    k1,d1 = detector.detectAndCompute(l,None)
    k2,d2 = detector.detectAndCompute(r,None)
    if d1 is None or d2 is None or len(k1)<8 or len(k2)<8:
        return {"left":l,"right":r,"reason":"features"}

    knn = matcher.knnMatch(d1,d2,k=2)
    good=[]
    for mset in knn:
        if len(mset)<2:
            continue
        m,n = mset
        if m.distance < 0.72*n.distance:
            p = k1[m.queryIdx].pt
            q = k2[m.trainIdx].pt
            dy = abs(p[1]-q[1])
            disp = p[0]-q[0]
            if dy <= EPI_Y_MAX_PX and MIN_DISP_PX <= disp <= MAX_DISP_PX:
                good.append((m,p,q,dy,disp))

    if len(good)<6:
        return {
            "left":l,"right":r,"k1":k1,"k2":k2,"good":good,
            "reason":"matches"
        }

    pL=np.array([g[1] for g in good],dtype=np.float64)
    pR=np.array([g[2] for g in good],dtype=np.float64)

    Xh=cv2.triangulatePoints(P1,P2,pL.T,pR.T)
    w=Xh[3]
    valid=np.abs(w)>1e-9
    X=np.empty((len(good),3),dtype=np.float64)
    X[:] = np.nan
    X[valid]= (Xh[:3,valid]/w[valid]).T

    finite=np.isfinite(X).all(axis=1)
    z=X[:,2]
    finite &= (z>0.05)&(z<3.0)

    Xv=X[finite]
    goodv=[g for g,v in zip(good,finite) if v]

    if len(Xv)<6:
        return {
            "left":l,"right":r,"k1":k1,"k2":k2,"good":goodv,
            "reason":"triangulation"
        }

    plane=fit_plane_ransac(Xv,seed)
    if plane is None:
        return {
            "left":l,"right":r,"k1":k1,"k2":k2,"good":goodv,
            "reason":"plane"
        }

    epi=[g[3] for g in goodv]
    disp=[g[4] for g in goodv]
    depth=Xv[:,2]

    return {
        "left":l,"right":r,"k1":k1,"k2":k2,"good":goodv,
        "pts":Xv,"plane":plane,
        "matches":len(goodv),
        "epi_med":float(np.median(epi)),
        "epi_p90":float(np.percentile(epi,90)),
        "disp_med":float(np.median(disp)),
        "depth_med":float(np.median(depth)),
        "reason":"ok"
    }

results=[]
stage_groups=defaultdict(list)

diag=run/"stereo_observability"
diag.mkdir(exist_ok=True)

for i,row in enumerate(rows):
    lp=run/row["left_file"]
    rp=run/row["right_file"]
    res=analyze_pair(lp,rp,1000+i)
    rec={
        "stage":row["stage"],
        "position":row["position"],
        "pair_index":int(row["pair_index"]),
        "dt_ms":float(row["dt_ms"]),
        "res":res
    }
    results.append(rec)
    stage_groups[row["stage"]].append(rec)

print("\n================ STAGE QUALITY ================")

stage_summary={}
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    xs=stage_groups.get(st,[])
    ok=[x for x in xs if x["res"] is not None and x["res"].get("reason")=="ok"]
    print(f"\n{st}: pairs={len(xs)} usable={len(ok)}")
    if not ok:
        print("  FAIL: нет пригодных stereo-пар")
        continue

    matches=[x["res"]["matches"] for x in ok]
    epi=[x["res"]["epi_med"] for x in ok]
    depth=[x["res"]["depth_med"] for x in ok]
    pin=[x["res"]["plane"]["inliers"] for x in ok]
    pratio=[x["res"]["plane"]["ratio"] for x in ok]
    pres=[x["res"]["plane"]["res_med"] for x in ok]
    p90=[x["res"]["plane"]["res_p90"] for x in ok]

    print(f"  matches median/p10={statistics.median(matches):.0f}/{np.percentile(matches,10):.0f}")
    print(f"  epipolar |dy| median-of-pairs={statistics.median(epi):.3f} px")
    print(f"  depth median-of-pairs={statistics.median(depth):.3f} m")
    print(f"  plane inliers median={statistics.median(pin):.0f}")
    print(f"  plane inlier ratio median/p10={statistics.median(pratio):.3f}/{np.percentile(pratio,10):.3f}")
    print(f"  plane residual median={statistics.median(pres)*1000:.2f} mm")
    print(f"  plane residual p90 median={statistics.median(p90)*1000:.2f} mm")

    stage_summary[st]={
        "usable":len(ok),"total":len(xs),
        "matches_med":statistics.median(matches),
        "ratio_med":statistics.median(pratio),
        "ratio_p10":float(np.percentile(pratio,10)),
        "res_med":statistics.median(pres),
        "epi_med":statistics.median(epi),
        "depth_med":statistics.median(depth),
    }

    # Representative diagnostic image.
    rep=ok[len(ok)//2]
    rr=rep["res"]
    canvas=cv2.cvtColor(rr["left"],cv2.COLOR_GRAY2BGR)
    pinmask=rr["plane"]["mask"]
    for j,(g,isin) in enumerate(zip(rr["good"],pinmask)):
        p=tuple(np.int32(g[1]))
        cv2.circle(canvas,p,3,(0,255,0) if isin else (0,0,255),-1)
    cv2.putText(canvas,
        f"{st} matches={rr['matches']} plane={rr['plane']['inliers']}",
        (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
    cv2.imwrite(str(diag/f"{st}.png"),canvas)

# Conservative observability gate only.
def stage_pass(s):
    if s not in stage_summary:
        return False
    x=stage_summary[s]
    return (x["usable"] >= 15 and
            x["matches_med"] >= 20 and
            x["ratio_p10"] >= 0.45 and
            x["res_med"] <= 0.006 and
            x["epi_med"] <= 1.5)

passes={st:stage_pass(st) for st in ["A1","B1","A2","B2","A3","B3","A4"]}

print("\n================ STAGE GATE ================")
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    print(f"{st}: {'PASS' if passes[st] else 'FAIL'}")

overall=all(passes.values())
print("\n================ OVERALL GATE ================")
print("PASS" if overall else "FAIL")

print(f"Annotated diagnostics: {diag}")
print("\nИНТЕРПРЕТАЦИЯ:")
print("- PASS означает только: stereo depth/plane-fit достаточно наблюдаемы в каждом stage.")
print("- На этом шаге нормали A и B намеренно НЕ сравниваются.")
print("- Если хотя бы один stage FAIL, причинный A/B normal test не проводится.")
print("- Даже при PASS следующий шаг должен сначала проверить repeatability normals внутри A и внутри B.")
