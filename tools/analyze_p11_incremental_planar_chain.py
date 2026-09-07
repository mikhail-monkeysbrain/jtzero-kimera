#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
import numpy as np

run = Path(sys.argv[1])
video_path = run / "ov9281_ruler_pass.avi"
frames_csv = run / "p11_ruler_pass_frames.csv"

print("================ P11 INCREMENTAL PLANAR CHAIN GATE ================")
print("run:", run)
print("Цель: проверить, можно ли непрерывно передавать геометрию кадр->кадр")
print("от A до B по конкретным KLT-точкам, а не по абстрактным углам линий.")
print("Пока это только gate качества цепочки, НЕ физический tilt.")

rows=[]
with frames_csv.open(newline="") as f:
    for r in csv.DictReader(f):
        rows.append({
            "frame_id": int(r["frame_id"]),
            "steady_ns": int(r["steady_ns"]),
            "state": r["state"],
        })

cap=cv2.VideoCapture(str(video_path))
if not cap.isOpened():
    raise SystemExit("FAIL: cannot open video")

video_frames=[]
while True:
    ok,im=cap.read()
    if not ok:
        break
    video_frames.append(im)
cap.release()

n=min(len(rows),len(video_frames))
rows=rows[:n]
video_frames=video_frames[:n]

if n < 10:
    raise SystemExit("FAIL: too few frames")

# Work at ~25 Hz using real timestamps, preserving adjacent overlap.
sel=[0]
last_ns=rows[0]["steady_ns"]
for i in range(1,n):
    if rows[i]["steady_ns"] - last_ns >= 35_000_000:
        sel.append(i)
        last_ns=rows[i]["steady_ns"]
if sel[-1] != n-1:
    sel.append(n-1)

print(f"video/csv aligned frames: {n}")
print(f"selected for chain: {len(sel)}")

def prep(im):
    g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    return cv2.createCLAHE(2.0,(8,8)).apply(g)

def pair_homography(im1, im2):
    g1=prep(im1)
    g2=prep(im2)
    p0=cv2.goodFeaturesToTrack(
        g1,
        maxCorners=700,
        qualityLevel=0.01,
        minDistance=6,
        blockSize=7,
        useHarrisDetector=False)
    if p0 is None or len(p0)<20:
        return None

    p1,st1,err1=cv2.calcOpticalFlowPyrLK(
        g1,g2,p0,None,
        winSize=(21,21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))

    p0b,st2,err2=cv2.calcOpticalFlowPyrLK(
        g2,g1,p1,None,
        winSize=(21,21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))

    if p1 is None or p0b is None:
        return None

    p0r=p0.reshape(-1,2)
    p1r=p1.reshape(-1,2)
    p0br=p0b.reshape(-1,2)
    good=(st1.ravel()==1)&(st2.ravel()==1)
    fb=np.linalg.norm(p0r-p0br,axis=1)
    good &= (fb < 1.0)

    a=p0r[good]
    b=p1r[good]

    if len(a)<12:
        return None

    H,mask=cv2.findHomography(a,b,cv2.RANSAC,2.0)
    if H is None or mask is None:
        return None

    inl=mask.ravel().astype(bool)
    ai=a[inl]
    bi=b[inl]
    if len(ai)<8:
        return None

    proj=cv2.perspectiveTransform(ai.reshape(-1,1,2),H).reshape(-1,2)
    reproj=np.linalg.norm(proj-bi,axis=1)

    return {
        "H": H/H[2,2],
        "tracks": len(a),
        "inliers": len(ai),
        "ratio": len(ai)/len(a),
        "reproj_med": float(np.median(reproj)),
        "reproj_p90": float(np.percentile(reproj,90)),
        "pts0": ai,
        "pts1": bi,
    }

records=[]
Hcum=np.eye(3,dtype=np.float64)
valid_pairs=0
fail_pairs=0

diag=run/"incremental_planar_chain"
diag.mkdir(exist_ok=True)

snapshots=[]

for j in range(1,len(sel)):
    i0=sel[j-1]
    i1=sel[j]
    r=pair_homography(video_frames[i0],video_frames[i1])
    state=rows[i1]["state"]

    if r is None:
        records.append({
            "i0":i0,"i1":i1,"state":state,"ok":False
        })
        fail_pairs+=1
        continue

    Hcum=r["H"] @ Hcum
    Hcum=Hcum/Hcum[2,2]
    valid_pairs+=1

    rec={
        "i0":i0,"i1":i1,"state":state,"ok":True,
        "tracks":r["tracks"],"inliers":r["inliers"],
        "ratio":r["ratio"],"reproj_med":r["reproj_med"],
        "reproj_p90":r["reproj_p90"],"Hcum":Hcum.copy()
    }
    records.append(rec)

    if j in (1,len(sel)//2,len(sel)-1):
        vis=video_frames[i1].copy()
        for p,q in zip(r["pts0"][:120],r["pts1"][:120]):
            cv2.circle(vis,tuple(np.int32(q)),2,(0,255,0),-1)
        cv2.putText(vis,f"{state} inl={r['inliers']} ratio={r['ratio']:.2f}",
                    (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
        snapshots.append(cv2.resize(vis,(426,320)))

if snapshots:
    sheet=cv2.hconcat(snapshots)
    cv2.imwrite(str(diag/"contact_sheet.png"),sheet)

oks=[r for r in records if r.get("ok")]
print("\n================ PAIR QUALITY ================")
print(f"valid pairs={valid_pairs}/{len(records)} fail={fail_pairs}")

if oks:
    ratios=[r["ratio"] for r in oks]
    inliers=[r["inliers"] for r in oks]
    mederr=[r["reproj_med"] for r in oks]
    p90err=[r["reproj_p90"] for r in oks]

    print(f"inliers median/p10={statistics.median(inliers):.0f}/{np.percentile(inliers,10):.0f}")
    print(f"inlier ratio median/p10={statistics.median(ratios):.3f}/{np.percentile(ratios,10):.3f}")
    print(f"reproj median median/p90={statistics.median(mederr):.3f}/{np.percentile(mederr,90):.3f} px")
    print(f"reproj p90 median/p90={statistics.median(p90err):.3f}/{np.percentile(p90err,90):.3f} px")

print("\n================ STATE QUALITY ================")
for st in ("PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"):
    xs=[r for r in oks if r["state"]==st]
    total=sum(1 for r in records if r["state"]==st)
    print(f"{st}: valid={len(xs)}/{total}")
    if xs:
        print(f"  inliers median={statistics.median([r['inliers'] for r in xs]):.0f}")
        print(f"  ratio median={statistics.median([r['ratio'] for r in xs]):.3f}")
        print(f"  reproj median={statistics.median([r['reproj_med'] for r in xs]):.3f} px")

# Chain health: look for implausible jumps in cumulative mapped image center.
center=np.array([[[320.0,240.0]]],dtype=np.float32)
centers=[]
persp=[]
for r in oks:
    H=r["Hcum"]
    c=cv2.perspectiveTransform(center,H).reshape(2)
    centers.append(c)
    persp.append(math.hypot(H[2,0],H[2,1]))

jump=[]
for a,b in zip(centers[:-1],centers[1:]):
    jump.append(float(np.linalg.norm(b-a)))

print("\n================ CHAIN HEALTH ================")
if jump:
    print(f"cumulative-center step median/p90/max={statistics.median(jump):.2f}/{np.percentile(jump,90):.2f}/{max(jump):.2f} px")
if persp:
    print(f"cumulative perspective-term final={persp[-1]:.6e} max={max(persp):.6e}")

# Conservative pass criterion.
pair_fraction=valid_pairs/max(1,len(records))
ratio_p10=float(np.percentile([r["ratio"] for r in oks],10)) if oks else 0.0
inlier_p10=float(np.percentile([r["inliers"] for r in oks],10)) if oks else 0.0
err_p90=float(np.percentile([r["reproj_med"] for r in oks],90)) if oks else 999.0

passed=(pair_fraction>=0.95 and ratio_p10>=0.60 and inlier_p10>=25 and err_p90<=1.0)

print("\n================ GATE ================")
print("PASS" if passed else "FAIL")

if passed:
    print("Frame-to-frame planar correspondence chain is strong enough for a next-stage calibrated motion decomposition.")
else:
    print("Chain quality is not strong enough; do NOT infer camera orientation from accumulated homographies.")

print(f"Annotated diagnostics: {diag/'contact_sheet.png'}")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- PASS означает, что соседние кадры хорошо связываются конкретными KLT-точками и RANSAC-homography.")
print("- Это сильнее, чем tracking только направлений линий.")
print("- Но накопленная homography всё ещё может дрейфовать; PASS не равен физическому углу.")
print("- При PASS следующий шаг — calibrated incremental homography decomposition с контролем согласованности.")
