#!/usr/bin/env python3
import sys, csv, cv2, statistics
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
calib=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 STEREO FORENSIC V2 ================")
print("run:",run)
print("Цель: выяснить, на каком именно фильтре теряются stereo-pairs.")
print("Никаких plane normals и A/B углов на этом шаге.")

fs=cv2.FileStorage(str(calib),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat(); D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat(); D2=fs.getNode("D2").mat()
R1=fs.getNode("R1").mat(); R2=fs.getNode("R2").mat()
P1=fs.getNode("P1").mat(); P2=fs.getNode("P2").mat()
fs.release()

size=(640,480)
m1x,m1y=cv2.initUndistortRectifyMap(K1,D1,R1,P1[:,:3],size,cv2.CV_32FC1)
m2x,m2y=cv2.initUndistortRectifyMap(K2,D2,R2,P2[:,:3],size,cv2.CV_32FC1)

rows=[]
with pairs_csv.open(newline="") as f:
    rows=list(csv.DictReader(f))

try:
    det=cv2.SIFT_create(nfeatures=2000,contrastThreshold=0.01,edgeThreshold=15)
    norm=cv2.NORM_L2
    name="SIFT"
except Exception:
    det=cv2.ORB_create(nfeatures=2200,fastThreshold=5)
    norm=cv2.NORM_HAMMING
    name="ORB"

bf=cv2.BFMatcher(norm,crossCheck=False)
print("matcher:",name)
print("pairs:",len(rows))

diag=run/"stereo_forensic_v2"
diag.mkdir(exist_ok=True)

stage_stats=defaultdict(lambda: {
    "kpL":[],"kpR":[],"knn":[],"ratio":[],"dy":[],"disp":[],"posdisp":[],"negdisp":[],
    "dy15":[],"dy25":[],"dy50":[],"both_pos":[],"both_neg":[],"reasons":Counter()
})

def analyze(row, save_overlay=False):
    l0=cv2.imread(str(run/row["left_file"]),cv2.IMREAD_GRAYSCALE)
    r0=cv2.imread(str(run/row["right_file"]),cv2.IMREAD_GRAYSCALE)
    if l0 is None or r0 is None:
        return {"reason":"read"}

    l=cv2.remap(l0,m1x,m1y,cv2.INTER_LINEAR)
    r=cv2.remap(r0,m2x,m2y,cv2.INTER_LINEAR)

    k1,d1=det.detectAndCompute(l,None)
    k2,d2=det.detectAndCompute(r,None)
    if d1 is None or d2 is None:
        return {"reason":"descriptor","kpL":len(k1),"kpR":len(k2),"l":l,"r":r}

    knn=bf.knnMatch(d1,d2,k=2)
    ratio=[]
    for ms in knn:
        if len(ms)<2: continue
        a,b=ms
        if a.distance < 0.80*b.distance:
            ratio.append(a)

    rec={"reason":"ok","kpL":len(k1),"kpR":len(k2),"knn":len(knn),"ratio":len(ratio),"l":l,"r":r}
    if not ratio:
        rec["reason"]="ratio"
        return rec

    dys=[]; disps=[]; pairs=[]
    for m in ratio:
        p=np.array(k1[m.queryIdx].pt)
        q=np.array(k2[m.trainIdx].pt)
        dys.append(abs(float(p[1]-q[1])))
        disps.append(float(p[0]-q[0]))
        pairs.append((m,p,q))

    rec["dys"]=dys; rec["disps"]=disps; rec["pairs"]=pairs

    if save_overlay:
        # side-by-side rectified diagnostic with ratio-test matches only
        vis=cv2.drawMatches(l,k1,r,k2,ratio[:100],None,
                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        for y in range(60,480,80):
            cv2.line(vis,(0,y),(1280,y),(255,255,255),1)
        cv2.putText(vis,
                    f"{row['stage']} ratio={len(ratio)} dy_med={np.median(dys):.1f} disp_med={np.median(disps):.1f}",
                    (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,0,255),2)
        cv2.imwrite(str(diag/f"{row['stage']}_rectified_matches.png"),vis)

        # plain rectified side-by-side
        cat=cv2.hconcat([l,r])
        cv2.imwrite(str(diag/f"{row['stage']}_rectified_pair.png"),cat)

    return rec

representative_done=set()

for row in rows:
    st=row["stage"]
    save=False
    if st not in representative_done and int(row["pair_index"])==10:
        save=True; representative_done.add(st)
    rec=analyze(row,save)
    S=stage_stats[st]
    S["reasons"][rec["reason"]]+=1
    for k in ("kpL","kpR","knn","ratio"):
        if k in rec: S[k].append(rec[k])
    if rec.get("reason")!="ok": continue
    dys=np.array(rec["dys"]); disps=np.array(rec["disps"])
    S["dy"].extend(dys.tolist())
    S["disp"].extend(disps.tolist())
    S["posdisp"].append(int(np.sum(disps>0)))
    S["negdisp"].append(int(np.sum(disps<0)))
    S["dy15"].append(int(np.sum(dys<=1.5)))
    S["dy25"].append(int(np.sum(dys<=2.5)))
    S["dy50"].append(int(np.sum(dys<=5.0)))
    S["both_pos"].append(int(np.sum((dys<=2.5)&(disps>=8)&(disps<=620))))
    S["both_neg"].append(int(np.sum((dys<=2.5)&(disps<=-8)&(disps>=-620))))

print("\n================ STAGE FORENSIC ================")
for st in ["A1","B1","A2","B2","A3","B3","A4"]:
    S=stage_stats[st]
    print(f"\n{st}: reasons={dict(S['reasons'])}")
    if S["kpL"]:
        print(f"  keypoints L/R median={statistics.median(S['kpL']):.0f}/{statistics.median(S['kpR']):.0f}")
    if S["ratio"]:
        print(f"  ratio-test matches median={statistics.median(S['ratio']):.0f}")
    if S["dy"]:
        print(f"  |dy| median/p90={statistics.median(S['dy']):.2f}/{np.percentile(S['dy'],90):.2f} px")
        print(f"  signed disparity median/p10/p90={statistics.median(S['disp']):+.2f}/{np.percentile(S['disp'],10):+.2f}/{np.percentile(S['disp'],90):+.2f} px")
        print(f"  count |dy|<=1.5/2.5/5.0 median={statistics.median(S['dy15']):.0f}/{statistics.median(S['dy25']):.0f}/{statistics.median(S['dy50']):.0f}")
        print(f"  count dy<=2.5 & positive disp median={statistics.median(S['both_pos']):.0f}")
        print(f"  count dy<=2.5 & negative disp median={statistics.median(S['both_neg']):.0f}")

print("\n================ GLOBAL DIAGNOSIS ================")
all_dy=[]; all_disp=[]
for st,S in stage_stats.items():
    all_dy.extend(S["dy"]); all_disp.extend(S["disp"])
if all_dy:
    print(f"all ratio matches: {len(all_dy)}")
    print(f"global |dy| median={np.median(all_dy):.2f} px")
    print(f"global signed disparity median={np.median(all_disp):+.2f} px")
    pos=sum(1 for d,y in zip(all_disp,all_dy) if y<=2.5 and d>=8)
    neg=sum(1 for d,y in zip(all_disp,all_dy) if y<=2.5 and d<=-8)
    print(f"global filtered positive-disparity matches={pos}")
    print(f"global filtered negative-disparity matches={neg}")

print("\nДиагностические изображения:",diag)
print("\nИНТЕРПРЕТАЦИЯ:")
print("- Если keypoints/ratio matches высокие, но |dy| огромный -> проблема rectification/calibration/order.")
print("- Если |dy| малый, но disparity в основном отрицательный -> v1 использовал неверный знак disparity.")
print("- Если matches мало уже до epipolar filter -> photometric/FOV overlap problem.")
print("- Новый физический прогон НЕ нужен до выяснения этого результата.")
