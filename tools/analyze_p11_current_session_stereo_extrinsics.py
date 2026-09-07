#!/usr/bin/env python3
import sys,csv,cv2,math,statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
pairs_csv=run/"p11_stereo_pairs.csv"
cal=repo/"calibration"/"stereo_ov9281_ov5647_final.yaml"

print("================ P11 CURRENT-SESSION STEREO EXTRINSICS ================")
print("run:",run)
print("Цель: проверить, стабильны ли реальные OV9281↔OV5647 extrinsics в текущем run.")
print("Используются только A stages, где видна ChArUco-мишень.")
print("B не нужен для этого диагностического шага.")

fs=cv2.FileStorage(str(cal),cv2.FILE_STORAGE_READ)
K1=fs.getNode("K1").mat();D1=fs.getNode("D1").mat()
K2=fs.getNode("K2").mat();D2=fs.getNode("D2").mat()
Rcal=fs.getNode("R_ov9281_to_ov5647").mat()
Tcal=fs.getNode("T_ov9281_to_ov5647_m").mat().reshape(3)
fs.release()

dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board=cv2.aruco.CharucoBoard((7,5),0.027324,0.020043,dictionary)
obj_all=np.asarray(board.getChessboardCorners(),dtype=np.float64)

with pairs_csv.open(newline="") as f:
    rows=list(csv.DictReader(f))

def detect_pose(img,K,D):
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) if img.ndim==3 else img
    corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary)
    if ids is None or len(ids)==0:
        return None
    ret,cc,ci=cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
    if ci is None or cc is None or int(ret)<6:
        return None
    ids1=ci.reshape(-1).astype(int)
    imgpts=cc.reshape(-1,2).astype(np.float64)
    objpts=obj_all[ids1]
    ok,rvec,tvec=cv2.solvePnP(objpts,imgpts,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R,_=cv2.Rodrigues(rvec)
    # reprojection sanity
    proj,_=cv2.projectPoints(objpts,rvec,tvec,K,D)
    err=np.linalg.norm(proj.reshape(-1,2)-imgpts,axis=1)
    return dict(R=R,t=tvec.reshape(3),n=len(ids1),
                reproj_med=float(np.median(err)),
                reproj_p90=float(np.percentile(err,90)))

def rot_angle(R):
    c=np.clip((np.trace(R)-1.0)/2.0,-1.0,1.0)
    return math.degrees(math.acos(c))

records=[]
for row in rows:
    if not row["stage"].startswith("A"):
        continue
    L=cv2.imread(str(run/row["left_file"]),cv2.IMREAD_COLOR)
    R=cv2.imread(str(run/row["right_file"]),cv2.IMREAD_COLOR)
    if L is None or R is None:
        continue
    pL=detect_pose(L,K1,D1)
    pR=detect_pose(R,K2,D2)
    if pL is None or pR is None:
        records.append(dict(stage=row["stage"],ok=False))
        continue

    Rrel=pR["R"]@pL["R"].T
    Trel=pR["t"]-Rrel@pL["t"]

    Rerr=Rrel@Rcal.T
    ang_err=rot_angle(Rerr)
    trans_err_mm=np.linalg.norm(Trel-Tcal)*1000.0
    baseline=np.linalg.norm(Trel)*1000.0

    records.append(dict(
        stage=row["stage"],ok=True,
        left_n=pL["n"],right_n=pR["n"],
        left_rep=pL["reproj_med"],right_rep=pR["reproj_med"],
        Rrel=Rrel,Trel=Trel,ang_err=ang_err,
        trans_err_mm=trans_err_mm,baseline_mm=baseline
    ))

groups=defaultdict(list)
for r in records:
    groups[r["stage"]].append(r)

print("\n================ A-STAGE EXTRINSICS ================")
stage_summary={}
for st in ["A1","A2","A3","A4"]:
    xs=groups.get(st,[])
    ok=[x for x in xs if x.get("ok")]
    print(f"\n{st}: usable={len(ok)}/{len(xs)}")
    if not ok: continue
    print(f"  Charuco corners L/R median={statistics.median([x['left_n'] for x in ok]):.0f}/{statistics.median([x['right_n'] for x in ok]):.0f}")
    print(f"  PnP reproj median L/R={statistics.median([x['left_rep'] for x in ok]):.3f}/{statistics.median([x['right_rep'] for x in ok]):.3f} px")
    print(f"  rotation error vs saved calibration median/p90={statistics.median([x['ang_err'] for x in ok]):.3f}/{np.percentile([x['ang_err'] for x in ok],90):.3f} deg")
    print(f"  translation error vs saved calibration median/p90={statistics.median([x['trans_err_mm'] for x in ok]):.2f}/{np.percentile([x['trans_err_mm'] for x in ok],90):.2f} mm")
    print(f"  current baseline median={statistics.median([x['baseline_mm'] for x in ok]):.3f} mm")

    # robust mean transform: median translation, chordal mean rotation projected to SO(3)
    M=sum(x["Rrel"] for x in ok)/len(ok)
    U,_,Vt=np.linalg.svd(M)
    Rm=U@Vt
    if np.linalg.det(Rm)<0:
        U[:,-1]*=-1; Rm=U@Vt
    Tm=np.median(np.stack([x["Trel"] for x in ok]),axis=0)
    stage_summary[st]=(Rm,Tm)

print("\n================ SAME-POSITION EXTRINSIC DRIFT ================")
valid_st=[s for s in ["A1","A2","A3","A4"] if s in stage_summary]
for i in range(len(valid_st)-1):
    a,b=valid_st[i],valid_st[i+1]
    Ra,Ta=stage_summary[a];Rb,Tb=stage_summary[b]
    dang=rot_angle(Rb@Ra.T)
    dt=np.linalg.norm(Tb-Ta)*1000.0
    print(f"{a}->{b}: relative-extrinsic change rotation={dang:.3f} deg translation={dt:.2f} mm")

if len(valid_st)>=2:
    Rs=[stage_summary[s][0] for s in valid_st]
    Ts=[stage_summary[s][1] for s in valid_st]
    rot_span=0.0
    trans_span=0.0
    for i in range(len(Rs)):
        for j in range(i+1,len(Rs)):
            rot_span=max(rot_span,rot_angle(Rs[j]@Rs[i].T))
            trans_span=max(trans_span,np.linalg.norm(Ts[j]-Ts[i])*1000.0)
    print(f"max A-stage extrinsic span: rotation={rot_span:.3f} deg translation={trans_span:.2f} mm")

print("\nИНТЕРПРЕТАЦИЯ:")
print("- Если current extrinsics стабильны между A1/A2/A3/A4, но смещены относительно saved calibration, можно строить session-specific rectification.")
print("- Если extrinsics сами меняются между возвратами в A, stereo rig/pose estimation нестабилен и A/B plane-normal test блокируется.")
print("- PnP reprojection должен быть мал; плохой reprojection означает, что этот diagnostic сам ненадёжен.")
print("- Это проверка механики/калибровки stereo rig, а не P11 tilt result.")
