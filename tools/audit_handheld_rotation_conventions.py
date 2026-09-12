#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math
from pathlib import Path
import cv2, numpy as np

D_FLU_FRD=np.diag([1.0,-1.0,-1.0])

def rotvec(R):
    v,_=cv2.Rodrigues(R)
    return v.reshape(3)

def closest_rotation(H):
    U,_,Vt=np.linalg.svd(H)
    R=U@Vt
    if np.linalg.det(R)<0:
        U[:,-1]*=-1
        R=U@Vt
    return R

def angle_deg(R):
    return np.linalg.norm(rotvec(R))*180.0/math.pi

def yaml_matrix(path):
    fs=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
    if not fs.isOpened(): raise RuntimeError(f"cannot open {path}")
    n=fs.getNode("T_BS"); data=n.getNode("data")
    vals=np.array([data.at(i).real() for i in range(data.size())],float)
    rows=int(round(n.getNode("rows").real())); cols=int(round(n.getNode("cols").real()))
    fs.release()
    return vals.reshape(rows,cols)[:3,:3]

def stats(x):
    x=np.asarray(x,float)
    return float(np.median(x)),float(np.percentile(x,95)),float(np.sqrt(np.mean(x*x)))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--yaml",type=Path,default=Path("params/JTZeroMonoFLU/LeftCameraParams.yaml"))
    ap.add_argument("--max-rms",type=float,default=1.5)
    ap.add_argument("--min-corners",type=int,default=8)
    args=ap.parse_args()

    rows=list(csv.DictReader((args.run_dir/"charuco_poses.csv").open(newline="")))
    q=[]
    for r in rows:
        if float(r["reproj_rms_px"])>args.max_rms or int(float(r["charuco_corners"]))<args.min_corners: continue
        t=float(r["camera_ts_ns"])*1e-9
        R_Board_C=np.array([[float(r[f"R{i}{j}"]) for j in range(3)] for i in range(3)])
        R_W_C=R_Board_C.T
        gyro=np.array([float(r["xgyro"]),float(r["ygyro"]),float(r["zgyro"])])
        q.append((t,R_W_C,gyro))
    q.sort(key=lambda z:z[0])

    wb=[]; wc=[]
    for i in range(len(q)-1):
        dt=q[i+1][0]-q[i][0]
        if not (0<dt<0.20): continue
        dR=q[i][1].T@q[i+1][1]
        c=rotvec(dR)/dt
        b=0.5*(q[i][2]+q[i+1][2])
        if np.linalg.norm(b)<0.12 or np.linalg.norm(c)<0.12: continue
        if np.linalg.norm(b)>3.5 or np.linalg.norm(c)>3.5: continue
        wb.append(b);wc.append(c)
    wb=np.asarray(wb);wc=np.asarray(wc)
    if len(wb)<30: raise SystemExit(f"Недостаточно rotational pairs: {len(wb)}")

    H=np.zeros((3,3))
    for b,c in zip(wb,wc): H+=np.outer(c,b)
    Rfit=closest_rotation(H) # body(FRD gyro axes) -> camera

    Y=yaml_matrix(args.yaml)
    # Explicitly test the plausible conventions instead of assuming the YAML comment/parser semantics.
    candidates={
      "YAML interpreted B_FLU<-C, converted to B_FRD<-C": (D_FLU_FRD@Y).T,
      "YAML interpreted B_FRD<-C directly": Y.T,
      "YAML interpreted C<-B_FLU, converted from B_FRD": Y@D_FLU_FRD,
      "YAML interpreted C<-B_FRD directly": Y,
    }

    print("===== ROTATION CONVENTION FORENSIC =====")
    print(f"rotational pairs={len(wb)}")
    print("Model residuals compare measured camera angular velocity to transformed FC gyro.")
    print()
    rows_out=[]
    for name,R in candidates.items():
        for sign in [1.0,-1.0]:
            pred=(sign*(R@wb.T)).T
            e=np.linalg.norm(wc-pred,axis=1)
            med,p95,rms=stats(e)
            delta=angle_deg(Rfit@R.T)
            rows_out.append((rms,med,p95,delta,sign,name))
    rows_out.sort()
    for rms,med,p95,delta,sign,name in rows_out:
        print(f"{name}; sign={sign:+.0f}: med/p95/rms={med:.3f}/{p95:.3f}/{rms:.3f} rad/s ; delta_to_fit={delta:.2f} deg")

    fit_e=np.linalg.norm(wc-(Rfit@wb.T).T,axis=1)
    med,p95,rms=stats(fit_e)
    print()
    print(f"FREE FIT: med/p95/rms={med:.3f}/{p95:.3f}/{rms:.3f} rad/s")
    print("R_C_B fitted =")
    for row in Rfit: print("  "+" ".join(f"{v:+.6f}" for v in row))

    best=rows_out[0]
    print()
    print("BEST YAML CONVENTION:")
    print(f"  {best[5]}, gyro sign={best[4]:+.0f}")
    print(f"  residual RMS={best[0]:.3f} rad/s")
    print(f"  delta to free fit={best[3]:.2f} deg")
    if best[3] < 10 and best[4] > 0:
        print("ROTATION CONVENTION: CONSISTENT")
    else:
        print("ROTATION CONVENTION: MISMATCH — lever-arm XYZ frame/sign must not be accepted yet.")

if __name__=="__main__":main()
