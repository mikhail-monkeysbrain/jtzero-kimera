#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path
import cv2, numpy as np

D_FRD_TO_FLU=np.diag([1.0,-1.0,-1.0])

def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],float)

def rotvec(R):
    rv,_=cv2.Rodrigues(R)
    return rv.reshape(3)

def closest_rotation(M):
    U,_,Vt=np.linalg.svd(M)
    R=U@Vt
    if np.linalg.det(R)<0:
        U[:,-1]*=-1; R=U@Vt
    return R

def yaml_R_BC(path):
    fs=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open camera YAML: {path}")
    n=fs.getNode("T_BS")
    if n.empty():
        fs.release()
        raise RuntimeError("T_BS not found")

    # LeftCameraParams.yaml stores T_BS as a plain YAML map:
    #   rows: 4
    #   cols: 4
    #   data: [ ... ]
    # not as an !!opencv-matrix, so node.mat() is invalid on OpenCV 4.10.
    data=n.getNode("data")
    if data.empty() or not data.isSeq():
        fs.release()
        raise RuntimeError("T_BS.data is not a YAML sequence")
    vals=np.array([data.at(i).real() for i in range(data.size())],dtype=float)
    rows=int(round(n.getNode("rows").real()))
    cols=int(round(n.getNode("cols").real()))
    fs.release()
    if rows*cols != vals.size or rows < 3 or cols < 3:
        raise RuntimeError(f"invalid T_BS shape {rows}x{cols} with {vals.size} values")
    M=vals.reshape(rows,cols)
    return M[:3,:3]

def local_poly_second(t,p,halfwin=0.28,minn=7):
    n=len(t); out=np.full_like(p,np.nan,dtype=float)
    for i in range(n):
        m=np.abs(t-t[i])<=halfwin
        idx=np.flatnonzero(m)
        if len(idx)<minn: continue
        x=t[idx]-t[i]
        A=np.column_stack([np.ones_like(x),x,x*x,x*x*x])
        # downweight window edges
        w=(1-(np.abs(x)/halfwin)**3)**3
        Aw=A*np.sqrt(w)[:,None]
        for k in range(3):
            bw=p[idx,k]*np.sqrt(w)
            coef=np.linalg.lstsq(Aw,bw,rcond=None)[0]
            out[i,k]=2*coef[2]
    return out

def local_poly_first(t,p,halfwin=0.18,minn=5):
    n=len(t); out=np.full_like(p,np.nan,dtype=float)
    for i in range(n):
        idx=np.flatnonzero(np.abs(t-t[i])<=halfwin)
        if len(idx)<minn: continue
        x=t[idx]-t[i]
        A=np.column_stack([np.ones_like(x),x,x*x])
        w=(1-(np.abs(x)/halfwin)**3)**3
        Aw=A*np.sqrt(w)[:,None]
        for k in range(3):
            coef=np.linalg.lstsq(Aw,p[idx,k]*np.sqrt(w),rcond=None)[0]
            out[i,k]=coef[1]
    return out

def robust_lstsq(A,b,iters=8):
    w=np.ones(len(b))
    x=np.linalg.lstsq(A,b,rcond=None)[0]
    for _ in range(iters):
        r=A@x-b
        med=np.median(r); mad=np.median(np.abs(r-med))
        s=max(1e-9,1.4826*mad)
        u=np.abs(r-med)/(1.5*s)
        w=np.where(u<=1,1,1/u)
        Aw=A*np.sqrt(w)[:,None]; bw=b*np.sqrt(w)
        x=np.linalg.lstsq(Aw,bw,rcond=None)[0]
    return x,w,A@x-b

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
        p=np.array([float(r["cam_board_x"]),float(r["cam_board_y"]),float(r["cam_board_z"])])
        R=np.array([[float(r[f"R{i}{j}"]) for j in range(3)] for i in range(3)])
        # CSV R is board->camera from solvePnP. Convert to camera->board/world.
        R_WC=R.T
        gyro_frd=np.array([float(r["xgyro"]),float(r["ygyro"]),float(r["zgyro"])])
        acc_frd=np.array([float(r["xacc"]),float(r["yacc"]),float(r["zacc"])])
        q.append((t,p,R_WC,gyro_frd,acc_frd))
    if len(q)<80: raise SystemExit(f"Недостаточно качественных poses: {len(q)}")
    q.sort(key=lambda x:x[0])

    t=np.array([x[0] for x in q]); t-=t[0]
    p=np.array([x[1] for x in q])
    RWCs=[x[2] for x in q]
    gyro=np.array([x[3] for x in q])
    acc=np.array([x[4] for x in q])

    # Estimate body(FRD)->camera rotation directly from camera relative rotations vs FC gyro.
    # For short intervals: omega_C ~= R_C_B * omega_B.
    vc=[]; vb=[]
    for i in range(len(q)-1):
        dt=t[i+1]-t[i]
        if dt<=0 or dt>0.20: continue
        # Relative camera rotation expressed in camera_i.
        dR=RWCs[i].T@RWCs[i+1]
        wC=rotvec(dR)/dt
        wB=0.5*(gyro[i]+gyro[i+1])
        if np.linalg.norm(wB)<0.12 or np.linalg.norm(wC)<0.12: continue
        if np.linalg.norm(wB)>3.5 or np.linalg.norm(wC)>3.5: continue
        vb.append(wB);vc.append(wC)
    vb=np.array(vb);vc=np.array(vc)
    H=np.zeros((3,3))
    for b,c in zip(vb,vc): H+=np.outer(c,b)
    R_C_B=closest_rotation(H)
    R_B_C=R_C_B.T

    # Compare with existing YAML FLU rotation converted to FRD.
    R_Bflu_C=yaml_R_BC(args.yaml)
    R_Bfrd_C=D_FRD_TO_FLU@R_Bflu_C
    dR=R_Bfrd_C.T@R_B_C
    rot_delta_deg=np.linalg.norm(rotvec(dR))*180/math.pi

    # Build world<-body orientation from camera pose and estimated extrinsic.
    RWBs=[RWCs[i]@R_C_B for i in range(len(q))]

    # Differentiate measured camera trajectory and gyro.
    aC=local_poly_second(t,p,0.30,7)
    alpha=local_poly_first(t,gyro,0.20,5)

    Arows=[]; brows=[]; used_idx=[]
    for i in range(len(q)):
        if not np.all(np.isfinite(aC[i])) or not np.all(np.isfinite(alpha[i])): continue
        w=gyro[i]
        if np.linalg.norm(w)<0.18: continue
        R=RWBs[i]
        M=skew(alpha[i])+skew(w)@skew(w)
        # a_C - R*f = R*M*r - R*b + g
        y=aC[i]-R@acc[i]
        Ai=np.hstack([R@M,-R,np.eye(3)])
        Arows.append(Ai);brows.append(y);used_idx.append(i)
    A=np.vstack(Arows);b=np.hstack(brows)
    x,wts,res=robust_lstsq(A,b)
    r=x[:3];bias=x[3:6];g=x[6:9]

    # split-half stability
    est=[]
    for sel in [np.arange(len(used_idx))%2==0,np.arange(len(used_idx))%2==1]:
        inds=np.flatnonzero(np.repeat(sel,3))
        xx,_,_=robust_lstsq(A[inds],b[inds])
        est.append(xx[:3])
    spread=np.abs(est[0]-est[1])

    s=np.linalg.svd(A,compute_uv=False)
    cond=float(s[0]/s[-1]) if s[-1]>0 else float("inf")
    rms=float(np.sqrt(np.mean(res**2)))
    med=float(np.median(np.abs(res)))

    print("===== CAMERA LEVER-ARM IDENTIFICATION =====")
    print(f"quality poses used={len(q)} rotational pairs={len(vb)} dynamic samples={len(used_idx)}")
    print(f"estimated camera rotation vs current YAML: delta={rot_delta_deg:.2f} deg")
    print()
    print("FC IMU -> OV9281 optical center, BODY FRD:")
    print(f"  X forward = {r[0]*1000:+.1f} mm")
    print(f"  Y right   = {r[1]*1000:+.1f} mm")
    print(f"  Z down    = {r[2]*1000:+.1f} mm")
    print(f"  norm      = {np.linalg.norm(r)*1000:.1f} mm")
    print()
    print("split-half disagreement:")
    print(f"  dX/dY/dZ = {spread[0]*1000:.1f} / {spread[1]*1000:.1f} / {spread[2]*1000:.1f} mm")
    print(f"dynamic residual RMS/median = {rms:.3f} / {med:.3f} m/s^2")
    print(f"design condition number = {cond:.1f}")
    print(f"fitted accel bias = [{bias[0]:+.3f},{bias[1]:+.3f},{bias[2]:+.3f}] m/s^2")
    print(f"fitted gravity vector norm = {np.linalg.norm(g):.3f} m/s^2 vector={g}")
    print()
    stable=np.max(spread)<0.025 and cond<500 and 8.0<np.linalg.norm(g)<11.5
    print("CAMERA LEVER ARM: "+("PLAUSIBLE" if stable else "WEAK/UNOBSERVABLE"))
    print("Это оценка из динамики, не физическое измерение. Сравнение линейкой оставляем независимой проверкой.")

    out=args.run_dir/"camera_lever_arm_estimate.txt"
    out.write_text("\n".join([
      f"X_fwd_mm={r[0]*1000:.6f}",f"Y_right_mm={r[1]*1000:.6f}",f"Z_down_mm={r[2]*1000:.6f}",
      f"norm_mm={np.linalg.norm(r)*1000:.6f}",f"rotation_delta_deg={rot_delta_deg:.6f}",
      f"split_dx_mm={spread[0]*1000:.6f}",f"split_dy_mm={spread[1]*1000:.6f}",f"split_dz_mm={spread[2]*1000:.6f}",
      f"residual_rms={rms:.6f}",f"condition={cond:.6f}",f"gravity_norm={np.linalg.norm(g):.6f}"
    ])+"\n")
    print(f"output={out}")

if __name__=="__main__":main()
