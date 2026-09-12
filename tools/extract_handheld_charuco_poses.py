#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path
import cv2, numpy as np

def _read_seq_node(fs, name):
    n=fs.getNode(name)
    if n.empty():
        raise RuntimeError(f"YAML node not found: {name}")
    if not n.isSeq():
        # Some OpenCV YAML writers may store a numeric vector as a matrix.
        m=n.mat()
        if m is None:
            raise RuntimeError(f"YAML node {name} is neither sequence nor matrix")
        return np.asarray(m,dtype=float).reshape(-1)
    return np.array([n.at(i).real() for i in range(n.size())],dtype=float)

def load_k(path:Path, scale:float):
    fs=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open camera YAML: {path}")
    vals=_read_seq_node(fs,"intrinsics")
    dv=_read_seq_node(fs,"distortion_coefficients")
    fs.release()
    if vals.size < 4:
        raise RuntimeError(f"intrinsics must contain at least 4 values, got {vals.size}")
    fx,fy,cx,cy=map(float,vals[:4])
    K=np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1]],dtype=float)
    return K,dv.astype(float)

def make_board(nx,ny,legacy=False):
    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        board=cv2.aruco.CharucoBoard((nx,ny),0.027324,0.020043,dic)
    except Exception:
        board=cv2.aruco.CharucoBoard_create(nx,ny,0.027324,0.020043,dic)
    if legacy and hasattr(board,"setLegacyPattern"):
        board.setLegacyPattern(True)
    return dic,board

def choose_board(rows, bin_path, K, D, sample_n=80):
    candidates=[]
    for nx,ny in [(5,7),(7,5)]:
        for legacy in [False,True]:
            dic,board=make_board(nx,ny,legacy)
            candidates.append([nx,ny,legacy,dic,board,0,0])

    step=max(1,len(rows)//sample_n)
    picks=rows[::step][:sample_n]
    with bin_path.open("rb") as fh:
        for r in picks:
            off=int(float(r["jpeg_offset"])); sz=int(float(r["jpeg_size"]))
            fh.seek(off); data=fh.read(sz)
            img=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
            if img is None: continue
            # dictionary is same for all candidates, detect once
            dic=candidates[0][3]
            try:
                det=cv2.aruco.ArucoDetector(dic)
                corners,ids,_=det.detectMarkers(img)
            except Exception:
                corners,ids,_=cv2.aruco.detectMarkers(img,dic)
            if ids is None or len(ids)<4: continue
            for q in candidates:
                board=q[4]
                try:
                    n,cc,ci=cv2.aruco.interpolateCornersCharuco(corners,ids,img,board,K,D)
                except cv2.error:
                    continue
                if ci is not None and cc is not None and int(n)>0:
                    q[5]+=1
                    q[6]+=int(n)
    candidates.sort(key=lambda q:(q[5],q[6]),reverse=True)
    return candidates

def detect(img,dic,board,K,D):
    try:
        det=cv2.aruco.ArucoDetector(dic)
        corners,ids,_=det.detectMarkers(img)
    except Exception:
        corners,ids,_=cv2.aruco.detectMarkers(img,dic)
    if ids is None or len(ids)<4:return None
    n,cc,ci=cv2.aruco.interpolateCornersCharuco(corners,ids,img,board,K,D)
    if ci is None or cc is None or int(n)<6:return None
    ids1=ci.reshape(-1).astype(int)
    pts2=cc.reshape(-1,2).astype(np.float64)
    try: chess=np.asarray(board.getChessboardCorners(),dtype=np.float64)
    except Exception: chess=np.asarray(board.chessboardCorners,dtype=np.float64)
    pts3=chess[ids1]
    ok,rvec,tvec=cv2.solvePnP(pts3,pts2,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None
    proj,_=cv2.projectPoints(pts3,rvec,tvec,K,D)
    err=np.linalg.norm(proj.reshape(-1,2)-pts2,axis=1)
    R,_=cv2.Rodrigues(rvec)
    return dict(n=int(n),rms=float(np.sqrt(np.mean(err**2))),t=tvec.reshape(3),R=R)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--yaml",type=Path,default=Path("params/JTZeroMonoFLU/LeftCameraParams.yaml"))
    ap.add_argument("--focal-scale",type=float,default=1.1060)
    args=ap.parse_args()
    with (args.run_dir/"frames.csv").open(newline="") as f: rows=list(csv.DictReader(f))
    K,D=load_k(args.yaml,args.focal_scale)
    candidates=choose_board(rows,args.run_dir/"frames.mjpgbin",K,D)
    print("===== CHARUCO BOARD PROBE =====")
    for nx,ny,legacy,_,_,frames_ok,corners_ok in candidates:
        print(f"{nx}x{ny} legacy={int(legacy)}: frames={frames_ok} interpolated_corners={corners_ok}")
    if not candidates or candidates[0][5]==0:
        raise SystemExit("Ни одна 5x7/7x5 ChArUco layout-гипотеза не дала интерполированных углов")
    nx,ny,legacy,dic,board,_,_=candidates[0]
    print(f"selected board: {nx}x{ny} legacy={int(legacy)}")
    print()
    out=[]; detected=0
    with (args.run_dir/"frames.mjpgbin").open("rb") as fh:
        for r in rows:
            off=int(float(r["jpeg_offset"])); sz=int(float(r["jpeg_size"]))
            fh.seek(off); data=fh.read(sz)
            img=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
            if img is None: continue
            q=detect(img,dic,board,K,D)
            if q is None: continue
            detected+=1
            t=q["t"]; R=q["R"]
            # camera center expressed in board coordinates
            cpos=-(R.T@t)
            out.append([
              r["frame"],r["camera_ts_ns"],r["recv_ns"],r["luna_m"],
              r["roll"],r["pitch"],r["yaw"],r["xacc"],r["yacc"],r["zacc"],
              r["xgyro"],r["ygyro"],r["zgyro"],
              q["n"],q["rms"],*t.tolist(),*cpos.tolist(),*R.reshape(-1).tolist()
            ])
    hdr=["frame","camera_ts_ns","recv_ns","luna_m","roll","pitch","yaw",
         "xacc","yacc","zacc","xgyro","ygyro","zgyro","charuco_corners","reproj_rms_px",
         "tvec_x","tvec_y","tvec_z","cam_board_x","cam_board_y","cam_board_z"]+[f"R{i}{j}" for i in range(3) for j in range(3)]
    op=args.run_dir/"charuco_poses.csv"
    with op.open("w",newline="") as f:
        w=csv.writer(f);w.writerow(hdr);w.writerows(out)
    rms=[float(x[14]) for x in out]; nc=[int(x[13]) for x in out]; z=[float(x[17]) for x in out]
    print("===== CHARUCO POSE EXTRACTION =====")
    print(f"board selected: {nx}x{ny}, legacy={int(legacy)}, DICT_4X4_50, square=27.324 mm, marker=20.043 mm")
    print(f"effective focal scale={args.focal_scale:.4f}")
    print(f"frames={len(rows)} poses={len(out)} detection={100*len(out)/max(1,len(rows)):.1f}%")
    if out:
        print(f"corners median/min/max={statistics.median(nc):.0f}/{min(nc)}/{max(nc)}")
        print(f"reprojection RMS median/p95={statistics.median(rms):.3f}/{np.percentile(rms,95):.3f} px")
        print(f"tvec_z median/min/max={statistics.median(z):.3f}/{min(z):.3f}/{max(z):.3f} m")
    print(f"output={op}")
    print()
    good=len(out)>=0.7*len(rows) and statistics.median(rms)<1.5 if out else False
    print("POSE DATASET: "+("PASS" if good else "WEAK"))
    print("TF-Luna table/floor samples are NOT classified here; that is the next robust-mixture geometry step.")
if __name__=="__main__":main()
