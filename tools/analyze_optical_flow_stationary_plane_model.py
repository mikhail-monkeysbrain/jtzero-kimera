#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, math, statistics
from pathlib import Path

try:
    import cv2
    import numpy as np
except Exception as e:
    raise SystemExit(f"Нужны python3-opencv и numpy: {e}")

# Current temporary mount geometry, body = FRD.
R_CAM_B = np.array([0.0625, 0.0, 0.0500], dtype=float)
R_RNG_B = np.array([-0.015, 0.0, 0.0710], dtype=float)

# Camera axes expressed in body FRD:
# Cx -> +Y_FRD, Cy -> -X_FRD, Cz -> +Z_FRD.
R_BC = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
], dtype=float)

# Saved calibration, with the same production focal_scale as current mount.
FX0 = 568.53170752165227
FY0 = 569.68005562865858
CX = 315.98271077441063
CY = 239.88148589100641
D = np.array([0.073569192194028493, -0.095253893789117,
              -0.010810530757187299, -0.0022843373576970235,
              0.082177400802757483], dtype=float)
W, H = 640, 480

def f(r,k,d=float("nan")):
    try: return float(r.get(k,""))
    except: return d

def i(r,k,d=0):
    try: return int(float(r.get(k,"")))
    except: return d

def pctl(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs: return float("nan")
    j=min(len(xs)-1,max(0,int(round((len(xs)-1)*p))))
    return xs[j]

def rms(xs):
    xs=[x for x in xs if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else float("nan")

def r_nb(roll,pitch,yaw):
    # ArduPilot body FRD -> NED, 3-2-1 yaw-pitch-roll.
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,   cp*sr,          cp*cr],
    ], dtype=float)

def med_vec(vs):
    a=np.asarray(vs,float)
    return np.median(a,axis=0)

def make_roi_rays(focal_scale, nx=13, ny=11):
    fx,fy=FX0*focal_scale,FY0*focal_scale
    K=np.array([[fx,0,CX],[0,fy,CY],[0,0,1]],dtype=float)
    # Same ROI as current mount: 0.20 0.32 0.80 0.90
    us=np.linspace(0.20*W,0.80*W,nx)
    vs=np.linspace(0.32*H,0.90*H,ny)
    pts=np.array([[u,v] for v in vs for u in us],dtype=np.float32).reshape(-1,1,2)
    und=cv2.undistortPoints(pts,K,D).reshape(-1,2)
    return np.column_stack([und,np.ones(len(und))])

def plane_flow(prev, cur, plane_z, rays_c, lever=True):
    dt=f(cur,"dt_s")
    if not (0<dt<0.2): return None

    rp=[f(prev,k) for k in ("fc_roll","fc_pitch","fc_yaw")]
    rc=[f(cur,k) for k in ("fc_roll","fc_pitch","fc_yaw")]
    if not all(map(math.isfinite,rp+rc)): return None

    R0=r_nb(*rp)
    R1=r_nb(*rc)
    RNC0=R0@R_BC
    RNC1=R1@R_BC

    C0=R0@R_CAM_B if lever else np.zeros(3)
    C1=R1@R_CAM_B if lever else np.zeros(3)

    du=[]; dv=[]
    for q0 in rays_c:
        dN=RNC0@q0
        if abs(dN[2])<1e-7: continue
        t=(plane_z-C0[2])/dN[2]
        if t<=0: continue
        P=C0+t*dN
        q1=RNC1.T@(P-C1)
        if q1[2]<=1e-7: continue
        x0,y0=q0[0]/q0[2],q0[1]/q0[2]
        x1,y1=q1[0]/q1[2],q1[1]/q1[2]
        du.append(x1-x0); dv.append(y1-y0)
    if len(du)<20: return None

    # Same convention as production estimator:
    # camera angular flow [x,y] = [dv/dt, -du/dt],
    # then camera -> body FRD.
    fc=np.array([statistics.median(dv)/dt,
                 -statistics.median(du)/dt,
                 0.0])
    fb=R_BC@fc
    return fb[:2]

def infer_ground_z(r):
    roll,pitch,yaw=f(r,"fc_roll"),f(r,"fc_pitch"),f(r,"fc_yaw")
    lm=f(r,"luna_m")
    if not all(map(math.isfinite,[roll,pitch,yaw,lm])) or lm<=0: return float("nan")
    R=r_nb(roll,pitch,yaw)
    Cr=R@R_RNG_B
    down=R@np.array([0.0,0.0,1.0])
    # TF-Luna beam assumed aligned with body +Z (down).
    return float(Cr[2] + lm*down[2])

def main():
    ap=argparse.ArgumentParser(description="Stationary-plane optical-flow model: rotation vs rotation+lever arm")
    ap.add_argument("csv",type=Path)
    ap.add_argument("--focal-scale",type=float,default=0.931)
    args=ap.parse_args()

    rows=list(csv.DictReader(args.csv.open(newline="")))
    if len(rows)<3: raise SystemExit("Слишком мало строк")

    gz=[infer_ground_z(r) for r in rows]
    gzv=[x for x in gz if math.isfinite(x)]
    if not gzv: raise SystemExit("Нет валидной геометрии TF-Luna/ATTITUDE")
    plane_z=statistics.median(gzv)
    rays=make_roi_rays(args.focal_scale)

    e_rot=[]; e_full=[]; meas_mag=[]; pred_rot_mag=[]; pred_full_mag=[]
    pairs=0
    samples=[]
    for j in range(1,len(rows)):
        a,b=rows[j-1],rows[j]
        if i(b,"valid")!=1 or i(b,"flow_sent")!=1: continue
        dt=f(b,"dt_s")
        if not (0<dt<0.2): continue
        m=np.array([f(b,"flow_body_x"),f(b,"flow_body_y")],float)
        if not np.all(np.isfinite(m)): continue

        pr=plane_flow(a,b,plane_z,rays,lever=False)
        pf=plane_flow(a,b,plane_z,rays,lever=True)
        if pr is None or pf is None: continue
        er=float(np.linalg.norm(m-pr))
        ef=float(np.linalg.norm(m-pf))
        mm=float(np.linalg.norm(m))
        e_rot.append(er); e_full.append(ef)
        meas_mag.append(mm); pred_rot_mag.append(float(np.linalg.norm(pr))); pred_full_mag.append(float(np.linalg.norm(pf)))
        pairs+=1
        samples.append((ef,j,m,pr,pf))

    print("===== МОДЕЛЬ НЕПОДВИЖНОЙ ПЛОСКОСТИ: ВРАЩЕНИЕ + LEVER ARM =====")
    print(f"строк = {len(rows)}, использовано пар кадров = {pairs}")
    print(f"геометрия: camera FRD={R_CAM_B.tolist()} m, range FRD={R_RNG_B.tolist()} m")
    print(f"plane Z from TF-Luna median = {plane_z:.4f} m")
    print(f"inferred plane Z median/p05/p95 = {statistics.median(gzv):.4f}/{pctl(gzv,.05):.4f}/{pctl(gzv,.95):.4f} m")
    print(f"inferred plane Z span = {(max(gzv)-min(gzv))*1000:.1f} mm")
    print()
    print("ИЗМЕРЕННЫЙ FLOW:")
    print(f"  |flow| median/p95 = {statistics.median(meas_mag):.4f}/{pctl(meas_mag,.95):.4f} rad/s")
    print()
    print("MODEL 1 — только изменение ориентации, камера в центре IMU:")
    print(f"  predicted |flow| median/p95 = {statistics.median(pred_rot_mag):.4f}/{pctl(pred_rot_mag,.95):.4f} rad/s")
    print(f"  residual RMS/median/p95 = {rms(e_rot):.4f}/{statistics.median(e_rot):.4f}/{pctl(e_rot,.95):.4f} rad/s")
    print()
    print("MODEL 2 — ориентация + реальный lever arm камеры:")
    print(f"  predicted |flow| median/p95 = {statistics.median(pred_full_mag):.4f}/{pctl(pred_full_mag,.95):.4f} rad/s")
    print(f"  residual RMS/median/p95 = {rms(e_full):.4f}/{statistics.median(e_full):.4f}/{pctl(e_full,.95):.4f} rad/s")
    improve=(1-rms(e_full)/rms(e_rot))*100 if rms(e_rot)>1e-12 else 0
    print(f"  улучшение RMS от lever arm = {improve:+.1f}%")
    print()

    samples.sort(reverse=True,key=lambda x:x[0])
    print("ХУДШИЕ 8 КАДРОВ ПО ПОЛНОЙ МОДЕЛИ:")
    for ef,j,m,pr,pf in samples[:8]:
        r=rows[j]
        print(f"  row={j:4d} dt={f(r,'dt_s')*1000:6.1f}ms luna={f(r,'luna_m'):.3f} "
              f"meas=({m[0]:+.3f},{m[1]:+.3f}) full=({pf[0]:+.3f},{pf[1]:+.3f}) "
              f"res={ef:.3f} rad/s RPY=({math.degrees(f(r,'fc_roll')):+.1f},"
              f"{math.degrees(f(r,'fc_pitch')):+.1f},{math.degrees(f(r,'fc_yaw')):+.1f})")

    print()
    rr=rms(e_rot); rf=rms(e_full)
    plane_span=(max(gzv)-min(gzv))
    if rf < 0.5*rr:
        print("ВЫВОД: lever-arm/полная геометрия объясняет существенную часть ложного flow при вращении.")
    elif rf < 0.85*rr:
        print("ВЫВОД: lever-arm помогает, но остаётся значительный residual; дальше проверять timing и модель плоскости.")
    else:
        print("ВЫВОД: lever-arm почти не объясняет residual; основной кандидат — temporal alignment / camera-motion model.")
    if plane_span>0.05:
        print("ПРИМЕЧАНИЕ: inferred ground plane меняется >50 мм; аппарат/дальномер не соответствуют идеальной stationary-plane модели.")

if __name__=="__main__":
    main()
