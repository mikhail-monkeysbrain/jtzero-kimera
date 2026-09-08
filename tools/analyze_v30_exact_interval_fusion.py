#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics
from collections import defaultdict

ROOT=Path("/home/vio/jtzero_runs")
cand=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5")) or sorted(ROOT.glob("*TBS_ROLD_7P5*"))
if not cand:
    raise SystemExit("Не найден архив V27 TBS_ROLD_7P5")
R=cand[-1]

G=9.81

def f(x):
    try:return float(x)
    except:return float("nan")
def ii(x):
    try:return int(float(x))
    except:return 0
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def sd(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.pstdev(v) if len(v)>1 else 0.0
def norm(v):return math.sqrt(sum(x*x for x in v))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def add(a,b):return tuple(x+y for x,y in zip(a,b))
def mul(a,s):return tuple(x*s for x in a)
def matvec(M,v):return tuple(sum(M[r][c]*v[c] for c in range(3)) for r in range(3))
def matmul(A,B):return tuple(tuple(sum(A[r][k]*B[k][c] for k in range(3)) for c in range(3)) for r in range(3))
def transpose(A):return tuple(tuple(A[c][r] for c in range(3)) for r in range(3))
def rpy_matrix(roll_deg,pitch_deg,yaw_deg):
    r,p,y=[math.radians(v) for v in (roll_deg,pitch_deg,yaw_deg)]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr))
    Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    Rz=((cy,-sy,0),(sy,cy,0),(0,0,1))
    return matmul(Rz,matmul(Ry,Rx))
def angle_deg(a,b):
    na,nb=norm(a),norm(b)
    if na<1e-12 or nb<1e-12:return float("nan")
    c=max(-1,min(1,sum(x*y for x,y in zip(a,b))/(na*nb)))
    return math.degrees(math.acos(c))
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<4:return float("nan")
    X=[a for a,b in p];Y=[b for a,b in p];mx=mean(X);my=mean(Y)
    sx=sum((a-mx)**2 for a in X); sy=sum((b-my)**2 for b in Y)
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy) if sx>0 and sy>0 else float("nan")
def read(name):
    with (R/name).open() as fh:return list(csv.DictReader(fh))

backend=read("jtzero_500mm_v25_backend.csv")
front=read("jtzero_500mm_v25_frontend.csv")
events=read("jtzero_500mm_v25_events.csv")

# Exact timestamp map. Multiple frontend rows at same timestamp are reduced to the PIM-valid one.
F={}
for r in front:
    if ii(r.get("pim_valid",0))==1:
        F[ii(r["timestamp_ns"])]=r

E=defaultdict(dict)
for e in events:
    n=ii(e.get("leg"))
    if 1<=n<=4:E[n][e.get("event","")]=e

samples=[]
print("="*168)
print("V30 EXACT-INTERVAL FUSION DECOMPOSITION")
print("="*168)
print(f"RUN: {R}")
print("Каждая строка использует ОДИН И ТОТ ЖЕ интервал i→j: предыдущий backend state i, frontend PIM с timestamp j и backend state j.")
print("IMU-only prediction:")
print("  Pj_pred = Pi + Vi*dt + 0.5*g*dt² + Ri*deltaPij")
print("  Vj_pred = Vi + g*dt + Ri*deltaVij")
print("где g=[0,0,-9.81] м/с² в мировой системе Kimera.")
print("mono_body_* = направление monocular visual translation; его масштаб произвольный, поэтому используется только угол направления.\n")

for leg in range(1,5):
    if "START" not in E[leg] or "END" not in E[leg]:continue
    t0=ii(E[leg]["START"]["state_timestamp_ns"]); t1=ii(E[leg]["END"]["state_timestamp_ns"])
    br=[r for r in backend if t0<=ii(r["timestamp_ns"])<=t1]
    direction="A_TO_B" if leg%2 else "B_TO_A"
    sign=1 if leg%2 else -1
    print("-"*168)
    print(f"PASS {leg} {direction}")
    print("-"*168)
    print("KF_i→j dtB_ms dtP_ms Δdt_ms OPT_Xmm IMU_Xmm FUSION_CORR_Xmm |corrP|mm |corrV|mm VISang_deg INLIER")
    leg_samples=[]
    for a,b in zip(br[:-1],br[1:]):
        tj=ii(b["timestamp_ns"])
        fr=F.get(tj)
        if fr is None:continue
        dtb=(tj-ii(a["timestamp_ns"]))/1e9
        dtp=f(fr["pim_dt_s"])
        if not (0<dtb<1 and 0<dtp<1):continue
        # Exact-interval gate: backend interval and PIM interval must agree within 2 ms.
        ddt=abs(dtb-dtp)
        if ddt>0.002:continue

        Pi=(f(a["px_m"]),f(a["py_m"]),f(a["pz_m"]))
        Vi=(f(a["vx_m_s"]),f(a["vy_m_s"]),f(a["vz_m_s"]))
        Pj=(f(b["px_m"]),f(b["py_m"]),f(b["pz_m"]))
        Vj=(f(b["vx_m_s"]),f(b["vy_m_s"]),f(b["vz_m_s"]))
        Ri=rpy_matrix(f(a["roll_deg"]),f(a["pitch_deg"]),f(a["yaw_deg"]))
        dp=(f(fr["pim_dpx"]),f(fr["pim_dpy"]),f(fr["pim_dpz"]))
        dv=(f(fr["pim_dvx"]),f(fr["pim_dvy"]),f(fr["pim_dvz"]))
        gw=(0.0,0.0,-G)
        Ppred=add(add(add(Pi,mul(Vi,dtp)),mul(gw,0.5*dtp*dtp)),matvec(Ri,dp))
        Vpred=add(add(Vi,mul(gw,dtp)),matvec(Ri,dv))
        opt_delta=sub(Pj,Pi)
        imu_delta=sub(Ppred,Pi)
        corrP=sub(Pj,Ppred)
        corrV=sub(Vj,Vpred)

        # Canonical X for human-readable directional comparison.
        optx=sign*opt_delta[0]*1000
        imux=sign*imu_delta[0]*1000
        cx=sign*corrP[0]*1000

        visang=float("nan")
        if ii(fr.get("mono_pose_valid",0))==1:
            vis=(f(fr["mono_body_tx"]),f(fr["mono_body_ty"]),f(fr["mono_body_tz"]))
            # Compare visual direction with optimized body-frame translation.
            opt_body=matvec(transpose(Ri),opt_delta)
            visang=min(angle_deg(vis,opt_body),angle_deg(mul(vis,-1),opt_body))

        s={"leg":leg,"direction":direction,"dtb":dtb,"dtp":dtp,"ddt_ms":ddt*1000,
           "optx_mm":optx,"imux_mm":imux,"corrx_mm":cx,
           "corrp_mm":norm(corrP)*1000,"corrv":norm(corrV),
           "visang":visang,"inlier":f(fr.get("mono_inlier_ratio","nan")),
           "mono_valid":ii(fr.get("mono_pose_valid",0))}
        samples.append(s);leg_samples.append(s)
        print(f"{ii(a['keyframe']):3d}→{ii(b['keyframe']):3d} {dtb*1000:7.1f} {dtp*1000:7.1f} {ddt*1000:7.2f}"
              f" {optx:+8.2f} {imux:+8.2f} {cx:+12.2f} {s['corrp_mm']:9.2f} {s['corrv']:9.4f}"
              f" {visang:10.2f} {s['inlier']:6.3f}")

    if leg_samples:
        print(f"SUMMARY pass {leg}: exact_intervals={len(leg_samples)}"
              f" mean|fusion position correction|={mean([x['corrp_mm'] for x in leg_samples]):.2f} mm"
              f" mean canonical correction X={mean([x['corrx_mm'] for x in leg_samples]):+.2f} mm"
              f" mean visual-direction angle={mean([x['visang'] for x in leg_samples]):.2f} deg")

print("\n"+"="*168)
print("GLOBAL EXACT-MATCH CHECK")
print("="*168)
print(f"exact intervals accepted: {len(samples)}")
print(f"mean |dt_backend-dt_PIM|: {mean([x['ddt_ms'] for x in samples]):.3f} ms")
print(f"max  |dt_backend-dt_PIM|: {max([x['ddt_ms'] for x in samples], default=float('nan')):.3f} ms")

print("\n"+"="*168)
print("DIRECTION SUMMARY")
print("="*168)
for d in ("A_TO_B","B_TO_A"):
    q=[x for x in samples if x["direction"]==d]
    print(f"{d}: n={len(q)} mean corrX={mean([x['corrx_mm'] for x in q]):+.2f} mm/interval"
          f" mean |corrP|={mean([x['corrp_mm'] for x in q]):.2f} mm"
          f" mean |corrV|={mean([x['corrv'] for x in q]):.4f} m/s"
          f" visual angle={mean([x['visang'] for x in q]):.2f} deg"
          f" inlier={mean([x['inlier'] for x in q]):.3f}")

print("\n"+"="*168)
print("INTERPRETATION")
print("="*168)
print("corrP = optimized backend position minus pure IMU prediction on the SAME interval.")
print("corrV = optimized backend velocity minus pure IMU prediction on the SAME interval.")
print("Large corrP does not itself mean an error: visual update is expected to correct IMU prediction.")
print("What matters next is whether the SIGN / magnitude of these corrections systematically differs between good and bad passes.")
print("If exact_intervals is low or dt mismatch is >2 ms, do NOT interpret correction values; new in-process logging is required.")
