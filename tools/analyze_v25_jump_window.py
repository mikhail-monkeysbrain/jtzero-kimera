#!/usr/bin/env python3
import csv, os, sys, math

CHAIN = os.path.expanduser("~/jtzero_kimera_chain.csv")
BACKEND = os.path.expanduser("~/jtzero_500mm_v25_backend.csv")
FRONTEND = os.path.expanduser("~/jtzero_500mm_v25_frontend.csv")

def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

def f(r,k,default=0.0):
    try: return float(r.get(k,default))
    except: return default

def i(r,k,default=0):
    try: return int(float(r.get(k,default)))
    except: return default

def vec(r, names):
    return [f(r,n) for n in names]

def norm(v):
    return math.sqrt(sum(x*x for x in v))

rows=load(CHAIN)
if len(rows)<2:
    raise SystemExit("chain CSV empty")

# Find the largest one-keyframe STATE position jump instead of hard-coding KF=202.
best=None
for a,b in zip(rows, rows[1:]):
    pa=vec(a,["state_px","state_py","state_pz"])
    pb=vec(b,["state_px","state_py","state_pz"])
    dp=[(pb[j]-pa[j])*1000 for j in range(3)]
    mag=norm(dp)
    dt=(f(b,"timestamp_ns")-f(a,"timestamp_ns"))/1e9
    if best is None or mag>best[0]:
        best=(mag, i(b,"keyframe"), dt, dp)
mag,kf0,dt,dp=best
print("================ V25 LARGEST STATE JUMP ================")
print(f"largest jump: KF={kf0} dP={mag:.2f}mm dt={dt*1000:.2f}ms delta=[{dp[0]:+.1f},{dp[1]:+.1f},{dp[2]:+.1f}]mm")
print(f"equivalent speed={mag/max(dt,1e-9):.1f}mm/s")

bykf={i(r,"keyframe"):r for r in rows}
print("\n================ CHAIN WINDOW ================")
print(" KF   stateStep[mm]  predStep[mm]  optCorr[mm]  PRED-STATE[mm]  predV->stateV[mm/s]")
for k in range(max(2,kf0-7), kf0+8):
    if k not in bykf or k-1 not in bykf: continue
    a,b=bykf[k-1],bykf[k]
    sa=vec(a,["state_px","state_py","state_pz"]); sb=vec(b,["state_px","state_py","state_pz"])
    pred=vec(b,["pred_px","pred_py","pred_pz"])
    state_step=[(sb[j]-sa[j])*1000 for j in range(3)]
    pred_step=[(pred[j]-sa[j])*1000 for j in range(3)]
    opt=[(sb[j]-pred[j])*1000 for j in range(3)]
    pv=vec(b,["pred_vx","pred_vy","pred_vz"]); sv=vec(b,["state_vx","state_vy","state_vz"])
    dv=norm([(sv[j]-pv[j])*1000 for j in range(3)])
    mark=" <<<" if k==kf0 else ""
    print(f"{k:3d} {norm(state_step):13.2f} {norm(pred_step):13.2f} {norm(opt):12.2f} {norm(opt):15.2f} {dv:20.2f}{mark}")

if os.path.exists(FRONTEND):
    fr=load(FRONTEND)
    # Frontend frame_id generally tracks keyframe callback stream; print rows nearest the anomalous KF.
    cand=[r for r in fr if abs(i(r,"frame_id")-kf0)<=10]
    print("\n================ FRONTEND NEAR SAME INDEX ================")
    print(" frame keyframe status          tracked inliers putatives ratio ransac")
    for r in cand:
        print(f"{i(r,'frame_id'):5d} {i(r,'is_keyframe'):8d} {r.get('mono_status',''):14s} {i(r,'tracked_features'):7d} {i(r,'mono_inliers'):7d} {i(r,'mono_putatives'):9d} {f(r,'mono_inlier_ratio'):5.2f} {i(r,'mono_ransac_iters'):6d}")
else:
    print("\nFRONTEND CSV not found")

print("\n================ VERDICT ================")
r=bykf[kf0]; prev=bykf.get(kf0-1)
if prev:
    sp=vec(prev,["state_px","state_py","state_pz"]); pred=vec(r,["pred_px","pred_py","pred_pz"]); st=vec(r,["state_px","state_py","state_pz"])
    predstep=norm([(pred[j]-sp[j])*1000 for j in range(3)])
    optcorr=norm([(st[j]-pred[j])*1000 for j in range(3)])
    print(f"At KF={kf0}: IMU/preintegration prediction step={predstep:.2f}mm; optimizer correction={optcorr:.2f}mm; final STATE jump={mag:.2f}mm.")
    if predstep > 0.7*mag and optcorr < 0.3*mag:
        print("SOURCE: prediction/preintegration side dominates the jump.")
    elif optcorr > 0.7*mag:
        print("SOURCE: optimizer/factor-graph correction dominates the jump.")
    else:
        print("SOURCE: mixed prediction + optimizer event; inspect both sides.")
