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
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<4:return float("nan")
    X=[a for a,b in p]; Y=[b for a,b in p]
    mx,my=mean(X),mean(Y)
    sx=sum((a-mx)**2 for a in X); sy=sum((b-my)**2 for b in Y)
    if sx<=0 or sy<=0:return float("nan")
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy)
def norm(v):return math.sqrt(sum(x*x for x in v))
def add(a,b):return tuple(x+y for x,y in zip(a,b))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def mul(a,s):return tuple(x*s for x in a)
def matvec(M,v):return tuple(sum(M[r][c]*v[c] for c in range(3)) for r in range(3))
def matmul(A,B):return tuple(tuple(sum(A[r][k]*B[k][c] for k in range(3)) for c in range(3)) for r in range(3))
def rpy_matrix(rdeg,pdeg,ydeg):
    r,p,y=map(math.radians,(rdeg,pdeg,ydeg))
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr))
    Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    Rz=((cy,-sy,0),(sy,cy,0),(0,0,1))
    return matmul(Rz,matmul(Ry,Rx))
def read(n):
    with (R/n).open() as fh:return list(csv.DictReader(fh))

backend=read("jtzero_500mm_v25_backend.csv")
front=read("jtzero_500mm_v25_frontend.csv")
events=read("jtzero_500mm_v25_events.csv")

F={}
for r in front:
    if ii(r.get("pim_valid",0))==1:
        F[ii(r["timestamp_ns"])]=r

E=defaultdict(dict)
for e in events:
    n=ii(e.get("leg"))
    if 1<=n<=4:E[n][e.get("event","")]=e

# Build exact intervals as in V30/V31.
rows=[]
for leg in range(1,5):
    if "START" not in E[leg] or "END" not in E[leg]:continue
    t0=ii(E[leg]["START"]["state_timestamp_ns"]); t1=ii(E[leg]["END"]["state_timestamp_ns"])
    br=[r for r in backend if t0<=ii(r["timestamp_ns"])<=t1]
    sign=1 if leg%2 else -1
    direction="A_TO_B" if leg%2 else "B_TO_A"

    # bias immediately before measured pass: last backend state strictly before START.
    prior=[r for r in backend if ii(r["timestamp_ns"])<t0]
    pre=prior[-1] if prior else br[0]
    Rpre=rpy_matrix(f(pre["roll_deg"]),f(pre["pitch_deg"]),f(pre["yaw_deg"]))
    bpre=(f(pre["bax"]),f(pre["bay"]),f(pre["baz"]))
    bpre_world=matvec(Rpre,bpre)
    pre_bias_x=sign*bpre_world[0]

    for idx,(a,b) in enumerate(zip(br[:-1],br[1:])):
        tj=ii(b["timestamp_ns"]); fr=F.get(tj)
        if fr is None:continue
        dt=(tj-ii(a["timestamp_ns"]))/1e9
        dtp=f(fr["pim_dt_s"])
        if not (0<dt<1 and abs(dt-dtp)<=0.002):continue

        Pi=(f(a["px_m"]),f(a["py_m"]),f(a["pz_m"]))
        Vi=(f(a["vx_m_s"]),f(a["vy_m_s"]),f(a["vz_m_s"]))
        Pj=(f(b["px_m"]),f(b["py_m"]),f(b["pz_m"]))
        Vj=(f(b["vx_m_s"]),f(b["vy_m_s"]),f(b["vz_m_s"]))
        Ri=rpy_matrix(f(a["roll_deg"]),f(a["pitch_deg"]),f(a["yaw_deg"]))
        dp=(f(fr["pim_dpx"]),f(fr["pim_dpy"]),f(fr["pim_dpz"]))
        dv=(f(fr["pim_dvx"]),f(fr["pim_dvy"]),f(fr["pim_dvz"]))
        gw=(0,0,-G)
        Ppred=add(add(add(Pi,mul(Vi,dt)),mul(gw,0.5*dt*dt)),matvec(Ri,dp))
        Vpred=add(add(Vi,mul(gw,dt)),matvec(Ri,dv))
        corrP=sub(Pj,Ppred)

        ba=(f(a["bax"]),f(a["bay"]),f(a["baz"]))
        bb=(f(b["bax"]),f(b["bay"]),f(b["baz"]))
        ba_world=matvec(Ri,ba)
        Rb=rpy_matrix(f(b["roll_deg"]),f(b["pitch_deg"]),f(b["yaw_deg"]))
        bb_world=matvec(Rb,bb)

        rows.append({
            "leg":leg,"direction":direction,"idx":idx,"dt":dt,
            "corrx":sign*corrP[0]*1000,
            "bias_start_x":sign*ba_world[0],
            "bias_end_x":sign*bb_world[0],
            "bias_prepass_x":pre_bias_x,
            "bias_start_pos_mm":sign*ba_world[0]*0.5*dt*dt*1000,
            "bias_prepass_pos_mm":pre_bias_x*0.5*dt*dt*1000,
        })

print("="*170)
print("V32 — ACCELEROMETER BIAS CAUSALITY SCREEN")
print("="*170)
print(f"RUN: {R}")
print("Цель: отличить 'bias создаёт систематическую IMU-ошибку' от 'backend меняет bias в ответ на визуальную коррекцию'.")
print("bias = внутренняя оценка постоянной ошибки акселерометра.")
print("prepass bias = значение bias из последнего backend state ДО нажатия START данного прохода.")
print("Если prepass bias заранее предсказывает знак/величину последующей corrX, причинная гипотеза усиливается.")
print("Если связь появляется только у bias ПОСЛЕ update, это больше похоже на реакцию оптимизатора.\n")

print("PASS-LEVEL PRE-EXISTING BIAS")
print("-"*170)
for leg in range(1,5):
    q=[r for r in rows if r["leg"]==leg]
    if not q:continue
    print(f"PASS {leg} {q[0]['direction']}:"
          f" prepass_biasX={q[0]['bias_prepass_x']:+.5f} m/s²"
          f"  mean_start_biasX={mean([r['bias_start_x'] for r in q]):+.5f}"
          f"  mean_end_biasX={mean([r['bias_end_x'] for r in q]):+.5f}"
          f"  mean_corrX={mean([r['corrx'] for r in q]):+.3f} mm/int"
          f"  prepass_bias_pos={mean([r['bias_prepass_pos_mm'] for r in q]):+.3f} mm/int")

print("\nWITHIN-DIRECTION CORRELATION: CURRENT vs FUTURE")
print("-"*170)
for d in ("A_TO_B","B_TO_A"):
    q=[r for r in rows if r["direction"]==d]
    # current interval relation
    c_start=corr([r["bias_start_x"] for r in q],[r["corrx"] for r in q])
    c_end=corr([r["bias_end_x"] for r in q],[r["corrx"] for r in q])
    # one-step lead: bias at interval k vs correction at k+1
    lead_x=[]; lead_y=[]
    lag_x=[]; lag_y=[]
    for leg in sorted(set(r["leg"] for r in q)):
        z=[r for r in q if r["leg"]==leg]
        z=sorted(z,key=lambda r:r["idx"])
        for a,b in zip(z[:-1],z[1:]):
            lead_x.append(a["bias_start_x"]); lead_y.append(b["corrx"])
            lag_x.append(b["bias_end_x"]); lag_y.append(a["corrx"])
    c_lead=corr(lead_x,lead_y)
    c_lag=corr(lag_x,lag_y)
    print(f"{d}: current_start→corr={c_start:+.3f}"
          f" current_end→corr={c_end:+.3f}"
          f" bias(k)→corr(k+1)={c_lead:+.3f}"
          f" corr(k)→bias(k+1)={c_lag:+.3f}")

print("\nFIRST 5 EXACT INTERVALS OF EACH PASS")
print("-"*170)
for leg in range(1,5):
    q=sorted([r for r in rows if r["leg"]==leg],key=lambda r:r["idx"])[:5]
    if not q:continue
    print(f"PASS {leg} {q[0]['direction']}")
    print(" idx  prepass_bX  start_bX   end_bX    corrX_mm")
    for r in q:
        print(f" {r['idx']:3d}  {r['bias_prepass_x']:+.5f}   {r['bias_start_x']:+.5f}  {r['bias_end_x']:+.5f}   {r['corrx']:+.3f}")

print("\nORDER / CAUSALITY VERDICT AIDS")
print("-"*170)
# Compare first-quarter vs last-quarter correlations, and how quickly bias moves from prepass value.
for d in ("A_TO_B","B_TO_A"):
    q=[r for r in rows if r["direction"]==d]
    early=[]; late=[]
    for leg in sorted(set(r["leg"] for r in q)):
        z=sorted([r for r in q if r["leg"]==leg],key=lambda r:r["idx"])
        n=len(z)
        early += z[:max(1,n//4)]
        late += z[-max(1,n//4):]
    print(f"{d}: early bias→corr={corr([r['bias_start_x'] for r in early],[r['corrx'] for r in early]):+.3f}"
          f"  late bias→corr={corr([r['bias_start_x'] for r in late],[r['corrx'] for r in late]):+.3f}"
          f"  early mean |bias-prepass|={mean([abs(r['bias_start_x']-r['bias_prepass_x']) for r in early]):.5f} m/s²"
          f"  late mean |bias-prepass|={mean([abs(r['bias_start_x']-r['bias_prepass_x']) for r in late]):.5f} m/s²")

print("\nINTERPRETATION RULES")
print("-"*170)
print("1) Strong prepass bias + strong bias(k)->corr(k+1) in both directions: bias is plausibly upstream/casual.")
print("2) Weak prepass relation but strong corr(k)->bias(k+1): bias is more likely optimizer response.")
print("3) If bias is already direction-dependent before START, inspect initialization / carry-over between passes.")
print("4) If bias changes mainly after motion begins, inspect observability and bias estimation during visual-inertial fusion.")
print("5) This test still cannot prove causality mathematically; it is a temporal-order screen using existing logs.")
