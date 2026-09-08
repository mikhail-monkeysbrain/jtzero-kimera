#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics
from collections import defaultdict

RUNS=[
 ("FAST_5S",Path("/home/vio/jtzero_runs/20260908_200424_v25_CONTROLLED_AB4_FAST_5S")),
 ("MEDIUM_7P5S",Path("/home/vio/jtzero_runs/20260908_195640_v25_CONTROLLED_AB4_REPEATABILITY")),
 ("SLOW_10S",Path("/home/vio/jtzero_runs/20260908_200841_v25_CONTROLLED_AB4_SLOW_10S")),
]
root=Path("/home/vio/jtzero_runs")
v27=sorted(root.glob("*_v25_TBS_ROLD_7P5")) or sorted(root.glob("*TBS_ROLD_7P5*"))
if v27: RUNS.append(("V27_ROLD",v27[-1]))

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
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<4:return float("nan")
    xa=[a for a,b in p]; ya=[b for a,b in p]; mx=mean(xa); my=mean(ya)
    sx=sum((a-mx)**2 for a in xa); sy=sum((b-my)**2 for b in ya)
    if sx<=0 or sy<=0:return float("nan")
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy)
def read(p):
    with p.open() as fh:return list(csv.DictReader(fh))
def norm3(x,y,z):return math.sqrt(x*x+y*y+z*z)

records=[]
for mode,R in RUNS:
    if not R.exists(): continue
    backend=read(R/"jtzero_500mm_v25_backend.csv")
    front=read(R/"jtzero_500mm_v25_frontend.csv")
    events=read(R/"jtzero_500mm_v25_events.csv")
    E=defaultdict(dict)
    for e in events:
        n=ii(e.get("leg"))
        if 1<=n<=4:E[n][e.get("event","")]=e

    # frontend by timestamp: Kimera callbacks use the same state timestamp grid often enough;
    # otherwise nearest within 80 ms is accepted.
    ftimes=[ii(r["timestamp_ns"]) for r in front]
    def near_front(ts):
        if not front:return None
        lo=0; hi=len(ftimes)
        while lo<hi:
            m=(lo+hi)//2
            if ftimes[m]<ts: lo=m+1
            else: hi=m
        cand=[]
        for j in (lo-1,lo,lo+1):
            if 0<=j<len(front): cand.append(front[j])
        if not cand:return None
        r=min(cand,key=lambda x:abs(ii(x["timestamp_ns"])-ts))
        return r if abs(ii(r["timestamp_ns"])-ts)<=80_000_000 else None

    for leg in range(1,5):
        if "START" not in E[leg] or "END" not in E[leg]: continue
        t0=ii(E[leg]["START"]["state_timestamp_ns"]); t1=ii(E[leg]["END"]["state_timestamp_ns"])
        br=[r for r in backend if t0<=ii(r["timestamp_ns"])<=t1]
        if len(br)<3: continue
        direction="A_TO_B" if leg%2 else "B_TO_A"
        x0,y0=f(br[0]["px_m"]),f(br[0]["py_m"])
        final_h=math.hypot(f(br[-1]["px_m"])-x0,f(br[-1]["py_m"])-y0)*1000.0
        err=final_h-500.0

        step_back=[]; step_pim_dp=[]; step_pim_dv=[]
        ratios=[]; abs_res=[]; valid_steps=0; visual_valid=[]
        first_half=[]; second_half=[]
        for a,b in zip(br[:-1],br[1:]):
            dx=f(b["px_m"])-f(a["px_m"]); dy=f(b["py_m"])-f(a["py_m"]); dz=f(b["pz_m"])-f(a["pz_m"])
            bd=norm3(dx,dy,dz)
            fr=near_front(ii(b["timestamp_ns"]))
            if fr is None or ii(fr.get("pim_valid",0))!=1: continue
            pd=norm3(f(fr["pim_dpx"]),f(fr["pim_dpy"]),f(fr["pim_dpz"]))
            pv=norm3(f(fr["pim_dvx"]),f(fr["pim_dvy"]),f(fr["pim_dvz"]))
            if not (math.isfinite(bd) and math.isfinite(pd)): continue
            valid_steps+=1
            step_back.append(bd); step_pim_dp.append(pd); step_pim_dv.append(pv)
            ratios.append(bd/pd if pd>1e-9 else float("nan"))
            abs_res.append(abs(bd-pd))
            visual_valid.append(f(fr.get("mono_pose_valid","nan")))
            frac=(ii(b["timestamp_ns"])-t0)/max(1,(t1-t0))
            (first_half if frac<0.5 else second_half).append((bd,pd,pv,abs(bd-pd)))

        def sm(arr,idx):return mean([x[idx] for x in arr])
        records.append({
          "mode":mode,"leg":leg,"direction":direction,"error_mm":err,"horizontal_mm":final_h,
          "n":valid_steps,
          "backend_step_mean_mm":mean(step_back)*1000,
          "pim_dp_mean_mm":mean(step_pim_dp)*1000,
          "pim_dv_mean":mean(step_pim_dv),
          "ratio_backend_to_pim":mean(ratios),
          "abs_step_residual_mm":mean(abs_res)*1000,
          "sum_abs_step_residual_mm":sum(x for x in abs_res if math.isfinite(x))*1000,
          "first_abs_res_mm":sm(first_half,3)*1000,
          "second_abs_res_mm":sm(second_half,3)*1000,
          "first_pim_dp_mm":sm(first_half,1)*1000,
          "second_pim_dp_mm":sm(second_half,1)*1000,
          "first_pim_dv":sm(first_half,2),
          "second_pim_dv":sm(second_half,2),
          "mono_valid_at_pim":mean(visual_valid),
        })

print("="*146)
print("PIM ↔ BACKEND CONSISTENCY — 16 CONTROLLED PASSES")
print("="*146)
print("PIM = IMU-предсказание движения между соседними состояниями Kimera.")
print("Сравниваются МОДУЛИ перемещений, поэтому мы не предполагаем совпадение систем координат PIM и backend.")
print("residual = |модуль шага backend − модуль PIM-предсказания|. Это диагностический показатель, не физическая ошибка расстояния.")
print()
print("MODE          LEG DIR      ERR_mm   N   Bstep_mm  PIMdp_mm  B/PIM  RES_mm  SUM_RES  RES_1st RES_2nd PIMdv_1 PIMdv_2")
for r in records:
    print(f"{r['mode']:<13} {r['leg']:>3} {r['direction']:<6} {r['error_mm']:+8.2f} {r['n']:3d}"
          f" {r['backend_step_mean_mm']:9.2f} {r['pim_dp_mean_mm']:9.2f} {r['ratio_backend_to_pim']:6.3f}"
          f" {r['abs_step_residual_mm']:7.2f} {r['sum_abs_step_residual_mm']:8.1f}"
          f" {r['first_abs_res_mm']:8.2f} {r['second_abs_res_mm']:8.2f}"
          f" {r['first_pim_dv']:7.3f} {r['second_pim_dv']:7.3f}")

features=[k for k in records[0] if k not in ("mode","leg","direction","error_mm","horizontal_mm","n")]
print("\n"+"="*146)
print("CORRELATION WITH FINAL SIGNED ERROR")
print("="*146)
for k in features:
    c=corr([r[k] for r in records],[r["error_mm"] for r in records])
    print(f"{k:<34} {c:+.3f}")

print("\n"+"="*146)
print("WITHIN-DIRECTION CHECK")
print("="*146)
print("FEATURE                            corr_A   corr_B   same_sign   conservative")
rank=[]
for k in features:
    A=[r for r in records if r["direction"]=="A_TO_B"]; B=[r for r in records if r["direction"]=="B_TO_A"]
    ca=corr([r[k] for r in A],[r["error_mm"] for r in A])
    cb=corr([r[k] for r in B],[r["error_mm"] for r in B])
    same=math.isfinite(ca) and math.isfinite(cb) and ca*cb>0
    score=min(abs(ca),abs(cb)) if same else 0.0
    rank.append((score,k,ca,cb,same))
for score,k,ca,cb,same in sorted(rank,reverse=True):
    print(f"{k:<34} {ca:+7.3f} {cb:+7.3f} {str(same):>10} {score:12.3f}")

print("\n"+"="*146)
print("VERDICT RULES")
print("="*146)
print("1) If PIM means correlate but backend/PIM residuals do NOT: PIM likely tracks motion profile, not the error source.")
print("2) If residuals correlate with same sign in both directions: inspect IMU↔visual fusion / preintegration consistency next.")
print("3) If only second-half residual correlates: error is created mainly near braking/end of motion.")
print("4) If ratio_backend_to_pim changes systematically with final error: suspect scale transfer inside fusion.")
