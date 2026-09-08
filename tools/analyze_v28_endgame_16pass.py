#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics
from collections import defaultdict

RUNS=[
 ("FAST_5S",Path("/home/vio/jtzero_runs/20260908_200424_v25_CONTROLLED_AB4_FAST_5S")),
 ("MEDIUM_7P5S",Path("/home/vio/jtzero_runs/20260908_195640_v25_CONTROLLED_AB4_REPEATABILITY")),
 ("SLOW_10S",Path("/home/vio/jtzero_runs/20260908_200841_v25_CONTROLLED_AB4_SLOW_10S")),
]
ROOT=Path("/home/vio/jtzero_runs")
v27=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5")) or sorted(ROOT.glob("*TBS_ROLD_7P5*"))
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
def rms(v):
    v=[x for x in v if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in v)/len(v)) if v else float("nan")
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<4:return float("nan")
    x=[a for a,b in p]; y=[b for a,b in p]; mx=mean(x); my=mean(y)
    sx=sum((a-mx)**2 for a in x); sy=sum((b-my)**2 for b in y)
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy) if sx>0 and sy>0 else float("nan")
def read(p):
    if not p.exists():return []
    with p.open() as fh:return list(csv.DictReader(fh))
def sl(rows,a,b,key):
    return [r for r in rows if a <= ii(r.get(key,0)) <= b]
def nearest_before(rows,t,key):
    rr=[r for r in rows if ii(r.get(key,0))<=t]
    return rr[-1] if rr else None
def hypot3(a,b,c):return math.sqrt(a*a+b*b+c*c)

records=[]
for mode,R in RUNS:
    backend=read(R/"jtzero_500mm_v25_backend.csv")
    front=read(R/"jtzero_500mm_v25_frontend.csv")
    events=read(R/"jtzero_500mm_v25_events.csv")
    imu=read(R/"jtzero_500mm_v25.csv")
    if not (backend and front and events and imu): continue
    E=defaultdict(dict)
    for e in events:
        n=ii(e.get("leg"))
        if 1<=n<=4:E[n][e.get("event","")]=e
    for n in range(1,5):
        if "START" not in E[n] or "END" not in E[n]:continue
        e0,e1=E[n]["START"],E[n]["END"]
        t0=ii(e0["state_timestamp_ns"]); t1=ii(e1["state_timestamp_ns"])
        b=sl(backend,t0,t1,"timestamp_ns")
        if len(b)<2:continue
        x0,y0=f(b[0]["px_m"]),f(b[0]["py_m"])
        x1,y1=f(b[-1]["px_m"]),f(b[-1]["py_m"])
        h=math.hypot(x1-x0,y1-y0)*1000
        sign=1 if n%2 else -1
        rec={"mode":mode,"leg":n,"direction":"A_TO_B" if n%2 else "B_TO_A",
             "error_mm":h-500.0,"horizontal_mm":h,
             "end_press_speed":hypot3(f(e1["vx_m_s"]),f(e1["vy_m_s"]),f(e1["vz_m_s"]))}
        for win in (0.5,1.0,2.0):
            a=t1-int(win*1e9)
            bw=sl(backend,a,t1,"timestamp_ns")
            iw=sl(imu,a,t1,"mapped_ns")
            fw=sl(front,a,t1,"timestamp_ns")
            if len(bw)>=2:
                dx=sign*(f(bw[-1]["px_m"])-f(bw[0]["px_m"]))*1000
                dy=sign*(f(bw[-1]["py_m"])-f(bw[0]["py_m"]))*1000
                rec[f"last{win:g}_dx_mm"]=dx
                rec[f"last{win:g}_dy_mm"]=dy
                rec[f"last{win:g}_vmean"]=mean([f(r["speed_m_s"]) for r in bw])
                rec[f"last{win:g}_vstart"]=f(bw[0]["speed_m_s"])
                rec[f"last{win:g}_vend"]=f(bw[-1]["speed_m_s"])
                rec[f"last{win:g}_dv_backend"]=f(bw[-1]["speed_m_s"])-f(bw[0]["speed_m_s"])
                dbax=f(bw[-1]["bax"])-f(bw[0]["bax"]); dbay=f(bw[-1]["bay"])-f(bw[0]["bay"]); dbaz=f(bw[-1]["baz"])-f(bw[0]["baz"])
                rec[f"last{win:g}_dba"]=hypot3(dbax,dbay,dbaz)
            else:
                for k in ("dx_mm","dy_mm","vmean","vstart","vend","dv_backend","dba"):rec[f"last{win:g}_{k}"]=float("nan")
            ah=[math.hypot(f(r["ax"]),f(r["ay"])) for r in iw]
            gn=[hypot3(f(r["gx"]),f(r["gy"]),f(r["gz"])) for r in iw]
            rec[f"last{win:g}_hacc_rms"]=rms(ah)
            rec[f"last{win:g}_gyro_rms"]=rms(gn)
            rec[f"last{win:g}_inlier"]=mean([f(r["mono_inlier_ratio"]) for r in fw])
            rec[f"last{win:g}_tracked"]=mean([f(r["tracked_features"]) for r in fw])
            rec[f"last{win:g}_mono_valid"]=mean([f(r["mono_pose_valid"]) for r in fw])
            pim=[r for r in fw if ii(r.get("pim_valid",0))==1]
            rec[f"last{win:g}_pim_dp"]=mean([hypot3(f(r["pim_dpx"]),f(r["pim_dpy"]),f(r["pim_dpz"])) for r in pim])
            rec[f"last{win:g}_pim_dv"]=mean([hypot3(f(r["pim_dvx"]),f(r["pim_dvy"]),f(r["pim_dvz"])) for r in pim])
        records.append(rec)

print("="*150)
print("ENDGAME FORENSIC — 16 CONTROLLED PASSES, LAST 0.5 / 1 / 2 SECONDS BEFORE END PRESS")
print("="*150)
print("PIM = IMU-предсказание перемещения/скорости между соседними состояниями Kimera.")
print("dBA = изменение оценки постоянной ошибки акселерометра внутри окна.")
print()
print("MODE          LEG DIR     ERR_mm END_V  L0.5_X  L1_X   L2_X   L1_dV  L1_HACC L1_GYRO L1_INL L1_TRK L1_dBA")
for r in records:
    print(f"{r['mode']:<13} {r['leg']:>3} {r['direction']:<6} {r['error_mm']:+8.2f} {r['end_press_speed']:6.3f}"
          f" {r['last0.5_dx_mm']:7.1f} {r['last1_dx_mm']:7.1f} {r['last2_dx_mm']:7.1f}"
          f" {r['last1_dv_backend']:+7.3f} {r['last1_hacc_rms']:8.4f} {r['last1_gyro_rms']:8.5f}"
          f" {r['last1_inlier']:6.3f} {r['last1_tracked']:6.1f} {r['last1_dba']:7.4f}")

features=[k for k in records[0] if k not in ("mode","leg","direction","error_mm","horizontal_mm")]
print("\n"+"="*150)
print("CORRELATION WITH FINAL SIGNED ERROR")
print("="*150)
rank=[]
for k in features:
    c=corr([r[k] for r in records],[r["error_mm"] for r in records])
    if math.isfinite(c):rank.append((abs(c),k,c))
for _,k,c in sorted(rank,reverse=True)[:30]:
    print(f"{k:<34} corr={c:+.3f}")

print("\n"+"="*150)
print("WITHIN-DIRECTION CONSISTENCY")
print("="*150)
rank2=[]
for k in features:
    ca=corr([r[k] for r in records if r["direction"]=="A_TO_B"],[r["error_mm"] for r in records if r["direction"]=="A_TO_B"])
    cb=corr([r[k] for r in records if r["direction"]=="B_TO_A"],[r["error_mm"] for r in records if r["direction"]=="B_TO_A"])
    if math.isfinite(ca) and math.isfinite(cb):
        same=ca*cb>0
        score=min(abs(ca),abs(cb)) if same else 0
        rank2.append((score,same,k,ca,cb))
print("FEATURE                            same_sign  corr_A  corr_B  score")
for score,same,k,ca,cb in sorted(rank2,reverse=True)[:25]:
    print(f"{k:<34} {str(same):>9} {ca:+7.3f} {cb:+7.3f} {score:6.3f}")

print("\n"+"="*150)
print("INTERPRETATION TARGETS")
print("="*150)
print("H1 braking speed: end_press_speed / last*_vend / last*_dv_backend")
print("H2 braking IMU excitation: last*_hacc_rms / last*_gyro_rms")
print("H3 frontend degradation: last*_inlier / tracked / mono_valid")
print("H4 accelerometer offset adaptation: last*_dba")
print("H5 IMU prediction issue: last*_pim_dp / last*_pim_dv")
print("H6 amount of motion left near endpoint: last*_dx_mm")
print("A candidate is strong only if correlation keeps the SAME SIGN in A→B and B→A.")
