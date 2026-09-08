#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics
from collections import defaultdict

ROOT=Path("/home/vio/jtzero_runs")
cand=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5")) or sorted(ROOT.glob("*TBS_ROLD_7P5*"))
if not cand:
    raise SystemExit("Не найден архив V27 TBS_ROLD_7P5")
R=cand[-1]

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
events=read("jtzero_500mm_v25_events.csv")
imu=read("jtzero_500mm_v25.csv")

E=defaultdict(dict)
for e in events:
    n=ii(e.get("leg"))
    if 1<=n<=4:E[n][e.get("event","")]=e

# backend bias transformed into world coordinates WITHOUT canonical direction sign.
B=[]
for r in backend:
    Rw=rpy_matrix(f(r["roll_deg"]),f(r["pitch_deg"]),f(r["yaw_deg"]))
    bb=(f(r["bax"]),f(r["bay"]),f(r["baz"]))
    bw=matvec(Rw,bb)
    B.append({
        "t":ii(r["timestamp_ns"]),"kf":ii(r["keyframe"]),
        "bwx":bw[0],"bwy":bw[1],"bwz":bw[2],
        "bax":bb[0],"bay":bb[1],"baz":bb[2],
        "vx":f(r["vx_m_s"]),"vy":f(r["vy_m_s"]),"vz":f(r["vz_m_s"]),
        "px":f(r["px_m"]),"py":f(r["py_m"]),"pz":f(r["pz_m"]),
    })

def nearest_before(t):
    q=[r for r in B if r["t"]<=t]
    return q[-1] if q else None
def nearest_after(t):
    q=[r for r in B if r["t"]>=t]
    return q[0] if q else None
def window(t0,t1):
    return [r for r in B if t0<=r["t"]<=t1]
def imu_window(t0,t1):
    # mapped_ns is in the same local clock domain as backend timestamp_ns.
    out=[]
    for r in imu:
        tm=ii(r.get("mapped_ns"))
        if t0<=tm<=t1:
            out.append((f(r["ax"]),f(r["ay"]),f(r["az"])))
    return out

print("="*176)
print("V33 — ACCELEROMETER BIAS CARRY-OVER / STATIONARY EVOLUTION")
print("="*176)
print(f"RUN: {R}")
print("Здесь НЕ меняется знак X для B→A. Все bias-векторы показаны в одной мировой системе координат.")
print("bias = внутренняя оценка постоянной ошибки акселерометра Kimera.")
print("Цель: проверить, переносится ли одна и та же оценка bias из предыдущего прохода в следующий и меняется ли она во время неподвижных пауз.\n")

print("PASS BOUNDARIES — WORLD-FRAME BIAS")
print("-"*176)
print("PASS DIR     BEFORE_START_X  START_X    END_X    AFTER_END_X   DELTA_PASS_X  SPEED_START  SPEED_END")
pass_info=[]
for leg in range(1,5):
    if "START" not in E[leg] or "END" not in E[leg]:continue
    ts=ii(E[leg]["START"]["state_timestamp_ns"]); te=ii(E[leg]["END"]["state_timestamp_ns"])
    bs=nearest_before(ts); es=nearest_before(te)
    ba=nearest_after(te)
    # before start: last state at least 0.5 s before start when possible
    pre=nearest_before(ts-int(0.5e9)) or bs
    direction="A_TO_B" if leg%2 else "B_TO_A"
    d=(es["bwx"]-bs["bwx"]) if bs and es else float("nan")
    print(f"{leg:4d} {direction:7s} {pre['bwx']:+14.5f} {bs['bwx']:+9.5f} {es['bwx']:+9.5f} {ba['bwx']:+13.5f}"
          f" {d:+13.5f} {norm((bs['vx'],bs['vy'],bs['vz'])):11.4f} {norm((es['vx'],es['vy'],es['vz'])):9.4f}")
    pass_info.append((leg,ts,te,bs,es))

print("\nCARRY-OVER BETWEEN PASSES")
print("-"*176)
print("PAIR      END_prev_X  START_next_X  CHANGE_PAUSE_X  PAUSE_s")
for (leg1,ts1,te1,bs1,es1),(leg2,ts2,te2,bs2,es2) in zip(pass_info[:-1],pass_info[1:]):
    print(f"{leg1}->{leg2:<3d} {es1['bwx']:+11.5f} {bs2['bwx']:+12.5f} {bs2['bwx']-es1['bwx']:+14.5f} {(ts2-te1)/1e9:8.3f}")

print("\nSTATIONARY PAUSES")
print("-"*176)
print("PAUSE      Nstates  bwX_start   bwX_end    delta_bwX  mean_speed_mm_s  IMU_ax_mean  IMU_ay_mean  IMU_az_mean")
for i in range(len(pass_info)-1):
    leg1,ts1,te1,bs1,es1=pass_info[i]
    leg2,ts2,te2,bs2,es2=pass_info[i+1]
    q=window(te1,ts2)
    if not q:continue
    speeds=[norm((r["vx"],r["vy"],r["vz"]))*1000 for r in q]
    iw=imu_window(te1,ts2)
    ax=mean([a for a,b,c in iw]); ay=mean([b for a,b,c in iw]); az=mean([c for a,b,c in iw])
    print(f"{leg1}->{leg2:<3d} {len(q):7d} {q[0]['bwx']:+10.5f} {q[-1]['bwx']:+10.5f} {q[-1]['bwx']-q[0]['bwx']:+11.5f}"
          f" {mean(speeds):16.3f} {ax:+12.5f} {ay:+12.5f} {az:+12.5f}")

print("\nPASS INTERNAL EVOLUTION — QUARTILES")
print("-"*176)
for leg,ts,te,bs,es in pass_info:
    q=window(ts,te)
    if not q:continue
    print(f"PASS {leg} {'A_TO_B' if leg%2 else 'B_TO_A'}")
    for pct in (0,25,50,75,100):
        target=ts+(te-ts)*pct/100
        r=min(q,key=lambda x:abs(x["t"]-target))
        print(f"  {pct:3d}% KF={r['kf']:3d} bwX={r['bwx']:+.5f} bwY={r['bwy']:+.5f} bwZ={r['bwz']:+.5f}")

print("\nGLOBAL WORLD-BIAS RANGE")
print("-"*176)
print(f"world bias X: min={min(r['bwx'] for r in B):+.5f} max={max(r['bwx'] for r in B):+.5f} span={max(r['bwx'] for r in B)-min(r['bwx'] for r in B):.5f} m/s²")
print(f"world bias Y: min={min(r['bwy'] for r in B):+.5f} max={max(r['bwy'] for r in B):+.5f} span={max(r['bwy'] for r in B)-min(r['bwy'] for r in B):.5f} m/s²")
print(f"world bias Z: min={min(r['bwz'] for r in B):+.5f} max={max(r['bwz'] for r in B):+.5f} span={max(r['bwz'] for r in B)-min(r['bwz'] for r in B):.5f} m/s²")

print("\nINTERPRETATION")
print("-"*176)
print("1) Если END предыдущего прохода ≈ START следующего, bias переносится как состояние фильтра, а не переинициализируется.")
print("2) Если во время неподвижной паузы bias почти не возвращается к исходному значению, накопленное состояние сохраняется.")
print("3) Если world-frame bias X сохраняет один физический знак через смену A→B/B→A, прежняя 'direction dependence' была в основном эффектом приведения знака.")
print("4) Если pass 1 начинает около нуля, а последующие старты уже имеют большой |bias|, движение первого прохода создаёт/делает наблюдаемой эту оценку.")
print("5) Тогда следующий решающий A/B — reset/reinitialize bias между проходами против carry-over, при неизменных остальных параметрах.")
