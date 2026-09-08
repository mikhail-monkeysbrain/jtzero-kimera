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
def norm(v): return math.sqrt(sum(x*x for x in v))
def add(a,b): return tuple(x+y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mul(a,s): return tuple(x*s for x in a)
def matvec(M,v): return tuple(sum(M[r][c]*v[c] for c in range(3)) for r in range(3))
def matmul(A,B): return tuple(tuple(sum(A[r][k]*B[k][c] for k in range(3)) for c in range(3)) for r in range(3))
def rpy_matrix(rdeg,pdeg,ydeg):
    r,p,y=map(math.radians,(rdeg,pdeg,ydeg))
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr))
    Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    Rz=((cy,-sy,0),(sy,cy,0),(0,0,1))
    return matmul(Rz,matmul(Ry,Rx))
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<4:return float("nan")
    X=[a for a,b in p];Y=[b for a,b in p]
    mx,my=mean(X),mean(Y)
    sx=sum((a-mx)**2 for a in X); sy=sum((b-my)**2 for b in Y)
    if sx<=0 or sy<=0:return float("nan")
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy)
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
    if 1<=n<=4:
        E[n][e.get("event","")]=e

rows=[]
for leg in range(1,5):
    if "START" not in E[leg] or "END" not in E[leg]: continue
    t0=ii(E[leg]["START"]["state_timestamp_ns"]); t1=ii(E[leg]["END"]["state_timestamp_ns"])
    br=[r for r in backend if t0<=ii(r["timestamp_ns"])<=t1]
    sign=1 if leg%2 else -1
    direction="A_TO_B" if leg%2 else "B_TO_A"

    for a,b in zip(br[:-1],br[1:]):
        tj=ii(b["timestamp_ns"]); fr=F.get(tj)
        if fr is None: continue
        dt=(tj-ii(a["timestamp_ns"]))/1e9
        dtp=f(fr["pim_dt_s"])
        if not (0<dt<1 and abs(dt-dtp)<=0.002): continue

        Pi=(f(a["px_m"]),f(a["py_m"]),f(a["pz_m"]))
        Vi=(f(a["vx_m_s"]),f(a["vy_m_s"]),f(a["vz_m_s"]))
        Pj=(f(b["px_m"]),f(b["py_m"]),f(b["pz_m"]))
        Vj=(f(b["vx_m_s"]),f(b["vy_m_s"]),f(b["vz_m_s"]))
        Ri=rpy_matrix(f(a["roll_deg"]),f(a["pitch_deg"]),f(a["yaw_deg"]))

        dp=(f(fr["pim_dpx"]),f(fr["pim_dpy"]),f(fr["pim_dpz"]))
        dv=(f(fr["pim_dvx"]),f(fr["pim_dvy"]),f(fr["pim_dvz"]))

        gw=(0,0,-G)
        p_vel=mul(Vi,dt)
        p_grav=mul(gw,0.5*dt*dt)
        p_pim=matvec(Ri,dp)
        Ppred=add(Pi,add(add(p_vel,p_grav),p_pim))
        Vpred=add(Vi,add(mul(gw,dt),matvec(Ri,dv)))

        corrP=sub(Pj,Ppred)
        corrV=sub(Vj,Vpred)

        ba=(f(a["bax"]),f(a["bay"]),f(a["baz"]))
        bb=(f(b["bax"]),f(b["bay"]),f(b["baz"]))
        dba=sub(bb,ba)
        ba_world=matvec(Ri,ba)
        dba_world=matvec(Ri,dba)

        # first-order expected displacement contribution from accel bias on this interval
        # sign convention is diagnostic only; compare both signed correlation and magnitude.
        bias_pos=mul(ba_world,0.5*dt*dt)
        dbias_pos=mul(dba_world,0.5*dt*dt)

        opt_delta=sub(Pj,Pi)
        vel_delta=sub(Vj,Vi)

        rows.append({
            "leg":leg,"direction":direction,"dt":dt,
            "corrx":sign*corrP[0]*1000,
            "corrp":norm(corrP)*1000,
            "corrv":norm(corrV),
            "vstart_x":sign*Vi[0]*1000,
            "vend_x":sign*Vj[0]*1000,
            "dvx":sign*vel_delta[0]*1000,
            "vel_term_x":sign*p_vel[0]*1000,
            "grav_term_x":sign*p_grav[0]*1000,
            "pim_term_x":sign*p_pim[0]*1000,
            "ba_x_world":sign*ba_world[0],
            "dba_x_world":sign*dba_world[0],
            "bias_pos_x_mm":sign*bias_pos[0]*1000,
            "dbias_pos_x_mm":sign*dbias_pos[0]*1000,
            "bax":f(a["bax"]),"bay":f(a["bay"]),"baz":f(a["baz"]),
            "inlier":f(fr.get("mono_inlier_ratio","nan")),
            "optx":sign*opt_delta[0]*1000,
        })

print("="*170)
print("V31 — EXACT-INTERVAL BIAS / GRAVITY / VELOCITY SCREEN")
print("="*170)
print(f"RUN: {R}")
print("Все показатели привязаны к тем же exact KF i→j интервалам, что и V30.")
print("bias = внутренняя оценка постоянной ошибки акселерометра Kimera.")
print("bax/bay/baz = оценка этой постоянной ошибки по осям X/Y/Z.")
print("bias_pos_x_mm = первый порядок вклада текущей оценки bias в перемещение за один интервал: 0.5*bias*dt².")
print("Это диагностическая оценка порядка величины, не точная реконструкция внутреннего Jacobian оптимизатора.\n")

print("DIRECTION SUMMARY")
print("-"*170)
for d in ("A_TO_B","B_TO_A"):
    q=[r for r in rows if r["direction"]==d]
    print(f"{d}: n={len(q)} corrX={mean([r['corrx'] for r in q]):+.3f} mm"
          f"  |corrP|={mean([r['corrp'] for r in q]):.3f} mm"
          f"  baX_world={mean([r['ba_x_world'] for r in q]):+.5f} m/s²"
          f"  dbaX_world={mean([r['dba_x_world'] for r in q]):+.5f} m/s²"
          f"  bias_posX={mean([r['bias_pos_x_mm'] for r in q]):+.4f} mm"
          f"  dvX={mean([r['dvx'] for r in q]):+.3f} mm/s")

print("\nCORRELATION WITH FUSION CORRECTION X")
print("-"*170)
features=[
 ("ba_x_world","оценка постоянной ошибки акселерометра вдоль движения"),
 ("dba_x_world","изменение этой оценки между состояниями"),
 ("bias_pos_x_mm","порядок вклада bias в положение"),
 ("dbias_pos_x_mm","порядок вклада изменения bias в положение"),
 ("vstart_x","скорость в начале интервала"),
 ("vend_x","скорость в конце интервала"),
 ("dvx","изменение скорости backend"),
 ("vel_term_x","член Vi*dt"),
 ("grav_term_x","член 0.5*g*dt² по X"),
 ("pim_term_x","PIM-вклад Ri*deltaP по X"),
 ("inlier","доля правильных визуальных соответствий"),
]
for k,desc in features:
    ca=corr([r[k] for r in rows if r["direction"]=="A_TO_B"],[r["corrx"] for r in rows if r["direction"]=="A_TO_B"])
    cb=corr([r[k] for r in rows if r["direction"]=="B_TO_A"],[r["corrx"] for r in rows if r["direction"]=="B_TO_A"])
    same=math.isfinite(ca) and math.isfinite(cb) and ca*cb>0
    score=min(abs(ca),abs(cb)) if same else 0
    print(f"{k:18s} corr_A={ca:+.3f} corr_B={cb:+.3f} same_sign={str(same):5s} conservative={score:.3f}  # {desc}")

print("\nPASS SUMMARY")
print("-"*170)
for leg in range(1,5):
    q=[r for r in rows if r["leg"]==leg]
    if not q: continue
    print(f"PASS {leg} {q[0]['direction']}:"
          f" corrX={mean([r['corrx'] for r in q]):+.3f}mm/int"
          f" baX={mean([r['ba_x_world'] for r in q]):+.5f}m/s²"
          f" dbaX={mean([r['dba_x_world'] for r in q]):+.5f}m/s²"
          f" bias_posX={mean([r['bias_pos_x_mm'] for r in q]):+.4f}mm/int"
          f" pimX={mean([r['pim_term_x'] for r in q]):+.3f}mm/int"
          f" velX={mean([r['vel_term_x'] for r in q]):+.3f}mm/int")

print("\nORDER-OF-MAGNITUDE CHECK")
print("-"*170)
mc=mean([abs(r["corrx"]) for r in rows])
mb=mean([abs(r["bias_pos_x_mm"]) for r in rows])
mdb=mean([abs(r["dbias_pos_x_mm"]) for r in rows])
print(f"mean |fusion correction X| = {mc:.3f} mm / interval")
print(f"mean |bias first-order X|  = {mb:.4f} mm / interval")
print(f"mean |Δbias first-order X| = {mdb:.4f} mm / interval")
if math.isfinite(mc) and math.isfinite(mb) and mb < 0.2*mc:
    print("Magnitude verdict: accelerometer-bias position term is too small to directly explain most of corrX.")
else:
    print("Magnitude verdict: accelerometer bias remains large enough to inspect as a direct contributor.")

print("\nINTERPRETATION RULES")
print("-"*170)
print("1) If ba_x_world or bias_pos_x_mm has strong same-sign correlation in both directions AND comparable magnitude, bias is a leading candidate.")
print("2) If bias magnitude is far below corrX, bias alone cannot directly create the observed per-interval correction.")
print("3) grav_term_x should be ~0 in world X. A nonzero systematic value indicates a coordinate/orientation implementation problem in this reconstruction.")
print("4) If pim_term_x / velocity terms track corrX strongly in both directions, inspect IMU preintegration / state propagation.")
print("5) If only inlier tracks corrX, visual update quality remains candidate; otherwise frontend quality is further weakened.")
