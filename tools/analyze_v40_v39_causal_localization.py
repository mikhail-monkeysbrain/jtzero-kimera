#!/usr/bin/env python3
from pathlib import Path
from collections import defaultdict
import csv, math, statistics

ROOT = Path("/home/vio/jtzero_runs")
runs = sorted(ROOT.glob("*_v39_CLEAN_RESTART_CARRYOVER"))
if not runs:
    raise SystemExit("Не найден архив *_v39_CLEAN_RESTART_CARRYOVER")
R = runs[-1]

G = 9.81

def ff(x):
    try: return float(x)
    except: return float("nan")

def ii(x):
    try: return int(float(x))
    except: return 0

def finite(v):
    return [x for x in v if math.isfinite(x)]

def mean(v):
    q = finite(v)
    return statistics.mean(q) if q else float("nan")

def sd(v):
    q = finite(v)
    return statistics.pstdev(q) if len(q) > 1 else 0.0

def mae(v):
    return mean([abs(x) for x in v])

def corr(x, y):
    p = [(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p) < 4: return float("nan")
    X=[a for a,b in p]; Y=[b for a,b in p]
    mx=mean(X); my=mean(Y)
    sx=sum((a-mx)**2 for a in X); sy=sum((b-my)**2 for b in Y)
    if sx <= 0 or sy <= 0: return float("nan")
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy)

def norm3(v):
    return math.sqrt(sum(x*x for x in v))

def add(a,b): return tuple(x+y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mul(a,s): return tuple(x*s for x in a)
def matvec(M,v): return tuple(sum(M[r][c]*v[c] for c in range(3)) for r in range(3))
def matmul(A,B): return tuple(tuple(sum(A[r][k]*B[k][c] for k in range(3)) for c in range(3)) for r in range(3))

def rpy_matrix(roll_deg,pitch_deg,yaw_deg):
    r,p,y=[math.radians(v) for v in (roll_deg,pitch_deg,yaw_deg)]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr))
    Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    Rz=((cy,-sy,0),(sy,cy,0),(0,0,1))
    return matmul(Rz,matmul(Ry,Rx))

def read_csv(d, name):
    p=d/name
    if not p.exists(): return []
    with p.open() as f: return list(csv.DictReader(f))

def event_bounds(events, leg):
    e={}
    for r in events:
        if ii(r.get("leg"))==leg:
            e[r.get("event","")]=r
    if "START" not in e or "END" not in e:
        return None
    return ii(e["START"]["state_timestamp_ns"]), ii(e["END"]["state_timestamp_ns"])

def interp_at(rows, t):
    if not rows: return None
    return min(rows, key=lambda r: abs(ii(r["timestamp_ns"])-t))

def percentile_time_slices(t0,t1):
    dur=(t1-t0)/1e9
    return [
        ("0-0.5s", t0, min(t1,t0+int(0.5e9))),
        ("0.5-1s", min(t1,t0+int(0.5e9)), min(t1,t0+int(1e9))),
        ("1-2s", min(t1,t0+int(1e9)), min(t1,t0+int(2e9))),
        ("MID", t0+int(0.25*(t1-t0)), t0+int(0.75*(t1-t0))),
        ("LAST1s", max(t0,t1-int(1e9)), t1),
    ]

def phase_stats(backend, front, imu, t0, t1):
    b=[r for r in backend if t0 <= ii(r["timestamp_ns"]) <= t1]
    f=[r for r in front if t0 <= ii(r["timestamp_ns"]) <= t1]
    m=[r for r in imu if t0 <= ii(r.get("mapped_ns")) <= t1 and r.get("type")=="IMU"]
    if not b: return {}
    x0=ff(b[0]["px_m"]); y0=ff(b[0]["py_m"]); z0=ff(b[0]["pz_m"])
    x1=ff(b[-1]["px_m"]); y1=ff(b[-1]["py_m"]); z1=ff(b[-1]["pz_m"])
    inl=[ff(r.get("mono_inlier_ratio","nan")) for r in f if ii(r.get("mono_pose_valid",0))==1]
    tr=[ff(r.get("tracked_features","nan")) for r in f]
    ax=[ff(r["ax"]) for r in m]; ay=[ff(r["ay"]) for r in m]
    gx=[ff(r["gx"]) for r in m]; gy=[ff(r["gy"]) for r in m]; gz=[ff(r["gz"]) for r in m]
    hacc=[math.hypot(a,b) for a,b in zip(ax,ay)]
    gyro=[math.sqrt(a*a+b*b+c*c) for a,b,c in zip(gx,gy,gz)]
    return {
        "dx_mm":(x1-x0)*1000, "dy_mm":(y1-y0)*1000, "dz_mm":(z1-z0)*1000,
        "speed_mean":mean([ff(r["speed_m_s"]) for r in b]),
        "bx_mean":mean([ff(r["bax"]) for r in b]),
        "by_mean":mean([ff(r["bay"]) for r in b]),
        "bz_mean":mean([ff(r["baz"]) for r in b]),
        "inlier":mean(inl), "tracked":mean(tr),
        "hacc_rms":math.sqrt(mean([x*x for x in hacc])) if hacc else float("nan"),
        "gyro_rms":math.sqrt(mean([x*x for x in gyro])) if gyro else float("nan"),
        "mono_valid":mean([1.0 if ii(r.get("mono_pose_valid",0)) else 0.0 for r in f]),
    }

def exact_fusion(backend, front, t0, t1):
    B=[r for r in backend if t0 <= ii(r["timestamp_ns"]) <= t1]
    F={ii(r["timestamp_ns"]):r for r in front if ii(r.get("pim_valid",0))==1}
    out=[]
    for a,b in zip(B[:-1],B[1:]):
        tj=ii(b["timestamp_ns"]); fr=F.get(tj)
        if fr is None: continue
        dtb=(tj-ii(a["timestamp_ns"]))/1e9
        dtp=ff(fr.get("pim_dt_s","nan"))
        if not (0<dtb<1 and 0<dtp<1) or abs(dtb-dtp)>0.002: continue
        Pi=(ff(a["px_m"]),ff(a["py_m"]),ff(a["pz_m"]))
        Vi=(ff(a["vx_m_s"]),ff(a["vy_m_s"]),ff(a["vz_m_s"]))
        Pj=(ff(b["px_m"]),ff(b["py_m"]),ff(b["pz_m"]))
        Vj=(ff(b["vx_m_s"]),ff(b["vy_m_s"]),ff(b["vz_m_s"]))
        Ri=rpy_matrix(ff(a["roll_deg"]),ff(a["pitch_deg"]),ff(a["yaw_deg"]))
        dp=(ff(fr["pim_dpx"]),ff(fr["pim_dpy"]),ff(fr["pim_dpz"]))
        dv=(ff(fr["pim_dvx"]),ff(fr["pim_dvy"]),ff(fr["pim_dvz"]))
        gw=(0.0,0.0,-G)
        Ppred=add(add(add(Pi,mul(Vi,dtp)),mul(gw,0.5*dtp*dtp)),matvec(Ri,dp))
        Vpred=add(add(Vi,mul(gw,dtp)),matvec(Ri,dv))
        corrP=sub(Pj,Ppred); corrV=sub(Vj,Vpred)
        out.append({
            "t":tj,
            "corrx_mm":corrP[0]*1000,
            "corrp_mm":norm3(corrP)*1000,
            "corrv":norm3(corrV),
            "pim_dp_mm":norm3(dp)*1000,
            "pim_dv":norm3(dv),
            "inlier":ff(fr.get("mono_inlier_ratio","nan")),
        })
    return out

def build_pass(label, d, leg):
    backend=read_csv(d,"jtzero_500mm_v25_backend.csv")
    front=read_csv(d,"jtzero_500mm_v25_frontend.csv")
    events=read_csv(d,"jtzero_500mm_v25_events.csv")
    imu=read_csv(d,"jtzero_500mm_v25.csv")
    camera=read_csv(d,"jtzero_500mm_v25_camera.csv")
    bounds=event_bounds(events,leg)
    if not bounds: return None
    t0,t1=bounds
    b=[r for r in backend if t0 <= ii(r["timestamp_ns"]) <= t1]
    if len(b)<2: return None
    p0=b[0]; p1=b[-1]
    horiz=math.hypot(ff(p1["px_m"])-ff(p0["px_m"]), ff(p1["py_m"])-ff(p0["py_m"]))*1000
    err=horiz-500.0
    phases={}
    for name,a,z in percentile_time_slices(t0,t1):
        if z>a: phases[name]=phase_stats(backend,front,imu,a,z)
    ex=exact_fusion(backend,front,t0,t1)
    cam=[r for r in camera if t0 <= ii(r.get("corrected_timestamp_ns")) <= t1 and ii(r.get("selected",0))==1]
    cts=sorted(ii(r["corrected_timestamp_ns"]) for r in cam)
    gaps=[(b-a)/1e6 for a,b in zip(cts[:-1],cts[1:])]
    return {
        "label":label,"dir":d,"leg":leg,"t0":t0,"t1":t1,
        "duration_s":(t1-t0)/1e9,"horiz_mm":horiz,"err_mm":err,
        "dz_mm":(ff(p1["pz_m"])-ff(p0["pz_m"]))*1000,
        "start_bx":ff(p0["bax"]),"end_bx":ff(p1["bax"]),
        "start_roll":ff(p0["roll_deg"]),"start_pitch":ff(p0["pitch_deg"]),
        "end_speed":ff(p1["speed_m_s"]),
        "camera_gap_max_ms":max(gaps) if gaps else float("nan"),
        "phases":phases,"fusion":ex,
    }

passes=[]
cont=R/"continuous"
for leg in range(1,5):
    q=build_pass(f"CONT_{leg}",cont,leg)
    if q: passes.append(q)
for n in range(1,5):
    d=R/f"fresh_{n}"
    q=build_pass(f"FRESH_{n}",d,1)
    if q: passes.append(q)

labels = ", ".join(p["label"] for p in passes) if passes else "none"
if len(contp := [p for p in passes if p["label"].startswith("CONT")]) != 4:
    raise SystemExit(
        f"INVALID DATASET: нужны все 4 CONTINUOUS. Найдены: {labels}"
    )
fresh = [p for p in passes if p["label"].startswith("FRESH")]
if len(fresh) < 2:
    raise SystemExit(
        f"INVALID DATASET: для сравнительного экрана нужны минимум 2 FRESH. Найдены: {labels}"
    )
dataset_complete = len(fresh) == 4

print("="*150)
print("V40 — V39 CLEAN RESTART / CARRY-OVER CAUSAL LOCALIZATION")
print("="*150)
print(f"RUN: {R}")
print("VIO = Visual-Inertial Odometry, визуально-инерциальная одометрия.")
print("backend = часть Kimera, которая оценивает положение, скорость, ориентацию и внутренние состояния.")
print("bias = внутренняя оценка постоянного смещения акселерометра; bax/bay/baz — её компоненты X/Y/Z.")
print("PIM = Preintegrated IMU Measurement, интегрированное IMU-предсказание движения между соседними состояниями Kimera.")
print("fusion correction = разница между итоговым состоянием backend и чистым IMU-предсказанием на ТОЧНО том же интервале.")
print("visual translation = монокулярная оценка направления движения камеры; её абсолютный масштаб в текущем frontend CSV НЕ наблюдаем.")
print(f"DATASET: {len(passes)} passes = 4 CONTINUOUS + {len(fresh)} FRESH; complete_8pass={dataset_complete}.")
if not dataset_complete:
    print("STATUS: PARTIAL BUT USABLE — дополнительных физических проходов для этого диагностического шага НЕ требуется.")
print("inlier = доля визуальных соответствий, признанных геометрически согласованными.")
print("tracked = число визуальных точек, прослеженных между кадрами.\n")

print("PER-PASS RESULT")
print("-"*150)
print("PASS       DUR_s  DIST_mm ERR_mm  DZ_mm   Bx_START   Bx_END   ROLL0  PITCH0  CAMgap")
for p in passes:
    print(f"{p['label']:10s} {p['duration_s']:6.3f} {p['horiz_mm']:8.2f} {p['err_mm']:+7.2f} {p['dz_mm']:+7.2f}"
          f" {p['start_bx']:+10.5f} {p['end_bx']:+9.5f} {p['start_roll']:+6.2f} {p['start_pitch']:+7.2f} {p['camera_gap_max_ms']:7.1f}")

# contp/fresh validated above. V40 intentionally supports the already-recorded
# partial V39 dataset so no additional physical pass is required merely to run analysis.

def series(name,q):
    errs=[p["err_mm"] for p in q]
    print(f"{name:10s}: n={len(q)} mean={mean([p['horiz_mm'] for p in q]):.2f} mm"
          f" mean_err={mean(errs):+.2f} mm MAE={mae(errs):.2f} mm STD={sd([p['horiz_mm'] for p in q]):.2f} mm"
          f" start|Bx|={mean([abs(p['start_bx']) for p in q]):.5f} m/s²")

print("\nSERIES RESULT")
print("-"*150)
series("CONT",contp); series("FRESH",fresh)

print("\nTEMPORAL LOCALIZATION — SERIES MEANS")
print("-"*150)
metrics=[
    ("dx_mm","ΔX backend, изменение X по оценке Kimera"),
    ("bx_mean","bias X, оценка постоянного смещения акселерометра по X"),
    ("inlier","inlier, доля правильных визуальных соответствий"),
    ("tracked","tracked, число отслеживаемых визуальных точек"),
    ("hacc_rms","IMU horizontal acceleration RMS, среднеквадратичное горизонтальное ускорение"),
    ("gyro_rms","gyro RMS, среднеквадратичная угловая скорость"),
    ("mono_valid","mono valid, доля кадров с допустимой визуальной оценкой движения"),
]
for phase in ("0-0.5s","0.5-1s","1-2s","MID","LAST1s"):
    print(f"\n[{phase}]")
    for key,desc in metrics:
        ca=mean([p["phases"].get(phase,{}).get(key,float("nan")) for p in contp])
        fa=mean([p["phases"].get(phase,{}).get(key,float("nan")) for p in fresh])
        print(f"  {desc:68s} CONT={ca:+10.5f} FRESH={fa:+10.5f} Δ={fa-ca:+10.5f}")

print("\nEXACT-INTERVAL IMU ↔ BACKEND FUSION")
print("-"*150)
for name,q in (("CONT",contp),("FRESH",fresh)):
    ex=[x for p in q for x in p["fusion"]]
    print(f"{name:10s}: exact_intervals={len(ex):3d}"
          f" mean_corrX={mean([x['corrx_mm'] for x in ex]):+.3f} mm/int"
          f" mean|corrP|={mean([x['corrp_mm'] for x in ex]):.3f} mm/int"
          f" mean|corrV|={mean([x['corrv'] for x in ex]):.5f} m/s"
          f" mean_PIM_dP={mean([x['pim_dp_mm'] for x in ex]):.2f} mm"
          f" mean_PIM_dV={mean([x['pim_dv'] for x in ex]):.4f} m/s")

print(f"\nCORRELATION WITH FINAL ERROR — ALL {len(passes)} AVAILABLE PASSES")
print("-"*150)
features=defaultdict(list); errors=[]
for p in passes:
    errors.append(p["err_mm"])
    features["start_bias_x"].append(p["start_bx"])
    features["end_bias_x"].append(p["end_bx"])
    features["duration_s"].append(p["duration_s"])
    features["camera_gap_max_ms"].append(p["camera_gap_max_ms"])
    for ph in ("0-0.5s","0.5-1s","1-2s","MID","LAST1s"):
        st=p["phases"].get(ph,{})
        for k in ("bx_mean","inlier","tracked","hacc_rms","gyro_rms","mono_valid"):
            features[f"{ph}:{k}"].append(st.get(k,float("nan")))
    ex=p["fusion"]
    features["fusion_corrx_mean"].append(mean([x["corrx_mm"] for x in ex]))
    features["fusion_corrp_mean"].append(mean([x["corrp_mm"] for x in ex]))
    features["pim_dp_mean"].append(mean([x["pim_dp_mm"] for x in ex]))
    features["pim_dv_mean"].append(mean([x["pim_dv"] for x in ex]))

rank=sorted(((abs(corr(v,errors)) if math.isfinite(corr(v,errors)) else -1,k,corr(v,errors)) for k,v in features.items()), reverse=True)
for _,k,c in rank[:18]:
    print(f"{k:34s} corr(error)={c:+.3f}")

# First divergence heuristic: compare standardized CONT/FRESH separation by phase.
print("\nFIRST-DIVERGENCE SCREEN")
print("-"*150)
phase_order=("0-0.5s","0.5-1s","1-2s","MID","LAST1s")
cand=[]
for ph in phase_order:
    for k in ("bx_mean","inlier","tracked","hacc_rms","gyro_rms","mono_valid"):
        a=[p["phases"].get(ph,{}).get(k,float("nan")) for p in contp]
        b=[p["phases"].get(ph,{}).get(k,float("nan")) for p in fresh]
        qa=finite(a); qb=finite(b)
        if not qa or not qb: continue
        pooled=math.sqrt((sd(qa)**2+sd(qb)**2)/2)
        sep=abs(mean(qb)-mean(qa))/(pooled+1e-12)
        cand.append((phase_order.index(ph),-sep,ph,k,mean(qa),mean(qb),sep))
for _,_,ph,k,a,b,sep in sorted(cand)[:12]:
    print(f"{ph:8s} {k:12s} CONT={a:+.5f} FRESH={b:+.5f} normalized_separation={sep:.2f}")

print("\nAUTOMATIC DECISION AID")
print("-"*150)
if contp and fresh:
    cont_mae=mae([p["err_mm"] for p in contp]); fresh_mae=mae([p["err_mm"] for p in fresh])
    cont_std=sd([p["horiz_mm"] for p in contp]); fresh_std=sd([p["horiz_mm"] for p in fresh])
    delta_mae=fresh_mae-cont_mae
    delta_std=fresh_std-cont_std
    print(f"ΔMAE FRESH-CONT = {delta_mae:+.2f} mm")
    print(f"ΔSTD FRESH-CONT = {delta_std:+.2f} mm")
    if abs(delta_mae)<3 and abs(delta_std)<3:
        print("CARRY-OVER EFFECT: WEAK — перезапуск VIO почти не меняет точность/повторяемость.")
    elif delta_mae < -3:
        print("CARRY-OVER EFFECT: HARMFUL — FRESH заметно точнее CONT; перенос состояния остаётся причинным кандидатом.")
    elif delta_mae > 3:
        print("FIRST-START EFFECT: SUSPECT — FRESH заметно хуже CONT; искать механизм первых секунд после инициализации.")
    else:
        print("REPEATABILITY EFFECT: MIXED — средняя точность и разброс реагируют по-разному.")

print("\nOBSERVABILITY LIMIT")
print("-"*150)
print("Текущие CSV НЕ содержат независимой метрической camera-only оценки перемещения.")
print("Поэтому этот анализ может локализовать расхождение IMU/PIM ↔ backend/fusion и изменения frontend quality,")
print("но НЕ может доказать абсолютную ошибку масштаба чисто визуального измерения.")
print("Если после этого шага visual-scale останется ведущим кандидатом, следующий физический тест должен быть ОДИН,")
print("с добавленным метрическим camera-only диагностическим каналом в тот же GUI, а не новая серия из 4–12 проходов.")

print("\nHOW TO READ THIS TEST")
print("-"*150)
print("1) Сначала сравнить MAE и STD: это отвечает, влияет ли перезапуск процесса на реальную точность.")
print("2) Затем смотреть FIRST-DIVERGENCE: первое по времени внутреннее различие важнее поздней корреляции.")
print("3) Если PIM одинаков, а fusion correction расходится — подозрение смещается к visual-inertial fusion, то есть объединению камеры и IMU.")
print("4) Если PIM расходится раньше backend — подозрение смещается к IMU preintegration, то есть интегрированию IMU между состояниями.")
print("5) Если bias расходится раньше fusion correction и имеет сопоставимый порядок величины — bias остаётся причинным кандидатом.")
print("6) Если frontend/inlier/tracked расходятся раньше всего — визуальная часть снова становится ведущим кандидатом.")
print("7) Следующий физический шаг допускается только если этот анализ не локализует источник по существующим данным.")
print("8) Любое изменение считается прогрессом только при снижении MAE (средней абсолютной ошибки) или STD (разброса), либо при строгом исключении целого класса причин.")

out=Path("/home/vio/jtzero_v40_v39_causal_localization.csv")
with out.open("w",newline="") as f:
    w=csv.writer(f)
    w.writerow(["pass","duration_s","distance_mm","error_mm","dz_mm","start_bias_x","end_bias_x","start_roll_deg","start_pitch_deg","camera_gap_max_ms"])
    for p in passes:
        w.writerow([p["label"],p["duration_s"],p["horiz_mm"],p["err_mm"],p["dz_mm"],p["start_bx"],p["end_bx"],p["start_roll"],p["start_pitch"],p["camera_gap_max_ms"]])
print(f"\nCSV: {out}")
