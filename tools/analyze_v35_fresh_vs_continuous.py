#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics

ROOT = Path("/home/vio/jtzero_runs")

def f(x):
    try: return float(x)
    except: return float("nan")

def ii(x):
    try: return int(float(x))
    except: return 0

def finite(v):
    return [x for x in v if math.isfinite(x)]

def mean(v):
    v = finite(v)
    return statistics.mean(v) if v else float("nan")

def sd(v):
    v = finite(v)
    return statistics.pstdev(v) if len(v) > 1 else 0.0

def norm3(v):
    return math.sqrt(sum(x*x for x in v))

def matvec(M, v):
    return tuple(sum(M[r][c]*v[c] for c in range(3)) for r in range(3))

def matmul(A, B):
    return tuple(tuple(sum(A[r][k]*B[k][c] for k in range(3)) for c in range(3)) for r in range(3))

def rpy_matrix(rdeg, pdeg, ydeg):
    r,p,y = map(math.radians, (rdeg,pdeg,ydeg))
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr))
    Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    Rz=((cy,-sy,0),(sy,cy,0),(0,0,1))
    return matmul(Rz, matmul(Ry, Rx))

def read_csv(path):
    with path.open() as fh:
        return list(csv.DictReader(fh))

def latest(pattern):
    q = sorted(ROOT.glob(pattern))
    return q[-1] if q else None

def world_bias_row(r):
    Rw = rpy_matrix(f(r.get("roll_deg")), f(r.get("pitch_deg")), f(r.get("yaw_deg")))
    bb = (f(r.get("bax")), f(r.get("bay")), f(r.get("baz")))
    bw = matvec(Rw, bb)
    return {
        "t": ii(r.get("timestamp_ns")),
        "kf": ii(r.get("keyframe")),
        "bwx": bw[0], "bwy": bw[1], "bwz": bw[2],
        "speed": norm3((f(r.get("vx_m_s")), f(r.get("vy_m_s")), f(r.get("vz_m_s")))),
    }

def find_start_end(events, leg=1):
    es = [r for r in events if ii(r.get("leg")) == leg]
    start = next((r for r in es if r.get("event") == "START"), None)
    end   = next((r for r in es if r.get("event") == "END"), None)
    return start, end

def nearest_before(rows, t):
    q = [r for r in rows if r["t"] <= t]
    return q[-1] if q else None

def extract_pass(run, leg=1, name=""):
    legs = read_csv(run/"jtzero_500mm_v25_legs.csv")
    backend_raw = read_csv(run/"jtzero_500mm_v25_backend.csv")
    events = read_csv(run/"jtzero_500mm_v25_events.csv")
    backend = [world_bias_row(r) for r in backend_raw]

    lr = next((r for r in legs if ii(r.get("leg")) == leg), None)
    if lr is None:
        raise RuntimeError(f"{run}: leg {leg} not found")

    start_ev, end_ev = find_start_end(events, leg)
    if start_ev is None or end_ev is None:
        raise RuntimeError(f"{run}: START/END events for leg {leg} not found")

    ts = ii(start_ev.get("state_timestamp_ns"))
    te = ii(end_ev.get("state_timestamp_ns"))
    bs = nearest_before(backend, ts)
    be = nearest_before(backend, te)
    if bs is None or be is None:
        raise RuntimeError(f"{run}: cannot align backend bias to leg {leg}")

    # Prefer explicit horizontal field names, otherwise reconstruct XY from leg displacement.
    horiz = f(lr.get("horizontal_m"))
    if math.isfinite(horiz):
        horiz_mm = horiz * 1000.0
    else:
        horiz_mm = f(lr.get("horizontal_mm"))
    if not math.isfinite(horiz_mm):
        dx = f(lr.get("dx_m")); dy = f(lr.get("dy_m"))
        if math.isfinite(dx) and math.isfinite(dy):
            horiz_mm = math.hypot(dx, dy) * 1000.0
    if not math.isfinite(horiz_mm):
        # archived V25 legs commonly stores distance_m
        dm = f(lr.get("distance_m"))
        if math.isfinite(dm):
            horiz_mm = dm * 1000.0

    dz_mm = f(lr.get("dz_m"))
    if math.isfinite(dz_mm):
        dz_mm *= 1000.0
    else:
        dz_mm = f(lr.get("dz_mm"))

    dur = f(lr.get("motion_duration_s"))
    if not math.isfinite(dur):
        dur = (te-ts)/1e9

    q = [r for r in backend if ts <= r["t"] <= te]
    return {
        "name": name,
        "run": str(run),
        "leg": leg,
        "duration_s": dur,
        "horizontal_mm": horiz_mm,
        "error_mm": horiz_mm - 500.0,
        "dz_mm": dz_mm,
        "start_bx": bs["bwx"],
        "end_bx": be["bwx"],
        "delta_bx": be["bwx"] - bs["bwx"],
        "start_bnorm": norm3((bs["bwx"],bs["bwy"],bs["bwz"])),
        "end_bnorm": norm3((be["bwx"],be["bwy"],be["bwz"])),
        "start_speed_mm_s": bs["speed"]*1000.0,
        "end_speed_mm_s": be["speed"]*1000.0,
        "bx_span": (max(r["bwx"] for r in q)-min(r["bwx"] for r in q)) if q else float("nan"),
    }

v34 = latest("*_v34_FRESHSTART_4X")
if v34 is None:
    raise SystemExit("Не найден архив *_v34_FRESHSTART_4X")

fresh = []
for n in range(1,5):
    d = v34/f"fresh_{n}"
    if not d.exists():
        raise SystemExit(f"Не найден {d}")
    fresh.append(extract_pass(d, 1, f"FRESH_{n}"))

# Continuous reference: V27 ROLD 7.5 s, four measured legs in one VIO process.
cont_run = latest("*_v25_TBS_ROLD_7P5")
if cont_run is None:
    raise SystemExit("Не найден контроль *_v25_TBS_ROLD_7P5")

continuous = [extract_pass(cont_run, leg, f"CONT_{leg}") for leg in range(1,5)]

print("="*170)
print("V35 — FRESH-START vs CONTINUOUS BIAS CARRY-OVER")
print("="*170)
print(f"FRESH archive:      {v34}")
print(f"CONTINUOUS control: {cont_run}")
print("Сравнение проверяет, меняется ли ошибка 500 мм, когда каждый A→B начинается новым процессом VIO.")
print("Все bias показаны в одной мировой системе координат; знак B→A здесь не канонизируется.\n")

print("PER-PASS TABLE")
print("-"*170)
print("SERIES      PASS  DUR_s   HORIZ_mm  ERR_mm   DZ_mm   Bx_START   Bx_END    dBx       |B|start  Bx_span   Vend_mm/s")
for group, rows in (("FRESH",fresh),("CONT",continuous)):
    for i,r in enumerate(rows,1):
        print(f"{group:10s} {i:4d} {r['duration_s']:7.3f} {r['horizontal_mm']:10.2f} {r['error_mm']:+8.2f}"
              f" {r['dz_mm']:+7.2f} {r['start_bx']:+10.5f} {r['end_bx']:+10.5f} {r['delta_bx']:+9.5f}"
              f" {r['start_bnorm']:9.5f} {r['bx_span']:9.5f} {r['end_speed_mm_s']:11.2f}")

def summary(label, rows):
    print(f"{label:12s}: mean={mean([r['horizontal_mm'] for r in rows]):7.2f} mm"
          f"  mean_err={mean([r['error_mm'] for r in rows]):+7.2f} mm"
          f"  MAE={mean([abs(r['error_mm']) for r in rows]):6.2f} mm"
          f"  STD={sd([r['horizontal_mm'] for r in rows]):6.2f} mm"
          f"  start|Bx|={mean([abs(r['start_bx']) for r in rows]):.5f} m/s²"
          f"  end|Bx|={mean([abs(r['end_bx']) for r in rows]):.5f} m/s²"
          f"  mean dBx={mean([r['delta_bx'] for r in rows]):+.5f} m/s²")

print("\nSERIES SUMMARY")
print("-"*170)
summary("FRESH-START", fresh)
summary("CONTINUOUS", continuous)

fresh_mae = mean([abs(r["error_mm"]) for r in fresh])
cont_mae = mean([abs(r["error_mm"]) for r in continuous])
fresh_std = sd([r["horizontal_mm"] for r in fresh])
cont_std = sd([r["horizontal_mm"] for r in continuous])
fresh_start = mean([abs(r["start_bx"]) for r in fresh])
cont_start = mean([abs(r["start_bx"]) for r in continuous])

print("\nDIRECT EFFECT")
print("-"*170)
print(f"MAE:             {cont_mae:.2f} -> {fresh_mae:.2f} mm   change={fresh_mae-cont_mae:+.2f} mm")
print(f"Repeatability σ: {cont_std:.2f} -> {fresh_std:.2f} mm   change={fresh_std-cont_std:+.2f} mm")
print(f"mean |start Bx|: {cont_start:.5f} -> {fresh_start:.5f} m/s²")

# First-pass-to-first-pass is the cleanest comparison because neither has prior measured motion in the same process.
c1 = continuous[0]
fmean = mean([r["error_mm"] for r in fresh])
print("\nFIRST-PASS CONTROL")
print("-"*170)
print(f"Continuous PASS1 error: {c1['error_mm']:+.2f} mm, start Bx={c1['start_bx']:+.5f} m/s²")
print(f"Fresh mean error:       {fmean:+.2f} mm, mean |start Bx|={fresh_start:.5f} m/s²")
print("Это критический контроль: если fresh-start помогает только относительно CONT passes 2-4, но не относительно CONT pass 1,")
print("то reset устраняет carry-over, но не исходный механизм ошибки первого движения.")

print("\nAUTOMATIC VERDICT")
print("-"*170)
bias_reset = fresh_start < cont_start * 0.50 if cont_start > 1e-9 else fresh_start < 0.01
mae_gain = fresh_mae < cont_mae * 0.75
std_gain = fresh_std < cont_std * 0.75
first_like = abs(fmean - c1["error_mm"]) <= 5.0

if bias_reset:
    print("1. START-BIAS RESET: YES — fresh-start существенно уменьшает начальный |bias X|.")
else:
    print("1. START-BIAS RESET: NO/WEAK — начальный |bias X| не уменьшился достаточно.")

if mae_gain:
    print("2. ACCURACY: STRONG IMPROVEMENT — MAE уменьшилась >=25%.")
elif fresh_mae < cont_mae:
    print("2. ACCURACY: SMALL/MODERATE IMPROVEMENT — MAE уменьшилась, но менее чем на 25%.")
else:
    print("2. ACCURACY: NO IMPROVEMENT — fresh-start не уменьшил MAE.")

if std_gain:
    print("3. REPEATABILITY: STRONG IMPROVEMENT — разброс уменьшился >=25%.")
elif fresh_std < cont_std:
    print("3. REPEATABILITY: SMALL/MODERATE IMPROVEMENT.")
else:
    print("3. REPEATABILITY: NO IMPROVEMENT.")

if bias_reset and (mae_gain or std_gain):
    print("4. CARRY-OVER VERDICT: SUPPORTED — перенос bias является реальным источником части ошибки/нестабильности.")
elif bias_reset:
    print("4. CARRY-OVER VERDICT: STATE EXISTS, BUT DISTANCE IMPACT NOT PROVEN — bias сбрасывается, но метрика расстояния существенно не улучшается.")
else:
    print("4. CARRY-OVER VERDICT: NOT ISOLATED by this experiment.")

if first_like:
    print("5. FIRST-MOTION MECHANISM: LIKELY REMAINS — fresh-start похож на первый проход continuous; следующий поиск должен смотреть, почему bias формируется во время самого A→B.")
else:
    print("5. FIRST-MOTION MECHANISM: CHANGED — fresh-start отличается даже от первого continuous pass; проверить startup/init conditions и профиль движения.")

out = Path.home()/"jtzero_v35_fresh_vs_continuous.csv"
fields = ["name","run","leg","duration_s","horizontal_mm","error_mm","dz_mm","start_bx","end_bx","delta_bx",
          "start_bnorm","end_bnorm","start_speed_mm_s","end_speed_mm_s","bx_span"]
with out.open("w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=fields)
    w.writeheader()
    for r in fresh + continuous:
        w.writerow({k:r.get(k) for k in fields})
print(f"\nCSV: {out}")
