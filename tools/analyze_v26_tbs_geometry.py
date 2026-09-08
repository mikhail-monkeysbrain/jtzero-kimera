#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics

ROOT = Path("/home/vio/jtzero-kimera-sync")
SRC = Path("/home/vio/jtzero_v26_12pass_directional.csv")
PARAM = ROOT / "params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003/LeftCameraParams.yaml"
OUT = Path("/home/vio/jtzero_v26_tbs_geometry.csv")

TRUE_MM = 500.0
DECLARED_RX_DEG = -1.5
DECLARED_RY_DEG = -5.5

def f(x):
    try: return float(x)
    except: return float("nan")

def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")

def sd(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.pstdev(v) if len(v)>1 else 0.0

def rot_y(v, deg):
    x,y,z=v
    a=math.radians(deg); c=math.cos(a); s=math.sin(a)
    return (c*x+s*z, y, -s*x+c*z)

if not SRC.exists():
    raise SystemExit("Сначала запустите tools/analyze_v26_12pass_rootcause.py и tools/analyze_v26_12pass_directional.py")
if not PARAM.exists():
    raise SystemExit(f"Не найден файл параметров: {PARAM}")

with SRC.open() as fh:
    rows=list(csv.DictReader(fh))

# Canonicalize all physical passes into the same A->B direction.
# Odd legs are A->B, even legs are B->A, so multiply even vectors by -1.
for r in rows:
    leg=int(r["leg"])
    sign=1.0 if leg%2 else -1.0
    dx=sign*f(r["dx_mm"]); dy=sign*f(r["dy_mm"]); dz=sign*f(r["dz_mm"])
    h=math.hypot(dx,dy)
    d3=math.sqrt(dx*dx+dy*dy+dz*dz)
    elev=math.degrees(math.atan2(dz,h))
    az=math.degrees(math.atan2(dy,dx))
    r["canon_dx_mm"]=dx; r["canon_dy_mm"]=dy; r["canon_dz_mm"]=dz
    r["elevation_deg"]=elev; r["azimuth_deg"]=az; r["norm3d_mm"]=d3
    r["norm3d_error_mm"]=d3-TRUE_MM

# Best common pitch-like tilt from the canonical displacement vectors.
mx=mean([r["canon_dx_mm"] for r in rows])
my=mean([r["canon_dy_mm"] for r in rows])
mz=mean([r["canon_dz_mm"] for r in rows])
fit_elev=math.degrees(math.atan2(mz, math.hypot(mx,my)))

# Remove the fitted elevation by rotating around body/world Y.
# Positive fit_elev removes positive canonical Z with this rotation convention.
for r in rows:
    v=(r["canon_dx_mm"],r["canon_dy_mm"],r["canon_dz_mm"])
    xc,yc,zc=rot_y(v, fit_elev)
    hcorr=math.hypot(xc,yc)
    r["detilt_dx_mm"]=xc; r["detilt_dy_mm"]=yc; r["detilt_dz_mm"]=zc
    r["detilt_horizontal_mm"]=hcorr
    r["detilt_error_mm"]=hcorr-TRUE_MM

# Also test exactly the magnitude of the declared -5.5 deg correction.
# We test both signs because the YAML comment alone does not prove which
# multiplication convention ultimately maps into the reported world vector.
for r in rows:
    v=(r["canon_dx_mm"],r["canon_dy_mm"],r["canon_dz_mm"])
    for label,deg in (("ry_plus_5p5",+5.5),("ry_minus_5p5",-5.5)):
        x,y,z=rot_y(v,deg)
        r[label+"_z_mm"]=z
        r[label+"_horizontal_mm"]=math.hypot(x,y)
        r[label+"_error_mm"]=math.hypot(x,y)-TRUE_MM

print("\n"+"="*132)
print("V26 T_BS / GEOMETRY FORENSIC — 12 CONTROLLED PASSES")
print("="*132)
print("Все векторы приведены к одному физическому направлению A→B.")
print("Угол наклона = atan2(ΔZ, горизонтальное перемещение).")
print()
print("MODE          LEG DIR   HORIZ_mm   ERR_mm    DZcanon   ANGLE_deg   3D_mm   3D_ERR   DETILT_H   DETILT_ERR")
for r in rows:
    d="A>B" if int(r["leg"])%2 else "B>A"
    print(f"{r['mode']:<13} {int(r['leg']):>3d} {d:<3}"
          f" {f(r['horizontal_mm']):10.2f} {f(r['error_mm']):+8.2f}"
          f" {r['canon_dz_mm']:+10.2f} {r['elevation_deg']:+11.3f}"
          f" {r['norm3d_mm']:8.2f} {r['norm3d_error_mm']:+8.2f}"
          f" {r['detilt_horizontal_mm']:10.2f} {r['detilt_error_mm']:+11.2f}")

angles=[r["elevation_deg"] for r in rows]
print("\n"+"="*132)
print("COMMON TILT")
print("="*132)
print(f"Средний угол отдельных проходов: {mean(angles):+.3f} deg")
print(f"Стандартное отклонение угла:     {sd(angles):.3f} deg")
print(f"Угол среднего 3D-вектора:        {fit_elev:+.3f} deg")
print(f"Заявленная коррекция T_BS Ry:    {DECLARED_RY_DEG:+.3f} deg")
print(f"|разница по модулю|:             {abs(abs(fit_elev)-abs(DECLARED_RY_DEG)):.3f} deg")
print(f"Средний canonical ΔY:            {my:+.3f} mm")
print(f"Средний canonical ΔZ:            {mz:+.3f} mm")

orig_err=[f(r["error_mm"]) for r in rows]
det_err=[r["detilt_error_mm"] for r in rows]
err3=[r["norm3d_error_mm"] for r in rows]
print("\n"+"="*132)
print("HOW MUCH OF THE 500 mm ERROR IS EXPLAINED BY THE COMMON TILT?")
print("="*132)
print(f"Исходная средняя ошибка:          {mean(orig_err):+.3f} mm   std={sd(orig_err):.3f} mm")
print(f"После удаления общего наклона:   {mean(det_err):+.3f} mm   std={sd(det_err):.3f} mm")
print(f"Ошибка полной длины 3D-вектора:  {mean(err3):+.3f} mm   std={sd(err3):.3f} mm")
print(f"Средняя поправка только наклоном: {mean([d-o for d,o in zip(det_err,orig_err)]):+.3f} mm")

print("\n"+"="*132)
print("EXACT ±5.5 DEG SIGN TEST")
print("="*132)
for label in ("ry_plus_5p5","ry_minus_5p5"):
    zs=[r[label+"_z_mm"] for r in rows]
    es=[r[label+"_error_mm"] for r in rows]
    print(f"{label:<16}: mean residual Z={mean(zs):+8.3f} mm  std Z={sd(zs):6.3f} mm"
          f"   mean horizontal error={mean(es):+8.3f} mm")

print("\n"+"="*132)
print("DIRECTION / SPEED AFTER DETILT")
print("="*132)
for mode in ("FAST_5S","MEDIUM_7P5S","SLOW_10S"):
    for odd,label in ((True,"A_TO_B"),(False,"B_TO_A")):
        rr=[r for r in rows if r["mode"]==mode and ((int(r["leg"])%2)==1)==odd]
        print(f"{mode:<13} {label:<6}"
              f" original_err={mean([f(r['error_mm']) for r in rr]):+7.2f} mm"
              f" detilt_err={mean([r['detilt_error_mm'] for r in rr]):+7.2f} mm"
              f" residual_Z={mean([r['detilt_dz_mm'] for r in rr]):+7.2f} mm")

# Interpretation guardrails, based only on computed geometry.
print("\n"+"="*132)
print("AUTOMATIC GEOMETRY VERDICT")
print("="*132)
angle_match = abs(abs(fit_elev)-abs(DECLARED_RY_DEG)) < 1.0
tilt_correction = abs(mean([d-o for d,o in zip(det_err,orig_err)]))
residual_scale = sd(det_err)
if angle_match:
    print("1. COMMON-TILT MATCH: YES — измеренный наклон близок по модулю к 5.5 deg из T_BS.")
else:
    print("1. COMMON-TILT MATCH: NO — измеренный наклон не совпадает с 5.5 deg из T_BS.")
if tilt_correction < 5.0:
    print("2. DISTANCE IMPACT: SMALL — общий наклон объясняет только несколько миллиметров горизонтальной ошибки.")
else:
    print("2. DISTANCE IMPACT: MATERIAL — общий наклон заметно меняет горизонтальную оценку.")
if residual_scale > 5.0:
    print("3. RESIDUAL ERROR: REMAINS — после устранения наклона остаётся существенный разброс/масштабная ошибка.")
else:
    print("3. RESIDUAL ERROR: LOW — после устранения наклона основная ошибка почти исчезает.")
print("4. ВАЖНО: совпадение угла с T_BS ещё не доказывает, что сама матрица применена неверно;")
print("   оно доказывает, что следующий A/B-тест должен менять только ориентацию T_BS и ничего больше.")

keys=list(rows[0].keys())
with OUT.open("w",newline="") as fh:
    w=csv.DictWriter(fh,fieldnames=keys)
    w.writeheader(); w.writerows(rows)
print(f"\nCSV: {OUT}")
