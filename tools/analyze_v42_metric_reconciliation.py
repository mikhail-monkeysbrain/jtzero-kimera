#!/usr/bin/env python3
from pathlib import Path
import argparse, csv, math, statistics

def f(x):
    try: return float(x)
    except: return float("nan")

p=argparse.ArgumentParser(description="V42 metric reconciliation: physical truth vs camera-only vs Kimera")
p.add_argument("--run", default="", help="V42 archive path; default = latest *_v42_DIRECT_LUNA_CAMERA_VS_FUSION")
p.add_argument("--camera-net-mm", type=float, required=True, help="CAMERA-ONLY net printed by V42")
p.add_argument("--luna-mm", type=float, required=True, help="direct static TF-Luna distance for this geometry")
args=p.parse_args()

root=Path("/home/vio/jtzero_runs")
if args.run:
    R=Path(args.run)
else:
    cand=sorted(root.glob("*_v42_DIRECT_LUNA_CAMERA_VS_FUSION"))
    if not cand: raise SystemExit("Не найден архив V42")
    R=cand[-1]

meta={}
mp=R/"V42_METADATA.txt"
if mp.exists():
    for line in mp.read_text().splitlines():
        if "=" in line:
            k,v=line.split("=",1); meta[k.strip()]=v.strip()

legs=list(csv.DictReader((R/"jtzero_500mm_v25_legs.csv").open()))
if not legs: raise SystemExit("Нет строки leg в jtzero_500mm_v25_legs.csv")
leg=legs[0]
truth_mm=500.0
kimera_mm=f(leg.get("horizontal_m", leg.get("horizontal", "nan")))*1000.0
if not math.isfinite(kimera_mm):
    # tolerate archived schema by finding the first field containing horizontal
    for k,v in leg.items():
        if "horizontal" in k.lower():
            z=f(v)
            if math.isfinite(z):
                kimera_mm=z*(1000.0 if abs(z)<10 else 1.0)
                break

offset_m=f(meta.get("camera_height_offset_m","nan"))
luna_m=args.luna_mm/1000.0
used_h_m=luna_m+offset_m if math.isfinite(offset_m) else float("nan")
cam_mm=args.camera_net_mm

print("="*104)
print("V42 — METRIC RECONCILIATION")
print("="*104)
print(f"RUN: {R}")
print(f"Физическая истина:              {truth_mm:8.2f} мм")
print(f"Kimera horizontal:              {kimera_mm:8.2f} мм  error={kimera_mm-truth_mm:+.2f} мм")
print(f"CAMERA-ONLY net:                {cam_mm:8.2f} мм  error={cam_mm-truth_mm:+.2f} мм")
print(f"TF-Luna direct:                 {args.luna_mm:8.2f} мм")
print(f"camera height offset metadata:  {offset_m*1000:+8.2f} мм" if math.isfinite(offset_m) else "camera height offset metadata:  N/A")
print(f"camera height used nominally:   {used_h_m*1000:8.2f} мм" if math.isfinite(used_h_m) else "camera height used nominally:   N/A")

if math.isfinite(used_h_m) and used_h_m>0:
    # Frame-to-frame metric translation is linear in h in V41/V42 camera-only estimator.
    cam_at_luna=cam_mm*(luna_m/used_h_m)
    required_h=used_h_m*(truth_mm/cam_mm)
    offset_effect=cam_mm-cam_at_luna
    residual_at_luna=cam_at_luna-truth_mm
    print("\nHEIGHT-SCALE RECONCILIATION")
    print("-"*104)
    print(f"Если убрать camera offset и использовать h={luna_m*1000:.1f} мм:")
    print(f"  ожидаемый CAMERA-ONLY ≈       {cam_at_luna:8.2f} мм")
    print(f"  остаточная ошибка ≈           {residual_at_luna:+8.2f} мм")
    print(f"Эффект текущего offset по масштабу ≈ {offset_effect:+.2f} мм")
    print(f"Высота, необходимая для ровно 500 мм при той же image-motion оценке: {required_h*1000:.2f} мм")
    print(f"Это требовало бы offset относительно TF-Luna: {(required_h-luna_m)*1000:+.2f} мм")

    explained=abs(offset_effect)/max(1e-9,abs(cam_mm-truth_mm))*100.0
    print(f"Доля CAMERA-ONLY ошибки, объясняемая только +offset: {explained:.1f}%")

print("\nINTERPRETATION")
print("-"*104)
print("1) V42 camera-only переводит image motion в метры линейно через высоту h.")
print("2) Поэтому можно строго оценить, сколько ошибки даёт только выбранный camera-height offset.")
print("3) Если после подстановки прямой TF-Luna остаются десятки миллиметров ошибки, одной высотой это не объясняется.")
print("4) Это НЕ доказывает, что ошибка находится в камере: frame-to-frame affine модель также чувствительна к вращению,")
print("   перспективе и неплоскостности сцены. Следующий тест должен проверить именно rotational/projective contamination.")
print("="*104)
