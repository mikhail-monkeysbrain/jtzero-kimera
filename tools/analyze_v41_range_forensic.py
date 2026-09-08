#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics, sys

DEFAULT = Path("/home/vio/jtzero_runs/20260908_222022_v41_CAMERA_VS_FUSION_SINGLE")
run = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
p = run / "jtzero_500mm_v25_range.csv"
if not p.exists():
    raise SystemExit(f"Нет файла: {p}")

with p.open() as f:
    rows = list(csv.DictReader(f))

def num(r,k):
    try: return float(r[k])
    except: return float("nan")

# tolerate current historical header variants
keys = rows[0].keys() if rows else []
def pick(*names):
    for n in names:
        if n in keys: return n
    return None

kc = pick("current_distance","current_distance_cm")
kv = pick("vertical_m","vertical_range_m")
kr = pick("roll_deg","fc_roll_deg")
kp = pick("pitch_deg","fc_pitch_deg")
if not kc:
    raise SystemExit(f"Не найден current_distance. Колонки: {list(keys)}")

curr = [num(r,kc) for r in rows if math.isfinite(num(r,kc))]
vert = [num(r,kv) for r in rows if kv and math.isfinite(num(r,kv))]
roll = [num(r,kr) for r in rows if kr and math.isfinite(num(r,kr))]
pitch = [num(r,kp) for r in rows if kp and math.isfinite(num(r,kp))]

print("="*92)
print("V41 TF-LUNA RANGE FORENSIC")
print("="*92)
print(f"RUN: {run}")
print(f"rows={len(rows)}")
if curr:
    print(f"current_distance (MAVLink, cm): mean={statistics.mean(curr):.3f} median={statistics.median(curr):.3f} min={min(curr):.3f} max={max(curr):.3f}")
    print(f"current_distance raw height: mean={statistics.mean(curr)*10:.1f} mm")
if vert:
    print(f"vertical_m (после cos(roll)*cos(pitch)): mean={statistics.mean(vert)*1000:.1f} mm median={statistics.median(vert)*1000:.1f} mm min={min(vert)*1000:.1f} max={max(vert)*1000:.1f}")
if roll:
    print(f"FC roll:  mean={statistics.mean(roll):+.3f} deg min={min(roll):+.3f} max={max(roll):+.3f}")
if pitch:
    print(f"FC pitch: mean={statistics.mean(pitch):+.3f} deg min={min(pitch):+.3f} max={max(pitch):+.3f}")
if curr and vert:
    rawm=statistics.mean(curr)*0.01
    vm=statistics.mean(vert)
    print(f"attitude correction effect: {(vm-rawm)*1000:+.2f} mm")

physical_camera_mm = 185.0
physical_luna_low_mm = 185.0
physical_luna_high_mm = 190.0
if curr:
    luna_mm=statistics.mean(curr)*10
    print()
    print("PHYSICAL CROSS-CHECK")
    print(f"measured OV9281 sensor plane: ~{physical_camera_mm:.1f} mm")
    print(f"measured TF-Luna center:      ~{physical_luna_low_mm:.1f}..{physical_luna_high_mm:.1f} mm")
    print(f"TF-Luna MAVLink mean:         {luna_mm:.1f} mm")
    print(f"MAVLink vs physical Luna:     {luna_mm-physical_luna_low_mm:+.1f}..{luna_mm-physical_luna_high_mm:+.1f} mm")

print()
print("INTERPRETATION")
print("1) current_distance — значение DISTANCE_SENSOR, уже пришедшее от FC; V41 его не калибрует до записи.")
print("2) vertical_m отличается только на cos(roll)*cos(pitch); при малых углах это доли миллиметра.")
print("3) Если current_distance уже ~160 мм при физической высоте ~185-190 мм, источник расхождения находится ДО V41:")
print("   TF-Luna / настройки rangefinder FC / геометрия измерения, а не camera_height_offset V41.")
