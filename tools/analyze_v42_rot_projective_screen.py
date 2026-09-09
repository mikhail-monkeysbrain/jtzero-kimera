#!/usr/bin/env python3
from pathlib import Path
import argparse, csv, math, statistics

def f(x):
    try:return float(x)
    except:return float("nan")
def ii(x):
    try:return int(float(x))
    except:return 0
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def corr(a,b):
    p=[(x,y) for x,y in zip(a,b) if math.isfinite(x) and math.isfinite(y)]
    if len(p)<5:return float("nan")
    ax=[x for x,y in p]; by=[y for x,y in p]
    ma=mean(ax); mb=mean(by)
    sa=sum((x-ma)**2 for x in ax); sb=sum((y-mb)**2 for y in by)
    if sa<=0 or sb<=0:return float("nan")
    return sum((x-ma)*(y-mb) for x,y in p)/math.sqrt(sa*sb)

p=argparse.ArgumentParser(description="V42 rotational/projective contamination screen")
p.add_argument("--run",default="",help="V42 archive; default latest")
args=p.parse_args()
root=Path("/home/vio/jtzero_runs")
R=Path(args.run) if args.run else sorted(root.glob("*_v42_DIRECT_LUNA_CAMERA_VS_FUSION"))[-1]

front=list(csv.DictReader((R/"jtzero_500mm_v25_frontend.csv").open()))
events=list(csv.DictReader((R/"jtzero_500mm_v25_events.csv").open()))
if not front or not events: raise SystemExit("Нет frontend/events данных")

start=end=None
for e in events:
    if e.get("event")=="START": start=ii(e.get("state_timestamp_ns"))
    elif e.get("event")=="END": end=ii(e.get("state_timestamp_ns"))
if start is None or end is None: raise SystemExit("Не найдены START/END")

rows=[]
for r in front:
    t=ii(r.get("timestamp_ns"))
    if not (start<=t<=end): continue
    if ii(r.get("mono_pose_valid",0))!=1: continue
    tx,ty,tz=[f(r.get(k)) for k in ("mono_body_tx","mono_body_ty","mono_body_tz")]
    n=math.sqrt(tx*tx+ty*ty+tz*tz)
    if not math.isfinite(n) or n<=1e-12: continue
    # Translation-direction tilt relative to body XY plane.
    tilt=math.degrees(math.atan2(abs(tz), math.hypot(tx,ty)))
    rot=math.sqrt(sum(f(r.get(k,0))**2 for k in ("mono_roll_deg","mono_pitch_deg","mono_yaw_deg")))
    inlier=f(r.get("mono_inlier_ratio"))
    rows.append((t,tilt,rot,inlier,tx,ty,tz))

print("="*108)
print("V42 — ROTATIONAL / PROJECTIVE CONTAMINATION SCREEN")
print("="*108)
print(f"RUN: {R}")
print(f"valid mono intervals: {len(rows)}")
if not rows: raise SystemExit(0)

tilt=[x[1] for x in rows]; rot=[x[2] for x in rows]; inl=[x[3] for x in rows]
print(f"mono translation out-of-plane tilt: mean={mean(tilt):.2f} deg  p90={statistics.quantiles(tilt,n=10)[8]:.2f} deg  max={max(tilt):.2f} deg")
print(f"mono relative rotation magnitude:    mean={mean(rot):.2f} deg  p90={statistics.quantiles(rot,n=10)[8]:.2f} deg  max={max(rot):.2f} deg")
print(f"mono inlier ratio:                  mean={mean(inl):.3f}")
print(f"corr(rotation magnitude, translation tilt) = {corr(rot,tilt):+.3f}")

# Split by rotation magnitude quartiles to see whether out-of-plane translation grows with rotation.
sr=sorted(rows,key=lambda x:x[2])
q=max(1,len(sr)//4)
lo=sr[:q]; hi=sr[-q:]
print("\nLOW-vs-HIGH ROTATION")
print("-"*108)
print(f"LOW  rotation: n={len(lo)} mean rot={mean([x[2] for x in lo]):.2f} deg  mean tilt={mean([x[1] for x in lo]):.2f} deg")
print(f"HIGH rotation: n={len(hi)} mean rot={mean([x[2] for x in hi]):.2f} deg  mean tilt={mean([x[1] for x in hi]):.2f} deg")
print(f"tilt increase HIGH-LOW = {mean([x[1] for x in hi])-mean([x[1] for x in lo]):+.2f} deg")

print("\nINTERPRETATION")
print("-"*108)
print("1) mono_body translation имеет произвольный масштаб, но направление можно использовать.")
print("2) Если out-of-plane tilt растёт вместе с mono rotation, простая affine camera-only модель")
print("   вероятно смешивает перспективу/вращение с поступательным движением.")
print("3) Это не доказывает источник ошибки Kimera; тест относится только к независимой camera-only оценке V42.")
print("4) Если связи почти нет, следующий кандидат camera-only — affine scale/model mismatch даже при малом вращении.")
print("="*108)
