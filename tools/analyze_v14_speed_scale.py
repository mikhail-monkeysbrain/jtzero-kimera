#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v13_legs.csv"
BACKEND=H+"/jtzero_500mm_v13_backend.csv"
OUT=H+"/jtzero_v14_speed_scale.csv"
TRUTH_MM=500.0

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS); be=read(BACKEND)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}
rows=[]
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    dt=(t1-t0)*1e-9
    xy=fv(L,"horizontal_m")*1000.0
    true_v=TRUTH_MM/dt if dt>0 else float("nan")
    vio_v=xy/dt if dt>0 else float("nan")
    rows.append({
      "leg":leg,
      "direction":L["direction"],
      "xy_mm":xy,
      "scale":xy/TRUTH_MM,
      "error_mm":xy-TRUTH_MM,
      "move_duration_s":dt,
      "true_mean_speed_mm_s":true_v,
      "vio_mean_speed_mm_s":vio_v,
    })

def corr(xs,ys):
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx=math.sqrt(sum((x-mx)**2 for x in xs)); dy=math.sqrt(sum((y-my)**2 for y in ys))
    return num/(dx*dy) if dx and dy else float("nan")

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V14 TRUE SPEED vs SCALE ================")
for r in rows:
    mark="  <-- LEG2" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: duration={r["move_duration_s"]:.3f}s '
          f'trueV={r["true_mean_speed_mm_s"]:.1f}mm/s '
          f'VIOV={r["vio_mean_speed_mm_s"]:.1f}mm/s '
          f'XY={r["xy_mm"]:.2f}mm scale={r["scale"]:.4f} err={r["error_mm"]:+.2f}mm{mark}')

xs=[r["true_mean_speed_mm_s"] for r in rows]
ys=[r["scale"] for r in rows]
print(f'all-legs Pearson(TRUE speed, scale) = {corr(xs,ys):.4f}')

for direction in ("A->B","B->A"):
    rr=[x for x in rows if x["direction"]==direction]
    print(f'{direction} Pearson(TRUE speed, scale) = '
          f'{corr([x["true_mean_speed_mm_s"] for x in rr],[x["scale"] for x in rr]):.4f}')

normal=[r for r in rows if r["leg"]!=2]
l2=next(r for r in rows if r["leg"]==2)
normal_v=statistics.mean(r["true_mean_speed_mm_s"] for r in normal)
print(f'normal TRUE mean speed = {normal_v:.1f} mm/s')
print(f'LEG2 TRUE mean speed   = {l2["true_mean_speed_mm_s"]:.1f} mm/s')
print(f'LEG2 TRUE speed excess = {l2["true_mean_speed_mm_s"]/normal_v-1:+.1%}')
print("NOTE: VIO speed is printed only for reference; correlation uses TRUE 500mm/duration speed.")
print("Saved:",OUT)
