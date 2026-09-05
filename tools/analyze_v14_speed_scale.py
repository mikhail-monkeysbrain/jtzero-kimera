#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v13_legs.csv"
BACKEND=H+"/jtzero_500mm_v13_backend.csv"
OUT=H+"/jtzero_v14_speed_scale.csv"

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
    xy=fv(L,"horizontal_m")
    d3=fv(L,"distance3d_m")
    rows.append({
      "leg":leg,
      "direction":L["direction"],
      "xy_mm":xy*1000,
      "scale":xy/0.5,
      "error_mm":(xy-0.5)*1000,
      "move_duration_s":dt,
      "mean_xy_speed_mm_s":xy/dt*1000 if dt>0 else float("nan"),
      "mean_3d_speed_mm_s":d3/dt*1000 if dt>0 else float("nan"),
    })

def corr(xs,ys):
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    num=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx=math.sqrt(sum((x-mx)**2 for x in xs)); dy=math.sqrt(sum((y-my)**2 for y in ys))
    return num/(dx*dy) if dx and dy else float("nan")

xs=[r["mean_xy_speed_mm_s"] for r in rows]
ys=[r["scale"] for r in rows]
rall=corr(xs,ys)

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V14 SPEED vs SCALE ================")
for r in rows:
    mark="  <-- LEG2" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: duration={r["move_duration_s"]:.3f}s '
          f'meanV={r["mean_xy_speed_mm_s"]:.1f}mm/s XY={r["xy_mm"]:.2f}mm '
          f'scale={r["scale"]:.4f} err={r["error_mm"]:+.2f}mm{mark}')
print(f"all-legs Pearson(speed, scale) = {rall:.4f}")

for direction in ("A->B","B->A"):
    rr=[x for x in rows if x["direction"]==direction]
    if len(rr)>=3:
        print(f'{direction} Pearson(speed, scale) = '
              f'{corr([x["mean_xy_speed_mm_s"] for x in rr],[x["scale"] for x in rr]):.4f}')

normal=[r for r in rows if r["leg"]!=2]
print(f'normal mean speed = {statistics.mean(r["mean_xy_speed_mm_s"] for r in normal):.1f} mm/s')
l2=next(r for r in rows if r["leg"]==2)
print(f'LEG2 mean speed   = {l2["mean_xy_speed_mm_s"]:.1f} mm/s')
print(f'LEG2 speed excess = {l2["mean_xy_speed_mm_s"]/statistics.mean(r["mean_xy_speed_mm_s"] for r in normal)-1:+.1%}')
print("Saved:",OUT)
