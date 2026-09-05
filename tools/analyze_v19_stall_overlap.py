#!/usr/bin/env python3
import csv, math

H="/home/vio"
LEGS=H+"/jtzero_500mm_v19_legs.csv"
BACK=H+"/jtzero_500mm_v19_backend.csv"
OUT=H+"/jtzero_v19_leg_stall_overlap.csv"

# Stalls copied from the V19 watchdog log supplied for this run.
# wall-clock association is unavailable in backend CSV, so this tool uses the
# watchdog phase/leg annotations already printed by V19.
stalls = {
  1: [],
  2: [],
  3: [],
  4: [1065.232],
  5: [],  # 1507.223 ms was phase=1 (START settling), not MOVING.
  6: [],
}
pre_or_settle = {
  1: [436.623], # warmup, not leg motion
  2: [],
  3: [137.083], # phase=0 before LEG3 START
  4: [],
  5: [1507.223], # phase=1 START settling
  6: [],
}

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS)
be=read(BACK)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

rows=[]
print("================ V19 SPEED vs MOVING-STALL ================")
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    dur=(t1-t0)/1e9
    xy=fv(L,"horizontal_m")*1000
    true_v=500.0/dur
    ms=stalls.get(leg,[])
    row=dict(
      leg=leg,direction=L["direction"],duration_s=dur,true_speed_mm_s=true_v,
      xy_mm=xy,scale=xy/500.0,moving_stall_count=len(ms),
      moving_stall_max_ms=max(ms) if ms else 0.0,
      outside_motion_stall_count=len(pre_or_settle.get(leg,[])),
      outside_motion_stall_max_ms=max(pre_or_settle.get(leg,[])) if pre_or_settle.get(leg) else 0.0
    )
    rows.append(row)
    print(f'LEG {leg} {L["direction"]}: trueV={true_v:.1f}mm/s scale={xy/500:.3f} '
          f'MOVING_STALLS={len(ms)} max={row["moving_stall_max_ms"]:.1f}ms '
          f'outside={row["outside_motion_stall_count"]}')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("\nKEY OBSERVATION:")
l3=next(r for r in rows if r["leg"]==3)
print(f'LEG3: trueV={l3["true_speed_mm_s"]:.1f}mm/s scale={l3["scale"]:.3f}, moving stalls={l3["moving_stall_count"]}')
print("If LEG3 has zero MOVING stalls yet severe scale collapse, speed/observability is independent of the disk/main-loop stall issue.")
print("Saved:",OUT)
