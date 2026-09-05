#!/usr/bin/env python3
import csv, math, statistics
from collections import Counter

H="/home/vio"
LEGS=H+"/jtzero_500mm_v19_legs.csv"
BACK=H+"/jtzero_500mm_v19_backend.csv"
FRONT=H+"/jtzero_500mm_v19_frontend.csv"
OUT=H+"/jtzero_v19_speed_disparity.csv"
TRUE_MM=500.0

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS)
be=read(BACK)
fe=read(FRONT)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

rows=[]
print("================ V19 SPEED / LOW-DISPARITY ================")
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    dt=(t1-t0)*1e-9
    xy=fv(L,"horizontal_m")*1000
    k=[r for r in fe if iv(r,"is_keyframe")!=0 and t0<=iv(r,"timestamp_ns")<=t1]
    st=Counter(r["mono_status"] for r in k)
    low=st.get("LOW_DISPARITY",0)
    valid=st.get("VALID",0)
    few=st.get("FEW_MATCHES",0)
    ratios=[fv(r,"mono_inlier_ratio") for r in k if fv(r,"mono_putatives")>0]
    tracked=[fv(r,"tracked_features") for r in k]
    true_v=TRUE_MM/dt if dt>0 else float("nan")
    vio_v=xy/dt if dt>0 else float("nan")
    row={
      "leg":leg,"direction":L["direction"],"duration_s":dt,
      "true_speed_mm_s":true_v,"vio_xy_mm":xy,"scale":xy/TRUE_MM,
      "kf":len(k),"low_disparity":low,"low_fraction":low/len(k) if k else float("nan"),
      "valid":valid,"few_matches":few,
      "inlier_ratio_mean":statistics.mean(ratios) if ratios else float("nan"),
      "tracked_mean":statistics.mean(tracked) if tracked else float("nan"),
    }
    rows.append(row)
    print(f'LEG {leg} {L["direction"]}: dur={dt:.2f}s trueV={true_v:.1f}mm/s '
          f'XY={xy:.1f}mm scale={xy/TRUE_MM:.3f} KF={len(k)} '
          f'LOW={low}/{len(k)} ({100*row["low_fraction"]:.1f}%) '
          f'VALID={valid} FEW={few} inlierMean={row["inlier_ratio_mean"]:.3f}')

if rows:
    with open(OUT,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

def corr(a,b):
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    n=sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da=math.sqrt(sum((x-ma)**2 for x in a)); db=math.sqrt(sum((y-mb)**2 for y in b))
    return n/(da*db) if da and db else float("nan")

print("\n================ CORRELATION ================")
print(f'Pearson(TRUE speed, scale) = {corr([r["true_speed_mm_s"] for r in rows],[r["scale"] for r in rows]):.4f}')
print(f'Pearson(LOW fraction, scale) = {corr([r["low_fraction"] for r in rows],[r["scale"] for r in rows]):.4f}')

slow=sorted(rows,key=lambda r:r["true_speed_mm_s"])[:2]
fast=sorted(rows,key=lambda r:r["true_speed_mm_s"])[2:]
print("\nSLOWEST TWO:")
for r in slow:
    print(f'LEG {r["leg"]}: trueV={r["true_speed_mm_s"]:.1f} scale={r["scale"]:.3f} LOW={100*r["low_fraction"]:.1f}%')
print("OTHER FOUR mean:")
print(f'trueV={statistics.mean(r["true_speed_mm_s"] for r in fast):.1f} '
      f'scale={statistics.mean(r["scale"] for r in fast):.3f} '
      f'LOW={100*statistics.mean(r["low_fraction"] for r in fast):.1f}%')
print("Saved:",OUT)
