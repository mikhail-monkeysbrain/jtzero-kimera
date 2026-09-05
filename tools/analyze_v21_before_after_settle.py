#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def d3(a,b):
    return math.sqrt((F(a["px_m"])-F(b["px_m"]))**2+
                     (F(a["py_m"])-F(b["py_m"]))**2+
                     (F(a["pz_m"])-F(b["pz_m"]))**2)
def dxy(a,b):
    return math.hypot(F(a["px_m"])-F(b["px_m"]),
                      F(a["py_m"])-F(b["py_m"]))
def pearson(xs,ys):
    if len(xs)<3:return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    vx=sum((x-mx)**2 for x in xs); vy=sum((y-my)**2 for y in ys)
    if vx<=0 or vy<=0:return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys))/math.sqrt(vx*vy)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v21_*")))
rows=[]

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v21_backend.csv")
    lp=os.path.join(root,"jtzero_500mm_v18_legs.csv")
    if not (os.path.exists(bp) and os.path.exists(lp)): continue
    backend=load(bp); legs=load(lp); bykf={I(r["keyframe"]):r for r in backend}
    run=os.path.basename(root)

    for lr in legs:
        leg=I(lr["leg"])
        ks=I(lr["start_settled_kf"])
        ke_press=I(lr["end_press_kf"])
        ke_set=I(lr["end_settled_kf"])
        s=bykf.get(ks); ep=bykf.get(ke_press); es=bykf.get(ke_set)
        if not s or not ep or not es: continue

        xy_press=dxy(s,ep)*1000
        xy_settle=dxy(s,es)*1000
        settle_xy=dxy(ep,es)*1000
        settle_3d=d3(ep,es)*1000
        corr_to_truth=abs(xy_press-500)-abs(xy_settle-500)
        rows.append({
            "run":run,"leg":leg,"direction":lr["direction"],
            "xy_at_end_press_mm":xy_press,
            "xy_at_settled_end_mm":xy_settle,
            "settle_xy_mm":settle_xy,
            "settle_3d_mm":settle_3d,
            "press_error_mm":xy_press-500,
            "settled_error_mm":xy_settle-500,
            "settle_delta_xy_signed_mm":xy_settle-xy_press,
            "settle_improvement_abs_error_mm":corr_to_truth,
        })

print("================ V21 ERROR BEFORE vs AFTER SETTLE ================")
for r in rows:
    print(
      f"{r['run']} LEG {r['leg']} {r['direction']}: "
      f"ENDpressXY={r['xy_at_end_press_mm']:.2f} err={r['press_error_mm']:+.2f} | "
      f"settledXY={r['xy_at_settled_end_mm']:.2f} err={r['settled_error_mm']:+.2f} | "
      f"settleΔXY={r['settle_delta_xy_signed_mm']:+.2f} settle3D={r['settle_3d_mm']:.2f}"
    )

print("\n================ DIRECTION GROUPS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    pm=statistics.mean(r["xy_at_end_press_mm"] for r in rr)
    ps=statistics.pstdev(r["xy_at_end_press_mm"] for r in rr)
    sm=statistics.mean(r["xy_at_settled_end_mm"] for r in rr)
    ss=statistics.pstdev(r["xy_at_settled_end_mm"] for r in rr)
    dm=statistics.mean(r["settle_delta_xy_signed_mm"] for r in rr)
    print(f"{d}: n={len(rr)} ENDpress={pm:.2f}±{ps:.2f}mm settled={sm:.2f}±{ss:.2f}mm meanSettleΔXY={dm:+.2f}mm")

ab=[r for r in rows if r["direction"]=="A->B"]
ba=[r for r in rows if r["direction"]=="B->A"]
if ab and ba:
    press_gap=statistics.mean(r["xy_at_end_press_mm"] for r in ba)-statistics.mean(r["xy_at_end_press_mm"] for r in ab)
    settled_gap=statistics.mean(r["xy_at_settled_end_mm"] for r in ba)-statistics.mean(r["xy_at_settled_end_mm"] for r in ab)
    print("\n================ ASYMMETRY DECOMPOSITION ================")
    print(f"B->A minus A->B at END press : {press_gap:+.2f} mm")
    print(f"B->A minus A->B after settle : {settled_gap:+.2f} mm")
    print(f"additional asymmetry created during settle: {settled_gap-press_gap:+.2f} mm")

print("\n================ RELATION TO SETTLE ================")
print(f"Pearson(settle3D, signed settle ΔXY)={pearson([r['settle_3d_mm'] for r in rows],[r['settle_delta_xy_signed_mm'] for r in rows]):+.4f}")
print(f"Pearson(settle3D, settled error)={pearson([r['settle_3d_mm'] for r in rows],[r['settled_error_mm'] for r in rows]):+.4f}")

out=os.path.expanduser("~/jtzero_v21_before_after_settle.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
