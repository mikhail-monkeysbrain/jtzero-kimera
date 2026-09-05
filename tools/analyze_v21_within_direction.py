#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
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
        leg=I(lr["leg"]); ks=I(lr["start_settled_kf"]); ke=I(lr["end_press_kf"])
        s=bykf.get(ks); e=bykf.get(ke)
        if not s or not e: continue
        dx=F(e["px_m"])-F(s["px_m"]); dy=F(e["py_m"])-F(s["py_m"])
        n=math.hypot(dx,dy)
        if n<1e-9: continue
        ux,uy=dx/n,dy/n
        vx=F(e["vx_m_s"])*1000; vy=F(e["vy_m_s"])*1000
        rows.append({
            "run":run,"leg":leg,"direction":lr["direction"],
            "end_error_mm":n*1000-500,
            "vend_proj_mm_s":vx*ux+vy*uy,
            "bax_start":F(s["bax"]),"bay_start":F(s["bay"]),"baz_start":F(s["baz"]),
            "bax_end":F(e["bax"]),"bay_end":F(e["bay"]),"baz_end":F(e["baz"]),
            "dbax_move":F(e["bax"])-F(s["bax"]),
            "dbay_move":F(e["bay"])-F(s["bay"]),
            "dbaz_move":F(e["baz"])-F(s["baz"]),
            "roll_end":F(e["roll_deg"]),"pitch_end":F(e["pitch_deg"]),"yaw_end":F(e["yaw_deg"]),
            "bgx_end":F(e["bgx"]),"bgy_end":F(e["bgy"]),"bgz_end":F(e["bgz"]),
        })

metrics=["vend_proj_mm_s","bax_start","bay_start","baz_start","bax_end","bay_end","baz_end",
         "dbax_move","dbay_move","dbaz_move","roll_end","pitch_end","yaw_end",
         "bgx_end","bgy_end","bgz_end"]

print("================ WITHIN-DIRECTION CORRELATIONS: END ERROR ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    y=[r["end_error_mm"] for r in rr]
    print(f"\n{d} n={len(rr)} error mean={statistics.mean(y):+.2f} sd={statistics.pstdev(y):.2f}")
    for m in metrics:
        print(f"  Pearson({m}, end_error)={pearson([r[m] for r in rr],y):+.4f}")

print("\n================ WITHIN-DIRECTION CORRELATIONS: RESIDUAL VELOCITY ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    y=[r["vend_proj_mm_s"] for r in rr]
    print(f"\n{d} n={len(rr)} VendProj mean={statistics.mean(y):+.2f} sd={statistics.pstdev(y):.2f}")
    for m in [x for x in metrics if x!="vend_proj_mm_s"]:
        print(f"  Pearson({m}, VendProj)={pearson([r[m] for r in rr],y):+.4f}")

# Direction-centered residual correlations: remove group means from every variable.
centered=[]
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    means={k:statistics.mean(r[k] for r in rr) for k in ["end_error_mm","vend_proj_mm_s"]+metrics[1:]}
    for r in rr:
        c={"direction":d}
        for k in means: c[k]=r[k]-means[k]
        centered.append(c)

print("\n================ DIRECTION-CENTERED CORRELATIONS ================")
y=[r["end_error_mm"] for r in centered]
for m in metrics:
    print(f"Pearson(centered {m}, centered end_error)={pearson([r[m] for r in centered],y):+.4f}")

print("\n================ DIRECTION-CENTERED vs RESIDUAL VELOCITY ================")
yv=[r["vend_proj_mm_s"] for r in centered]
for m in [x for x in metrics if x!="vend_proj_mm_s"]:
    print(f"Pearson(centered {m}, centered VendProj)={pearson([r[m] for r in centered],yv):+.4f}")

out=os.path.expanduser("~/jtzero_v21_within_direction.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
