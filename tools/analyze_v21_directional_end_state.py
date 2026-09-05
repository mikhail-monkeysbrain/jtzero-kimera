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
        leg=I(lr["leg"])
        ks=I(lr["start_settled_kf"]); ke=I(lr["end_press_kf"])
        s=bykf.get(ks); e=bykf.get(ke)
        if not s or not e: continue

        dx=F(e["px_m"])-F(s["px_m"])
        dy=F(e["py_m"])-F(s["py_m"])
        n=math.hypot(dx,dy)
        if n<1e-9: continue
        ux,uy=dx/n,dy/n

        vx=F(e["vx_m_s"])*1000.0; vy=F(e["vy_m_s"])*1000.0
        vend_proj=vx*ux+vy*uy

        rec={
            "run":run,"leg":leg,"direction":lr["direction"],
            "end_xy_mm":n*1000.0,
            "end_error_mm":n*1000.0-500.0,
            "vend_proj_mm_s":vend_proj,
            "bax_start":F(s["bax"]),"bay_start":F(s["bay"]),"baz_start":F(s["baz"]),
            "bax_end":F(e["bax"]),"bay_end":F(e["bay"]),"baz_end":F(e["baz"]),
            "dbax_move":F(e["bax"])-F(s["bax"]),
            "dbay_move":F(e["bay"])-F(s["bay"]),
            "dbaz_move":F(e["baz"])-F(s["baz"]),
            "bgx_end":F(e["bgx"]),"bgy_end":F(e["bgy"]),"bgz_end":F(e["bgz"]),
            "roll_end":F(e["roll_deg"]),"pitch_end":F(e["pitch_deg"]),"yaw_end":F(e["yaw_deg"]),
        }
        rows.append(rec)

print("================ V21 DIRECTIONAL END-STATE DIAGNOSTICS ================")
for r in rows:
    print(
        f"{r['run']} LEG {r['leg']} {r['direction']}: "
        f"ENDxy={r['end_xy_mm']:.2f} err={r['end_error_mm']:+.2f} "
        f"VendProj={r['vend_proj_mm_s']:+.2f}mm/s "
        f"BAend=[{r['bax_end']:+.5f},{r['bay_end']:+.5f},{r['baz_end']:+.5f}] "
        f"dBA_move=[{r['dbax_move']:+.5f},{r['dbay_move']:+.5f},{r['dbaz_move']:+.5f}]"
    )

print("\n================ DIRECTION GROUP MEANS ================")
metrics=["end_xy_mm","end_error_mm","vend_proj_mm_s",
         "bax_start","bay_start","baz_start","bax_end","bay_end","baz_end",
         "dbax_move","dbay_move","dbaz_move","bgx_end","bgy_end","bgz_end",
         "roll_end","pitch_end","yaw_end"]
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    print(f"{d}: n={len(rr)}")
    for m in metrics:
        vals=[r[m] for r in rr]
        print(f"  {m}: mean={statistics.mean(vals):+.6f} sd={statistics.pstdev(vals):.6f}")

print("\n================ CORRELATION WITH END ERROR ================")
y=[r["end_error_mm"] for r in rows]
for m in ["vend_proj_mm_s","bax_start","bay_start","baz_start","bax_end","bay_end","baz_end",
          "dbax_move","dbay_move","dbaz_move","bgx_end","bgy_end","bgz_end",
          "roll_end","pitch_end","yaw_end"]:
    print(f"Pearson({m}, end_error_mm)={pearson([r[m] for r in rows],y):+.4f}")

print("\n================ CORRELATION WITH END RESIDUAL VELOCITY ================")
y=[r["vend_proj_mm_s"] for r in rows]
for m in ["bax_start","bay_start","baz_start","bax_end","bay_end","baz_end",
          "dbax_move","dbay_move","dbaz_move","bgx_end","bgy_end","bgz_end",
          "roll_end","pitch_end","yaw_end"]:
    print(f"Pearson({m}, VendProj)={pearson([r[m] for r in rows],y):+.4f}")

out=os.path.expanduser("~/jtzero_v21_directional_end_state.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
