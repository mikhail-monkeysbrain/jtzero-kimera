#!/usr/bin/env python3
import csv, glob, math, os, statistics, sys

BINS=[0.0,0.25,0.50,0.75,1.0]

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def norm2(x,y): return math.hypot(x,y)
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def sd(xs):
    if not xs:return float("nan")
    m=mean(xs); return math.sqrt(sum((x-m)**2 for x in xs)/len(xs))
def nearest(seg,target):
    return min(seg,key=lambda r:abs(I(r["timestamp_ns"])-target))

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v23_*")))
rows=[]

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v23_backend.csv")
    lp=os.path.join(root,"jtzero_500mm_v23_legs.csv")
    if not(os.path.exists(bp) and os.path.exists(lp)): continue
    backend=load(bp); legs=load(lp); bykf={I(r["keyframe"]):r for r in backend}
    run=os.path.basename(root)
    for lr in legs:
        leg=I(lr["leg"]); ks=I(lr["start_settled_kf"]); ke=I(lr["end_press_kf"])
        s=bykf.get(ks); e=bykf.get(ke)
        if not s or not e: continue
        t0=I(s["timestamp_ns"]); t1=I(e["timestamp_ns"])
        seg=sorted([r for r in backend if t0<=I(r["timestamp_ns"])<=t1],key=lambda r:I(r["timestamp_ns"]))
        if len(seg)<2 or t1<=t0: continue

        # Estimated physical line for this leg. Positive u always means from that leg's start toward its end.
        ex=F(e["px_m"])-F(s["px_m"]); ey=F(e["py_m"])-F(s["py_m"])
        en=norm2(ex,ey)
        if en<1e-9: continue
        ux,uy=ex/en,ey/en
        sx,sy,sz=F(s["px_m"]),F(s["py_m"]),F(s["pz_m"])
        sroll,spitch,syaw=F(s["roll_deg"]),F(s["pitch_deg"]),F(s["yaw_deg"])
        sbax,sbay,sbaz=F(s["bax"]),F(s["bay"]),F(s["baz"])

        for frac in BINS:
            target=int(round(t0+frac*(t1-t0)))
            r=nearest(seg,target)
            dx=(F(r["px_m"])-sx)*1000.0; dy=(F(r["py_m"])-sy)*1000.0
            along=dx*ux+dy*uy
            cross=-dx*uy+dy*ux
            vx=F(r["vx_m_s"])*1000.0; vy=F(r["vy_m_s"])*1000.0
            valong=vx*ux+vy*uy
            vcross=-vx*uy+vy*ux
            rows.append({
                "run":run,"leg":leg,"direction":lr["direction"],"bin":frac,
                "actual_time_frac":(I(r["timestamp_ns"])-t0)/(t1-t0),
                "along_mm":along,"cross_mm":cross,
                "z_change_mm":(F(r["pz_m"])-sz)*1000.0,
                "v_along_mm_s":valong,"v_cross_mm_s":vcross,"v_z_mm_s":F(r["vz_m_s"])*1000.0,
                "droll_deg":F(r["roll_deg"])-sroll,
                "dpitch_deg":F(r["pitch_deg"])-spitch,
                "dyaw_deg":F(r["yaw_deg"])-syaw,
                "dbax":F(r["bax"])-sbax,"dbay":F(r["bay"])-sbay,"dbaz":F(r["baz"])-sbaz,
            })

print("================ V23 ALONG-LEG EVOLUTION ================")
for d in ("A->B","B->A"):
    print(f"\n{d}")
    rr=[r for r in rows if r["direction"]==d]
    for b in BINS:
        x=[r for r in rr if r["bin"]==b]
        if not x: continue
        print(
          f"  {int(b*100):3d}%: along={mean([r['along_mm'] for r in x]):7.2f}±{sd([r['along_mm'] for r in x]):5.2f}mm "
          f"V={mean([r['v_along_mm_s'] for r in x]):+7.2f}mm/s "
          f"Z={mean([r['z_change_mm'] for r in x]):+7.2f}mm "
          f"dRP=[{mean([r['droll_deg'] for r in x]):+6.3f},{mean([r['dpitch_deg'] for r in x]):+6.3f}]deg "
          f"dBA=[{mean([r['dbax'] for r in x]):+7.4f},{mean([r['dbay'] for r in x]):+7.4f},{mean([r['dbaz'] for r in x]):+7.4f}]"
        )

print("\n================ B->A MINUS A->B BY FRACTION ================")
for b in BINS:
    a=[r for r in rows if r["direction"]=="A->B" and r["bin"]==b]
    q=[r for r in rows if r["direction"]=="B->A" and r["bin"]==b]
    if not a or not q: continue
    def delta(k): return mean([r[k] for r in q])-mean([r[k] for r in a])
    print(
      f"{int(b*100):3d}%: Δalong={delta('along_mm'):+7.2f}mm "
      f"ΔV={delta('v_along_mm_s'):+7.2f}mm/s "
      f"ΔZ={delta('z_change_mm'):+7.2f}mm "
      f"ΔdRP=[{delta('droll_deg'):+6.3f},{delta('dpitch_deg'):+6.3f}]deg "
      f"ΔdBA=[{delta('dbax'):+7.4f},{delta('dbay'):+7.4f},{delta('dbaz'):+7.4f}]"
    )

print("\n================ PER-LEG END-PRESS STATE ================")
for run in sorted(set(r["run"] for r in rows)):
    for leg in sorted(set(r["leg"] for r in rows if r["run"]==run)):
        x=[r for r in rows if r["run"]==run and r["leg"]==leg and r["bin"]==1.0]
        if not x: continue
        r=x[0]
        print(
          f"{run} LEG{leg} {r['direction']}: along={r['along_mm']:.2f}mm "
          f"Vend={r['v_along_mm_s']:+.2f}mm/s Z={r['z_change_mm']:+.2f}mm "
          f"dRP=[{r['droll_deg']:+.3f},{r['dpitch_deg']:+.3f}] "
          f"dBA=[{r['dbax']:+.5f},{r['dbay']:+.5f},{r['dbaz']:+.5f}]"
        )

out=os.path.expanduser("~/jtzero_v23_along_leg_evolution.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
