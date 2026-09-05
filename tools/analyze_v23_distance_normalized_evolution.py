#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

FRACTIONS=[0.0,0.25,0.50,0.75,1.0]

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def sd(xs):
    if not xs:return float("nan")
    m=mean(xs); return math.sqrt(sum((x-m)**2 for x in xs)/len(xs))

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
        seg=sorted([r for r in backend if t0<=I(r["timestamp_ns"])<=t1], key=lambda r:I(r["timestamp_ns"]))
        if len(seg)<2: continue

        sx,sy,sz=F(s["px_m"]),F(s["py_m"]),F(s["pz_m"])
        ex,ey=F(e["px_m"])-sx,F(e["py_m"])-sy
        en=math.hypot(ex,ey)
        if en<1e-9: continue
        ux,uy=ex/en,ey/en

        sroll,spitch,syaw=F(s["roll_deg"]),F(s["pitch_deg"]),F(s["yaw_deg"])
        sbax,sbay,sbaz=F(s["bax"]),F(s["bay"]),F(s["baz"])

        pts=[]
        for r in seg:
            dx=(F(r["px_m"])-sx)*1000.0
            dy=(F(r["py_m"])-sy)*1000.0
            along=dx*ux+dy*uy
            cross=-dx*uy+dy*ux
            pts.append((along,r,cross))

        final_along=pts[-1][0]
        if abs(final_along)<1e-6: continue

        for frac in FRACTIONS:
            target=frac*final_along
            # choose state with closest projected along-distance target
            along,r,cross=min(pts,key=lambda z:abs(z[0]-target))
            vx=F(r["vx_m_s"])*1000.0; vy=F(r["vy_m_s"])*1000.0
            rows.append({
                "run":run,"leg":leg,"direction":lr["direction"],"fraction":frac,
                "actual_along_fraction":along/final_along if final_along else 0.0,
                "along_mm":along,"cross_mm":cross,
                "z_change_mm":(F(r["pz_m"])-sz)*1000.0,
                "v_along_mm_s":vx*ux+vy*uy,
                "v_cross_mm_s":-vx*uy+vy*ux,
                "v_z_mm_s":F(r["vz_m_s"])*1000.0,
                "droll_deg":F(r["roll_deg"])-sroll,
                "dpitch_deg":F(r["pitch_deg"])-spitch,
                "dyaw_deg":F(r["yaw_deg"])-syaw,
                "dbax":F(r["bax"])-sbax,
                "dbay":F(r["bay"])-sbay,
                "dbaz":F(r["baz"])-sbaz,
            })

print("================ V23 DISTANCE-NORMALIZED EVOLUTION ================")
for d in ("A->B","B->A"):
    print(f"\n{d}")
    rr=[r for r in rows if r["direction"]==d]
    for frac in FRACTIONS:
        x=[r for r in rr if r["fraction"]==frac]
        if not x: continue
        print(
            f" {int(frac*100):3d}%: along={mean([r['along_mm'] for r in x]):7.2f}±{sd([r['along_mm'] for r in x]):5.2f}mm "
            f"V={mean([r['v_along_mm_s'] for r in x]):+7.2f}mm/s "
            f"Z={mean([r['z_change_mm'] for r in x]):+7.2f}mm "
            f"dRP=[{mean([r['droll_deg'] for r in x]):+6.3f},{mean([r['dpitch_deg'] for r in x]):+6.3f}]deg "
            f"dBA=[{mean([r['dbax'] for r in x]):+7.4f},{mean([r['dbay'] for r in x]):+7.4f},{mean([r['dbaz'] for r in x]):+7.4f}] "
            f"actualFrac={mean([r['actual_along_fraction'] for r in x]):.3f}"
        )

print("\n================ B->A MINUS A->B BY DISTANCE FRACTION ================")
for frac in FRACTIONS:
    a=[r for r in rows if r["direction"]=="A->B" and r["fraction"]==frac]
    b=[r for r in rows if r["direction"]=="B->A" and r["fraction"]==frac]
    if not a or not b: continue
    def D(k): return mean([r[k] for r in b])-mean([r[k] for r in a])
    print(
        f"{int(frac*100):3d}%: Δalong={D('along_mm'):+7.2f}mm "
        f"ΔV={D('v_along_mm_s'):+7.2f}mm/s "
        f"ΔZ={D('z_change_mm'):+7.2f}mm "
        f"ΔdRP=[{D('droll_deg'):+6.3f},{D('dpitch_deg'):+6.3f}]deg "
        f"ΔdBA=[{D('dbax'):+7.4f},{D('dbay'):+7.4f},{D('dbaz'):+7.4f}]"
    )

out=os.path.expanduser("~/jtzero_v23_distance_normalized_evolution.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
