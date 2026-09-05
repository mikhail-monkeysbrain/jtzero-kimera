#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def pearson(xs,ys):
    if len(xs)<3: return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    vx=sum((x-mx)**2 for x in xs); vy=sum((y-my)**2 for y in ys)
    if vx<=0 or vy<=0: return float("nan")
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
        ks=I(lr["start_settled_kf"]); kep=I(lr["end_press_kf"]); kes=I(lr["end_settled_kf"])
        s=bykf.get(ks); ep=bykf.get(kep); es=bykf.get(kes)
        if not s or not ep or not es: continue

        dx=F(ep["px_m"])-F(s["px_m"])
        dy=F(ep["py_m"])-F(s["py_m"])
        n=math.hypot(dx,dy)
        if n<1e-9: continue
        ux,uy=dx/n,dy/n

        vepx=F(ep["vx_m_s"])*1000.0
        vepy=F(ep["vy_m_s"])*1000.0
        vepz=F(ep["vz_m_s"])*1000.0
        vproj_end=vepx*ux+vepy*uy
        vcross_end=-vepx*uy+vepy*ux

        seg=sorted([r for r in backend if kep<=I(r["keyframe"])<=kes], key=lambda r:I(r["keyframe"]))
        proj_samples=[]
        cross_samples=[]
        times=[]
        for r in seg:
            vx=F(r["vx_m_s"])*1000.0
            vy=F(r["vy_m_s"])*1000.0
            proj_samples.append(vx*ux+vy*uy)
            cross_samples.append(-vx*uy+vy*ux)
            times.append(I(r["timestamp_ns"])/1e9)

        # trapezoidal integral of projected velocity over settle
        integ=0.0
        integ_cross=0.0
        for j in range(1,len(seg)):
            dt=times[j]-times[j-1]
            if dt<=0: continue
            integ += 0.5*(proj_samples[j]+proj_samples[j-1])*dt
            integ_cross += 0.5*(cross_samples[j]+cross_samples[j-1])*dt

        settle_dx=(F(es["px_m"])-F(ep["px_m"]))*1000.0
        settle_dy=(F(es["py_m"])-F(ep["py_m"]))*1000.0
        settle_proj=settle_dx*ux+settle_dy*uy
        settle_cross=-settle_dx*uy+settle_dy*ux

        xy_press=n*1000.0
        xy_settle=math.hypot(F(es["px_m"])-F(s["px_m"]),F(es["py_m"])-F(s["py_m"]))*1000.0

        rows.append({
            "run":run,"leg":leg,"direction":lr["direction"],
            "xy_press_mm":xy_press,"xy_settle_mm":xy_settle,
            "settle_proj_mm":settle_proj,"settle_cross_mm":settle_cross,
            "vproj_end_mm_s":vproj_end,"vcross_end_mm_s":vcross_end,"vz_end_mm_s":vepz,
            "mean_vproj_settle_mm_s":sum(proj_samples)/len(proj_samples),
            "mean_abs_cross_settle_mm_s":sum(abs(x) for x in cross_samples)/len(cross_samples),
            "integrated_vproj_mm":integ,
            "integrated_vcross_mm":integ_cross,
            "settle_duration_s":times[-1]-times[0] if len(times)>1 else 0.0,
        })

print("================ V21 RESIDUAL VELOCITY PROJECTION ================")
for r in rows:
    print(
        f"{r['run']} LEG {r['leg']} {r['direction']}: "
        f"ENDpress={r['xy_press_mm']:.2f}mm settled={r['xy_settle_mm']:.2f}mm "
        f"settleProj={r['settle_proj_mm']:+.2f}mm cross={r['settle_cross_mm']:+.2f}mm | "
        f"VendProj={r['vproj_end_mm_s']:+.1f}mm/s cross={r['vcross_end_mm_s']:+.1f} "
        f"meanProj={r['mean_vproj_settle_mm_s']:+.1f}mm/s intProj={r['integrated_vproj_mm']:+.2f}mm"
    )

print("\n================ DIRECTION GROUPS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    print(
        f"{d}: n={len(rr)} "
        f"ENDprojV={statistics.mean(r['vproj_end_mm_s'] for r in rr):+.2f}mm/s "
        f"meanSettleProjV={statistics.mean(r['mean_vproj_settle_mm_s'] for r in rr):+.2f}mm/s "
        f"settleProj={statistics.mean(r['settle_proj_mm'] for r in rr):+.2f}mm "
        f"cross={statistics.mean(abs(r['settle_cross_mm']) for r in rr):.2f}mm"
    )

print("\n================ CORRELATIONS ================")
print(f"Pearson(VendProj, settleProj)={pearson([r['vproj_end_mm_s'] for r in rows],[r['settle_proj_mm'] for r in rows]):+.4f}")
print(f"Pearson(meanSettleProjV, settleProj)={pearson([r['mean_vproj_settle_mm_s'] for r in rows],[r['settle_proj_mm'] for r in rows]):+.4f}")
print(f"Pearson(integratedVproj, settleProj)={pearson([r['integrated_vproj_mm'] for r in rows],[r['settle_proj_mm'] for r in rows]):+.4f}")
print(f"Pearson(absCrossV, absSettleCross)={pearson([abs(r['mean_abs_cross_settle_mm_s']) for r in rows],[abs(r['settle_cross_mm']) for r in rows]):+.4f}")

print("\n================ ASYMMETRY AT END ================")
ab=[r for r in rows if r["direction"]=="A->B"]; ba=[r for r in rows if r["direction"]=="B->A"]
if ab and ba:
    print(f"A->B ENDpress mean={statistics.mean(r['xy_press_mm'] for r in ab):.2f}mm")
    print(f"B->A ENDpress mean={statistics.mean(r['xy_press_mm'] for r in ba):.2f}mm")
    print(f"A->B VendProj mean={statistics.mean(r['vproj_end_mm_s'] for r in ab):+.2f}mm/s")
    print(f"B->A VendProj mean={statistics.mean(r['vproj_end_mm_s'] for r in ba):+.2f}mm/s")

out=os.path.expanduser("~/jtzero_v21_residual_velocity_projection.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
