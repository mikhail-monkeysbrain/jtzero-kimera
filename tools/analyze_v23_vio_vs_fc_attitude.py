#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics, bisect

BINS=[0.0,0.25,0.50,0.75,1.0]

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def sd(xs):
    if not xs:return float("nan")
    m=mean(xs); return math.sqrt(sum((x-m)**2 for x in xs)/len(xs))
def nearest_by_ts(rows, ts, key):
    vals=[I(r[key]) for r in rows]
    j=bisect.bisect_left(vals,ts)
    cand=[]
    if j<len(rows): cand.append(rows[j])
    if j>0: cand.append(rows[j-1])
    return min(cand,key=lambda r:abs(I(r[key])-ts)) if cand else None

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v23_*")))
out_rows=[]

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v23_backend.csv")
    ap=os.path.join(root,"jtzero_500mm_v23_attitude.csv")
    lp=os.path.join(root,"jtzero_500mm_v23_legs.csv")
    if not all(os.path.exists(p) for p in (bp,ap,lp)):
        print("SKIP",root,"missing backend/attitude/legs")
        continue

    backend=load(bp); att=load(ap); legs=load(lp)
    att.sort(key=lambda r:I(r["recv_ns"]))
    bykf={I(r["keyframe"]):r for r in backend}
    run=os.path.basename(root)

    for lr in legs:
        leg=I(lr["leg"])
        ks=I(lr["start_settled_kf"]); ke=I(lr["end_press_kf"])
        s=bykf.get(ks); e=bykf.get(ke)
        if not s or not e: continue
        t0=I(s["timestamp_ns"]); t1=I(e["timestamp_ns"])
        seg=sorted([r for r in backend if t0<=I(r["timestamp_ns"])<=t1], key=lambda r:I(r["timestamp_ns"]))
        if len(seg)<2: continue

        # FC reference at VIO start callback.
        fcs=nearest_by_ts(att,I(s["callback_wall_ns"]),"recv_ns")
        if not fcs: continue
        fc0r,fc0p,fc0y=F(fcs["roll_deg"]),F(fcs["pitch_deg"]),F(fcs["yaw_deg"])
        vr0,vp0,vy0=F(s["roll_deg"]),F(s["pitch_deg"]),F(s["yaw_deg"])

        for frac in BINS:
            target=int(round(t0+frac*(t1-t0)))
            vr=min(seg,key=lambda r:abs(I(r["timestamp_ns"])-target))
            fc=nearest_by_ts(att,I(vr["callback_wall_ns"]),"recv_ns")
            if not fc: continue

            dv_r=F(vr["roll_deg"])-vr0
            dv_p=F(vr["pitch_deg"])-vp0
            dv_y=F(vr["yaw_deg"])-vy0
            df_r=F(fc["roll_deg"])-fc0r
            df_p=F(fc["pitch_deg"])-fc0p
            df_y=F(fc["yaw_deg"])-fc0y

            out_rows.append({
                "run":run,"leg":leg,"direction":lr["direction"],"bin":frac,
                "vio_kf":I(vr["keyframe"]),
                "fc_match_dt_ms":(I(fc["recv_ns"])-I(vr["callback_wall_ns"]))/1e6,
                "vio_droll_deg":dv_r,"vio_dpitch_deg":dv_p,"vio_dyaw_deg":dv_y,
                "fc_droll_deg":df_r,"fc_dpitch_deg":df_p,"fc_dyaw_deg":df_y,
                "vio_dtilt_deg":math.hypot(dv_r,dv_p),
                "fc_dtilt_deg":math.hypot(df_r,df_p),
            })

print("================ V23 VIO vs FC ATTITUDE ALONG LEG ================")
for d in ("A->B","B->A"):
    print(f"\n{d}")
    rr=[r for r in out_rows if r["direction"]==d]
    for b in BINS:
        x=[r for r in rr if r["bin"]==b]
        if not x: continue
        print(
            f" {int(b*100):3d}%: "
            f"VIO dRP=[{mean([r['vio_droll_deg'] for r in x]):+6.3f},{mean([r['vio_dpitch_deg'] for r in x]):+6.3f}] "
            f"|dTilt|={mean([r['vio_dtilt_deg'] for r in x]):5.3f}deg | "
            f"FC dRP=[{mean([r['fc_droll_deg'] for r in x]):+6.3f},{mean([r['fc_dpitch_deg'] for r in x]):+6.3f}] "
            f"|dTilt|={mean([r['fc_dtilt_deg'] for r in x]):5.3f}deg "
            f"match_dt={mean([abs(r['fc_match_dt_ms']) for r in x]):.1f}ms"
        )

print("\n================ PER-LEG END PRESS ================")
ends=[r for r in out_rows if r["bin"]==1.0]
for r in ends:
    ratio=r["vio_dtilt_deg"]/r["fc_dtilt_deg"] if r["fc_dtilt_deg"]>1e-6 else float("inf")
    print(
        f"{r['run']} LEG{r['leg']} {r['direction']}: "
        f"VIO dRP=[{r['vio_droll_deg']:+.3f},{r['vio_dpitch_deg']:+.3f}] tilt={r['vio_dtilt_deg']:.3f}deg | "
        f"FC dRP=[{r['fc_droll_deg']:+.3f},{r['fc_dpitch_deg']:+.3f}] tilt={r['fc_dtilt_deg']:.3f}deg | "
        f"VIO/FC tilt ratio={ratio:.2f} match_dt={r['fc_match_dt_ms']:+.1f}ms"
    )

print("\n================ END-PRESS DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in ends if r["direction"]==d]
    if rr:
        print(
            f"{d}: VIO tilt={mean([r['vio_dtilt_deg'] for r in rr]):.3f}±{sd([r['vio_dtilt_deg'] for r in rr]):.3f}deg "
            f"FC tilt={mean([r['fc_dtilt_deg'] for r in rr]):.3f}±{sd([r['fc_dtilt_deg'] for r in rr]):.3f}deg"
        )

out=os.path.expanduser("~/jtzero_v23_vio_vs_fc_attitude.csv")
if out_rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(out_rows[0].keys()))
        w.writeheader(); w.writerows(out_rows)
    print(f"\nSaved: {out}")
