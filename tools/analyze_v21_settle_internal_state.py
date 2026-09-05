#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

def load(path):
    with open(path,newline="") as f:
        return list(csv.DictReader(f))

def F(x): return float(x)
def I(x): return int(float(x))

def norm3(a,b,c): return math.sqrt(a*a+b*b+c*c)
def pearson(xs,ys):
    if len(xs)<3: return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    vx=sum((x-mx)**2 for x in xs); vy=sum((y-my)**2 for y in ys)
    if vx<=0 or vy<=0: return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys))/math.sqrt(vx*vy)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v21_*")))
if not roots:
    raise SystemExit("No V21 archives found")

rows=[]
by_key={}

for root in roots:
    evp=os.path.join(root,"jtzero_500mm_v21_events.csv")
    bep=os.path.join(root,"jtzero_500mm_v21_backend.csv")
    lep=os.path.join(root,"jtzero_500mm_v18_legs.csv")
    if not all(os.path.exists(x) for x in (evp,bep,lep)): continue

    events=load(evp); backend=load(bep); legs=load(lep)
    ends={I(e["leg"]):e for e in events if e["event"]=="END"}
    bykf={I(r["keyframe"]):r for r in backend}
    run=os.path.basename(root)

    for lr in legs:
        leg=I(lr["leg"])
        e=ends.get(leg)
        if not e: continue
        k0=I(lr["end_press_kf"]); k1=I(lr["end_settled_kf"])
        r0=bykf.get(k0); r1=bykf.get(k1)
        if not r0 or not r1: continue
        ew=I(e["event_wall_ns"])
        seg=[r for r in backend if k0<=I(r["keyframe"])<=k1]
        if len(seg)<2: continue

        def vals(r):
            return {
                "v": norm3(F(r["vx_m_s"]),F(r["vy_m_s"]),F(r["vz_m_s"]))*1000.0,
                "ba": norm3(F(r["bax"]),F(r["bay"]),F(r["baz"])),
                "bg": norm3(F(r["bgx"]),F(r["bgy"]),F(r["bgz"])),
                "roll":F(r["roll_deg"]),"pitch":F(r["pitch_deg"]),"yaw":F(r["yaw_deg"]),
                "bax":F(r["bax"]),"bay":F(r["bay"]),"baz":F(r["baz"]),
                "bgx":F(r["bgx"]),"bgy":F(r["bgy"]),"bgz":F(r["bgz"]),
                "vx":F(r["vx_m_s"])*1000.0,"vy":F(r["vy_m_s"])*1000.0,"vz":F(r["vz_m_s"])*1000.0,
            }

        s0=vals(r0); s1=vals(r1)
        ts=[(I(r["timestamp_ns"])-ew)/1e6 for r in seg]
        ss=[vals(r) for r in seg]
        post=[(t,s) for t,s in zip(ts,ss) if t>=0]
        if not post: continue

        peak_v=max(s["v"] for _,s in post)
        t_peak_v=max(post,key=lambda x:x[1]["v"])[0]
        mean_v=sum(s["v"] for _,s in post)/len(post)

        ba_delta=norm3(s1["bax"]-s0["bax"],s1["bay"]-s0["bay"],s1["baz"]-s0["baz"])
        bg_delta=norm3(s1["bgx"]-s0["bgx"],s1["bgy"]-s0["bgy"],s1["bgz"]-s0["bgz"])
        droll=s1["roll"]-s0["roll"]; dpitch=s1["pitch"]-s0["pitch"]; dyaw=s1["yaw"]-s0["yaw"]
        dtilt=math.sqrt(droll*droll+dpitch*dpitch)

        settle=math.sqrt(
            (F(r1["px_m"])-F(r0["px_m"]))**2+
            (F(r1["py_m"])-F(r0["py_m"]))**2+
            (F(r1["pz_m"])-F(r0["pz_m"]))**2
        )*1000.0

        rec={
            "run":run,"leg":leg,"direction":lr["direction"],
            "xy_mm":F(lr["horizontal_m"])*1000.0,
            "settle_mm":settle,
            "v0_mm_s":s0["v"],"v1_mm_s":s1["v"],"peak_v_mm_s":peak_v,"t_peak_v_ms":t_peak_v,"mean_post_v_mm_s":mean_v,
            "ba0_norm":s0["ba"],"ba1_norm":s1["ba"],"delta_ba_norm":ba_delta,
            "bg0_norm":s0["bg"],"bg1_norm":s1["bg"],"delta_bg_norm":bg_delta,
            "droll_deg":droll,"dpitch_deg":dpitch,"dyaw_deg":dyaw,"dtilt_deg":dtilt,
            "dvx_mm_s":s1["vx"]-s0["vx"],"dvy_mm_s":s1["vy"]-s0["vy"],"dvz_mm_s":s1["vz"]-s0["vz"],
            "dbax":s1["bax"]-s0["bax"],"dbay":s1["bay"]-s0["bay"],"dbaz":s1["baz"]-s0["baz"],
            "dbgx":s1["bgx"]-s0["bgx"],"dbgy":s1["bgy"]-s0["bgy"],"dbgz":s1["bgz"]-s0["bgz"],
        }
        rows.append(rec)
        by_key[(run,leg)]=(rec,[(t,s) for t,s in zip(ts,ss)])

print("================ V21 SETTLE INTERNAL STATE ================")
for r in rows:
    print(
        f"{r['run']} LEG {r['leg']} {r['direction']}: settle={r['settle_mm']:.2f}mm XY={r['xy_mm']:.2f}mm "
        f"V0={r['v0_mm_s']:.1f} peakV={r['peak_v_mm_s']:.1f}@{r['t_peak_v_ms']:.0f}ms V1={r['v1_mm_s']:.1f} "
        f"dBA={r['delta_ba_norm']:.5f} dBG={r['delta_bg_norm']:.6f} "
        f"dRPY=[{r['droll_deg']:+.3f},{r['dpitch_deg']:+.3f},{r['dyaw_deg']:+.3f}]"
    )

metrics=[
    "v0_mm_s","peak_v_mm_s","t_peak_v_ms","mean_post_v_mm_s",
    "ba0_norm","ba1_norm","delta_ba_norm",
    "bg0_norm","bg1_norm","delta_bg_norm",
    "droll_deg","dpitch_deg","dyaw_deg","dtilt_deg",
    "dvx_mm_s","dvy_mm_s","dvz_mm_s",
    "dbax","dbay","dbaz","dbgx","dbgy","dbgz"
]
print("\n================ CORRELATION WITH SETTLE ================")
ys=[r["settle_mm"] for r in rows]
for m in metrics:
    xs=[r[m] for r in rows]
    print(f"Pearson({m}, settle_mm)={pearson(xs,ys):+.4f}")

print("\n================ DIRECTION GROUPS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if rr:
        print(
            f"{d}: n={len(rr)} settleMean={statistics.mean(r['settle_mm'] for r in rr):.2f}mm "
            f"peakVMean={statistics.mean(r['peak_v_mm_s'] for r in rr):.1f}mm/s "
            f"dBAMean={statistics.mean(r['delta_ba_norm'] for r in rr):.5f} "
            f"dTiltMean={statistics.mean(r['dtilt_deg'] for r in rr):.3f}deg"
        )

# Contrast within the third run: LEG2 vs LEG6 if present.
third=[r for r in rows if r["run"].startswith("20260905_155540")]
if third:
    print("\n================ CONTRAST RUN3 LEG2 vs LEG6 ================")
    for leg in (2,6):
        r=next((x for x in third if x["leg"]==leg),None)
        if r:
            print(f"LEG{leg} {r['direction']}:")
            for k in ["settle_mm","xy_mm","v0_mm_s","peak_v_mm_s","t_peak_v_ms","v1_mm_s",
                      "ba0_norm","ba1_norm","delta_ba_norm","bg0_norm","bg1_norm","delta_bg_norm",
                      "droll_deg","dpitch_deg","dyaw_deg","dtilt_deg",
                      "dbax","dbay","dbaz","dvx_mm_s","dvy_mm_s","dvz_mm_s"]:
                print(f"  {k}={r[k]:+.6f}")

out=os.path.expanduser("~/jtzero_v21_settle_state_summary.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
