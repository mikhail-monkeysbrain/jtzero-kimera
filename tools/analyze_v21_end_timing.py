#!/usr/bin/env python3
import csv, glob, math, os, sys

GUARD_NS = 20_000_000  # ±20 ms ambiguity around physical END

def f(v): return float(v)
def i(v): return int(float(v))

def dist(a,b):
    return math.sqrt(sum((f(a[k])-f(b[k]))**2 for k in ("px_m","py_m","pz_m")))

def load(path):
    with open(path,newline="") as fh:
        return list(csv.DictReader(fh))

roots = sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v21_*")))
if not roots:
    raise SystemExit("No V21 archives found")

all_rows=[]
for root in roots:
    evp=os.path.join(root,"jtzero_500mm_v21_events.csv")
    bep=os.path.join(root,"jtzero_500mm_v21_backend.csv")
    lep=os.path.join(root,"jtzero_500mm_v18_legs.csv")
    if not all(os.path.exists(x) for x in (evp,bep,lep)):
        print(f"SKIP {root}: missing events/backend/legs")
        continue

    events=load(evp); backend=load(bep); legs=load(lep)
    ends={i(e["leg"]):e for e in events if e["event"]=="END"}
    bykf={i(r["keyframe"]):r for r in backend}

    first_dir=legs[0]["direction"] if legs else "?"
    print(f"\n================ {os.path.basename(root)} first={first_dir} ================")
    for lr in legs:
        leg=i(lr["leg"]); e=ends.get(leg)
        if not e: continue
        press_kf=i(lr["end_press_kf"]); settled_kf=i(lr["end_settled_kf"])
        press=bykf.get(press_kf)
        settled=bykf.get(settled_kf)
        if not press or not settled:
            print(f"LEG {leg}: missing backend KF press/settled")
            continue

        ew=i(e["event_wall_ns"])
        seg=[r for r in backend if press_kf <= i(r["keyframe"]) <= settled_kf]
        definite_pre=[r for r in seg if i(r["timestamp_ns"]) < ew-GUARD_NS]
        ambiguous=[r for r in seg if abs(i(r["timestamp_ns"])-ew) <= GUARD_NS]
        definite_post=[r for r in seg if i(r["timestamp_ns"]) > ew+GUARD_NS]

        last_pre=definite_pre[-1] if definite_pre else press
        first_post=definite_post[0] if definite_post else None

        total=dist(press,settled)*1000
        pre_tail=dist(press,last_pre)*1000
        post_from_last_pre=dist(last_pre,settled)*1000
        event_state_age=f(e["state_age_at_event_ms"])
        source_age=(ew-i(e["state_timestamp_ns"]))/1e6
        cb_latency=(i(e["state_callback_wall_ns"])-i(e["state_timestamp_ns"]))/1e6

        if first_post:
            first_post_source_dt=(i(first_post["timestamp_ns"])-ew)/1e6
            first_post_cb_dt=(i(first_post["callback_wall_ns"])-ew)/1e6
        else:
            first_post_source_dt=float("nan"); first_post_cb_dt=float("nan")

        direction=lr["direction"]
        xy=f(lr["horizontal_m"])*1000
        print(
            f"LEG {leg} {direction}: XY={xy:.2f}mm total_settle3D={total:.2f}mm "
            f"END_state source_age={source_age:.1f}ms callback_age={event_state_age:.1f}ms "
            f"state->callback={cb_latency:.1f}ms | "
            f"states pre/amb/post={len(definite_pre)}/{len(ambiguous)}/{len(definite_post)} "
            f"pre_tail={pre_tail:.2f}mm post_from_last_pre={post_from_last_pre:.2f}mm "
            f"first_post source={first_post_source_dt:+.1f}ms callback={first_post_cb_dt:+.1f}ms"
        )
        all_rows.append((direction,xy,total,event_state_age,source_age,pre_tail,post_from_last_pre))

if all_rows:
    print("\n================ ALL RUNS ================")
    for d in ("A->B","B->A"):
        rr=[r for r in all_rows if r[0]==d]
        if rr:
            vals=[r[1] for r in rr]
            mean=sum(vals)/len(vals)
            sd=math.sqrt(sum((x-mean)**2 for x in vals)/len(vals))
            print(f"{d}: n={len(vals)} meanXY={mean:.2f}mm sd={sd:.2f}mm")
    ages=[r[3] for r in all_rows]; settles=[r[2] for r in all_rows]
    ma=sum(ages)/len(ages); ms=sum(settles)/len(settles)
    cov=sum((a-ma)*(s-ms) for a,s in zip(ages,settles))
    va=sum((a-ma)**2 for a in ages); vs=sum((s-ms)**2 for s in settles)
    corr=cov/math.sqrt(va*vs) if va and vs else float("nan")
    print(f"Pearson(callback_age_at_END, total_settle3D)={corr:.3f}")
