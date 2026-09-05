#!/usr/bin/env python3
import csv, glob, math, os, sys

THRESHOLDS_MS = [0, 50, 100, 150, 200, 300, 500, 1000]

def f(v): return float(v)
def i(v): return int(float(v))

def load(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))

def vec(r):
    return (f(r["px_m"]), f(r["py_m"]), f(r["pz_m"]))

def sub(a,b):
    return tuple(x-y for x,y in zip(a,b))

def norm(v):
    return math.sqrt(sum(x*x for x in v))

def dot(a,b):
    return sum(x*y for x,y in zip(a,b))

def pearson(xs,ys):
    if len(xs)<2: return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    vx=sum((x-mx)**2 for x in xs); vy=sum((y-my)**2 for y in ys)
    if vx<=0 or vy<=0: return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys))/math.sqrt(vx*vy)

roots = sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v21_*")))
if not roots:
    raise SystemExit("No V21 archives found")

trace_rows=[]
summary_rows=[]

for run_idx,root in enumerate(roots,1):
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
        press=bykf.get(press_kf); settled=bykf.get(settled_kf)
        if not press or not settled:
            print(f"LEG {leg}: missing backend KF press/settled")
            continue

        ew=i(e["event_wall_ns"])
        p0=vec(press); pfinal=vec(settled)
        final_vec=sub(pfinal,p0); final_norm=norm(final_vec)
        final_unit=tuple(x/final_norm for x in final_vec) if final_norm>1e-12 else (0.0,0.0,0.0)

        seg=sorted(
            [r for r in backend if press_kf <= i(r["keyframe"]) <= settled_kf],
            key=lambda r:i(r["keyframe"])
        )

        leg_trace=[]
        first_post_dt=float("nan")
        for r in seg:
            pv=vec(r)
            dv=sub(pv,p0)
            dt_source_ms=(i(r["timestamp_ns"])-ew)/1e6
            dt_callback_ms=(i(r["callback_wall_ns"])-ew)/1e6
            disp_norm_mm=norm(dv)*1000.0
            disp_proj_mm=dot(dv,final_unit)*1000.0 if final_norm>1e-12 else 0.0
            remain_mm=norm(sub(pfinal,pv))*1000.0
            if dt_source_ms>0 and math.isnan(first_post_dt):
                first_post_dt=dt_source_ms

            row={
                "run":os.path.basename(root),
                "leg":leg,
                "direction":lr["direction"],
                "keyframe":i(r["keyframe"]),
                "source_dt_from_END_ms":dt_source_ms,
                "callback_dt_from_END_ms":dt_callback_ms,
                "px_m":f(r["px_m"]),"py_m":f(r["py_m"]),"pz_m":f(r["pz_m"]),
                "vx_m_s":f(r["vx_m_s"]),"vy_m_s":f(r["vy_m_s"]),"vz_m_s":f(r["vz_m_s"]),
                "speed_m_s":f(r["speed_m_s"]),
                "roll_deg":f(r["roll_deg"]),"pitch_deg":f(r["pitch_deg"]),"yaw_deg":f(r["yaw_deg"]),
                "bax":f(r["bax"]),"bay":f(r["bay"]),"baz":f(r["baz"]),
                "bgx":f(r["bgx"]),"bgy":f(r["bgy"]),"bgz":f(r["bgz"]),
                "disp_from_END_mm":disp_norm_mm,
                "disp_projected_to_final_mm":disp_proj_mm,
                "remaining_to_settled_mm":remain_mm,
                "final_settle_vector_mm":final_norm*1000.0,
            }
            leg_trace.append(row); trace_rows.append(row)

        threshold_parts=[]
        for th in THRESHOLDS_MS:
            candidates=[r for r in leg_trace if r["source_dt_from_END_ms"] <= th]
            if candidates:
                rr=candidates[-1]
                threshold_parts.append(
                    f"+{th}ms:{rr['disp_from_END_mm']:.1f}mm/rem{rr['remaining_to_settled_mm']:.1f}"
                )
            else:
                threshold_parts.append(f"+{th}ms:n/a")

        event_source_age_ms=(ew-i(e["state_timestamp_ns"]))/1e6
        event_callback_age_ms=f(e["state_age_at_event_ms"])
        xy=f(lr["horizontal_m"])*1000.0
        total=final_norm*1000.0

        print(
            f"LEG {leg} {lr['direction']}: XY={xy:.2f}mm "
            f"ENDstate source_age={event_source_age_ms:.1f}ms callback_age={event_callback_age_ms:.1f}ms "
            f"first_post={first_post_dt:+.1f}ms final_settle3D={total:.2f}mm"
        )
        print("  TRACE " + " | ".join(threshold_parts))

        # Print every backend state for direct inspection.
        for rr in leg_trace:
            print(
                f"    KF={rr['keyframe']} t={rr['source_dt_from_END_ms']:+.1f}ms "
                f"cb={rr['callback_dt_from_END_ms']:+.1f}ms "
                f"dP={rr['disp_from_END_mm']:.2f}mm "
                f"proj={rr['disp_projected_to_final_mm']:.2f}mm "
                f"remain={rr['remaining_to_settled_mm']:.2f}mm "
                f"|V|={rr['speed_m_s']*1000.0:.2f}mm/s"
            )

        summary_rows.append({
            "run":os.path.basename(root),
            "leg":leg,
            "direction":lr["direction"],
            "xy_mm":xy,
            "final_settle3d_mm":total,
            "event_source_age_ms":event_source_age_ms,
            "event_callback_age_ms":event_callback_age_ms,
            "first_post_source_dt_ms":first_post_dt,
        })

out=os.path.expanduser("~/jtzero_v21_end_state_trace.csv")
if trace_rows:
    with open(out,"w",newline="") as fh:
        w=csv.DictWriter(fh,fieldnames=list(trace_rows[0].keys()))
        w.writeheader(); w.writerows(trace_rows)
    print(f"\nSaved per-state trace: {out}")

if summary_rows:
    print("\n================ ALL RUNS ================")
    for d in ("A->B","B->A"):
        rr=[r for r in summary_rows if r["direction"]==d]
        if rr:
            vals=[r["xy_mm"] for r in rr]
            mean=sum(vals)/len(vals)
            sd=math.sqrt(sum((x-mean)**2 for x in vals)/len(vals))
            print(f"{d}: n={len(vals)} meanXY={mean:.2f}mm sd={sd:.2f}mm")

    fp=[r["first_post_source_dt_ms"] for r in summary_rows if not math.isnan(r["first_post_source_dt_ms"])]
    fs=[r["final_settle3d_mm"] for r in summary_rows if not math.isnan(r["first_post_source_dt_ms"])]
    ca=[r["event_callback_age_ms"] for r in summary_rows]
    cs=[r["final_settle3d_mm"] for r in summary_rows]
    sa=[r["event_source_age_ms"] for r in summary_rows]
    ss=[r["final_settle3d_mm"] for r in summary_rows]

    print(f"Pearson(first_post_source_dt, final_settle3D)={pearson(fp,fs):.3f}")
    print(f"Pearson(callback_age_at_END, final_settle3D)={pearson(ca,cs):.3f}")
    print(f"Pearson(source_age_at_END, final_settle3D)={pearson(sa,ss):.3f}")
