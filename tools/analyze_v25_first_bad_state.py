#!/usr/bin/env python3
import csv, glob, math, os, sys, bisect

def load(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))
def I(x): return int(float(x))
def F(x): return float(x)
def n3(x,y,z): return math.sqrt(x*x+y*y+z*z)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v25_*")))
if not roots:
    raise SystemExit("No archived V25 runs found")

for root in roots:
    ep=os.path.join(root,"jtzero_500mm_v25_events.csv")
    bp=os.path.join(root,"jtzero_500mm_v25_backend.csv")
    fp=os.path.join(root,"jtzero_500mm_v25_frontend.csv")
    if not all(os.path.exists(p) for p in (ep,bp,fp)):
        continue

    events=load(ep); backend=load(bp); frontend=load(fp)
    backend.sort(key=lambda r:I(r["timestamp_ns"]))
    frontend.sort(key=lambda r:I(r["timestamp_ns"]))
    fts=[I(r["timestamp_ns"]) for r in frontend]

    def nearest_front(ts):
        j=bisect.bisect_left(fts,ts)
        c=[]
        if j<len(frontend): c.append(frontend[j])
        if j>0: c.append(frontend[j-1])
        return min(c,key=lambda r:abs(I(r["timestamp_ns"])-ts)) if c else None

    starts={I(e["leg"]):e for e in events if e["event"]=="START"}
    ends={I(e["leg"]):e for e in events if e["event"]=="END"}

    print(f"================ {os.path.basename(root)} ================")
    for leg in sorted(set(starts)&set(ends)):
        sw=I(starts[leg]["event_wall_ns"]); ew=I(ends[leg]["event_wall_ns"])
        seg=[r for r in backend if sw<=I(r["callback_wall_ns"])<=ew]
        if len(seg)<2: continue

        # Use source timestamps for frontend/backend ordering; wall time only defines physical leg.
        rows=[]
        b0=seg[0]
        for idx,b in enumerate(seg):
            prev=seg[idx-1] if idx else b
            dv=n3(F(b["vx_m_s"])-F(prev["vx_m_s"]),
                   F(b["vy_m_s"])-F(prev["vy_m_s"]),
                   F(b["vz_m_s"])-F(prev["vz_m_s"]))
            dba=n3(F(b["bax"])-F(prev["bax"]),
                    F(b["bay"])-F(prev["bay"]),
                    F(b["baz"])-F(prev["baz"]))
            dp=n3(F(b["px_m"])-F(prev["px_m"]),
                   F(b["py_m"])-F(prev["py_m"]),
                   F(b["pz_m"])-F(prev["pz_m"]))
            dba0=n3(F(b["bax"])-F(b0["bax"]),
                     F(b["bay"])-F(b0["bay"]),
                     F(b["baz"])-F(b0["baz"]))
            v=n3(F(b["vx_m_s"]),F(b["vy_m_s"]),F(b["vz_m_s"]))
            fr=nearest_front(I(b["timestamp_ns"]))
            ratio=F(fr["mono_inlier_ratio"]) if fr else float("nan")
            rows.append(dict(b=b,fr=fr,dv=dv,dba=dba,dp=dp,dba0=dba0,v=v,ratio=ratio))

        # Data-driven event candidates. These are diagnostic rankings, not BAD/GOOD truth labels.
        dv_vals=sorted(((r["dv"],i) for i,r in enumerate(rows[1:])), reverse=True)
        dba_vals=sorted(((r["dba"],i) for i,r in enumerate(rows[1:])), reverse=True)
        ratio_vals=sorted(((r["ratio"],i) for i,r in enumerate(rows) if math.isfinite(r["ratio"])))
        status_changes=[]
        for i in range(1,len(rows)):
            a=rows[i-1]["fr"]; b=rows[i]["fr"]
            if a and b and a["mono_status"]!=b["mono_status"]:
                status_changes.append(i)

        candidates=set()
        for _,i in dv_vals[:2]: candidates.add(i+1)
        for _,i in dba_vals[:2]: candidates.add(i+1)
        for _,i in ratio_vals[:2]: candidates.add(i)
        candidates.update(status_changes)

        print(f"\nLEG {leg}: backend states={len(rows)}")
        if dv_vals:
            x,i=dv_vals[0]; print(f"  largest dV : KF={I(rows[i+1]['b']['keyframe'])} {x*1000:.1f}mm/s")
        if dba_vals:
            x,i=dba_vals[0]; print(f"  largest dBA: KF={I(rows[i+1]['b']['keyframe'])} {x:.5f}m/s2")
        if ratio_vals:
            x,i=ratio_vals[0]; print(f"  lowest ratio: KF={I(rows[i]['b']['keyframe'])} ratio={x:.3f}")
        if status_changes:
            print("  status changes:",", ".join(
                f"KF{I(rows[i]['b']['keyframe'])}:{rows[i-1]['fr']['mono_status']}->{rows[i]['fr']['mono_status']}"
                for i in status_changes))

        # Merge overlapping +/-3 KF windows.
        expanded=set()
        for i in candidates:
            expanded.update(range(max(0,i-3),min(len(rows),i+4)))
        print("  ---- diagnostic windows ----")
        last=-2
        for i in sorted(expanded):
            if i>last+1: print("  ...")
            r=rows[i]; b=r["b"]; fr=r["fr"]
            if fr:
                ftxt=(f"frame={I(fr['frame_id'])} trk={I(fr['tracked_features'])} "
                      f"inl={I(fr['mono_inliers'])}/{I(fr['mono_putatives'])} "
                      f"ratio={F(fr['mono_inlier_ratio']):.3f} status={fr['mono_status']} "
                      f"dtSrc={(I(fr['timestamp_ns'])-I(b['timestamp_ns']))/1e6:+.1f}ms")
            else:
                ftxt="front=NONE"
            tstart=(I(b["callback_wall_ns"])-sw)/1e9
            print(f"  KF={I(b['keyframe']):3d} t={tstart:+6.2f}s "
                  f"dP={r['dp']*1000:6.1f}mm |V|={r['v']*1000:6.1f} "
                  f"dV={r['dv']*1000:6.1f}mm/s dBA={r['dba']:.5f} "
                  f"dBA0={r['dba0']:.5f} "
                  f"RPY=[{F(b['roll_deg']):+.2f},{F(b['pitch_deg']):+.2f},{F(b['yaw_deg']):+.2f}] | {ftxt}")
            last=i
    print()
