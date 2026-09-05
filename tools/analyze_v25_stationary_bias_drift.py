#!/usr/bin/env python3
import csv, glob, math, os, sys, bisect, statistics

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def I(x): return int(float(x))
def F(x): return float(x)
def norm3(a,b,c): return math.sqrt(a*a+b*b+c*c)
def nearest(rows, ts, key):
    vals=[I(r[key]) for r in rows]
    j=bisect.bisect_left(vals,ts)
    cand=[]
    if j<len(rows): cand.append(rows[j])
    if j>0: cand.append(rows[j-1])
    return min(cand,key=lambda r:abs(I(r[key])-ts)) if cand else None

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v25_*")))
if not roots:
    raise SystemExit("No archived V25 runs found")

for root in roots:
    ep=os.path.join(root,"jtzero_500mm_v25_events.csv")
    bp=os.path.join(root,"jtzero_500mm_v25_backend.csv")
    ap=os.path.join(root,"jtzero_500mm_v25_attitude.csv")
    if not (os.path.exists(ep) and os.path.exists(bp)):
        continue
    events=load(ep); backend=load(bp)
    att=load(ap) if os.path.exists(ap) else []
    backend.sort(key=lambda r:I(r["callback_wall_ns"]))
    if att: att.sort(key=lambda r:I(r["recv_ns"]))

    print(f"================ {os.path.basename(root)} ================")

    ends=[e for e in events if e["event"]=="END"]
    starts=[e for e in events if e["event"]=="START"]

    for e in ends:
        leg=I(e["leg"])
        end_wall=I(e["event_wall_ns"])
        next_starts=[s for s in starts if I(s["event_wall_ns"])>end_wall]
        next_wall=I(next_starts[0]["event_wall_ns"]) if next_starts else I(backend[-1]["callback_wall_ns"])
        dur=(next_wall-end_wall)/1e9
        if dur<1.0:
            continue

        b0=nearest(backend,end_wall,"callback_wall_ns")
        if not b0:
            continue

        print(f"\nLEG {leg} stationary after END: available={dur:.2f}s "
              f"anchorKF={I(b0['keyframe'])}")

        max_sec=min(20,int(dur))
        rows=[]
        for sec in range(max_sec+1):
            ts=end_wall+sec*1_000_000_000
            b=nearest(backend,ts,"callback_wall_ns")
            if not b: continue
            ba=[F(b["bax"]),F(b["bay"]),F(b["baz"])]
            bg=[F(b["bgx"]),F(b["bgy"]),F(b["bgz"])]
            v=[F(b["vx_m_s"]),F(b["vy_m_s"]),F(b["vz_m_s"])]
            p=[F(b["px_m"]),F(b["py_m"]),F(b["pz_m"])]
            dba=[ba[i]-F(b0[k]) for i,k in enumerate(("bax","bay","baz"))]
            dp=[p[i]-F(b0[k]) for i,k in enumerate(("px_m","py_m","pz_m"))]
            row=dict(sec=sec,kf=I(b["keyframe"]),ba=ba,bg=bg,v=v,p=p,dba=dba,dp=dp)
            rows.append(row)
            print(
                f" t={sec:2d}s KF={row['kf']:3d} "
                f"BA=[{ba[0]:+.4f},{ba[1]:+.4f},{ba[2]:+.4f}] "
                f"dBA=[{dba[0]:+.4f},{dba[1]:+.4f},{dba[2]:+.4f}] "
                f"|dBA|={norm3(*dba):.4f} "
                f"V=[{v[0]:+.4f},{v[1]:+.4f},{v[2]:+.4f}] |V|={norm3(*v)*1000:.1f}mm/s "
                f"dP=[{dp[0]*1000:+.1f},{dp[1]*1000:+.1f},{dp[2]*1000:+.1f}]mm "
                f"|dP|={norm3(*dp)*1000:.1f}mm"
            )

        if len(rows)>=2:
            r=rows[-1]
            print(
                f" SUMMARY {r['sec']}s: |dBA|={norm3(*r['dba']):.4f} m/s^2, "
                f"|dP|={norm3(*r['dp'])*1000:.1f}mm, "
                f"|V|={norm3(*r['v'])*1000:.1f}mm/s"
            )
    print()
