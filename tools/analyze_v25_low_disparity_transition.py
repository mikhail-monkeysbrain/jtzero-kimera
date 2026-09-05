#!/usr/bin/env python3
import csv, glob, math, os, sys, bisect

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def I(x): return int(float(x))
def F(x): return float(x)
def n3(x,y,z): return math.sqrt(x*x+y*y+z*z)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v25_*")))
if not roots: raise SystemExit("No V25 archives")

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v25_backend.csv")
    fp=os.path.join(root,"jtzero_500mm_v25_frontend.csv")
    ep=os.path.join(root,"jtzero_500mm_v25_events.csv")
    if not all(os.path.exists(p) for p in (bp,fp,ep)): continue
    b=load(bp); f=load(fp); e=load(ep)
    byts={I(r["timestamp_ns"]):r for r in b}
    fkeys=[r for r in f if I(r["is_keyframe"])==1]
    fkeys.sort(key=lambda r:I(r["timestamp_ns"]))
    b.sort(key=lambda r:I(r["timestamp_ns"]))

    def near_backend(ts):
        return min(b,key=lambda r:abs(I(r["timestamp_ns"])-ts))

    print(f"================ {os.path.basename(root)} ================")
    for end in [x for x in e if x["event"]=="END"]:
        leg=I(end["leg"]); ets=I(end["state_timestamp_ns"])
        # keyframes from ~2 s before to ~5 s after END
        win=[r for r in fkeys if ets-2_000_000_000 <= I(r["timestamp_ns"]) <= ets+5_000_000_000]
        first_low=None
        for r in win:
            if I(r["timestamp_ns"])>=ets and r["mono_status"]=="LOW_DISPARITY":
                first_low=r; break
        if not first_low:
            print(f"LEG {leg}: no post-END LOW_DISPARITY in window")
            continue
        idx=fkeys.index(first_low)
        lo=max(0,idx-4); hi=min(len(fkeys),idx+7)
        seq=fkeys[lo:hi]
        anchor=near_backend(ets)
        print(f"\nLEG {leg}: END KF~{I(anchor['keyframe'])}, first LOW frame={I(first_low['frame_id'])} "
              f"dt={(I(first_low['timestamp_ns'])-ets)/1e6:+.1f}ms")
        for fr in seq:
            br=near_backend(I(fr["timestamp_ns"]))
            dba=[F(br[k])-F(anchor[k]) for k in ("bax","bay","baz")]
            dp=[(F(br[k])-F(anchor[k]))*1000 for k in ("px_m","py_m","pz_m")]
            v=[F(br[k]) for k in ("vx_m_s","vy_m_s","vz_m_s")]
            marker="<<< FIRST LOW" if fr is first_low else ""
            print(
              f" KF={I(br['keyframe']):3d} dt={(I(br['timestamp_ns'])-ets)/1e3/1e3:+7.1f}ms "
              f"{fr['mono_status']:13s} inl={I(fr['mono_inliers']):3d}/{I(fr['mono_putatives']):3d} "
              f"r={F(fr['mono_inlier_ratio']):.3f} "
              f"|V|={n3(*v)*1000:6.1f}mm/s "
              f"dBA=[{dba[0]:+.4f},{dba[1]:+.4f},{dba[2]:+.4f}] |dBA|={n3(*dba):.4f} "
              f"dP=[{dp[0]:+.1f},{dp[1]:+.1f},{dp[2]:+.1f}]mm {marker}"
            )
    print()
