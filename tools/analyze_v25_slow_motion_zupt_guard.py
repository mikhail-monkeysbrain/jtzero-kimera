#!/usr/bin/env python3
import csv, os, math, statistics, sys

FRONT=os.path.expanduser("~/jtzero_500mm_v25_frontend.csv")
BACK=os.path.expanduser("~/jtzero_500mm_v25_backend.csv")
LEGS=os.path.expanduser("~/jtzero_500mm_v25_legs.csv")

for p in (FRONT,BACK,LEGS):
    if not os.path.exists(p):
        print("MISSING:", p); sys.exit(2)

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(r[k])

front=read(FRONT); back=read(BACK); legs=read(LEGS)
bykf={I(r,"keyframe"):r for r in back}

print("================ V25 SLOW-MOTION / ZUPT GUARD ================")
print("Purpose: verify that strong LOW_DISPARITY stationary constraints do not freeze real slow motion.")

for l in legs:
    ks=I(l,"start_settled_kf"); ke=I(l,"end_settled_kf")
    print(f"\nLEG {l['leg']} {l['direction']} KF {ks}->{ke}")
    br=[r for r in back if ks <= I(r,"keyframe") <= ke]
    if not br:
        print("  backend rows missing"); continue

    # Match frontend rows by nearest keyframe timestamp; only keyframes.
    fks=[r for r in front if I(r,"is_keyframe")==1]
    ts0=I(br[0],"timestamp_ns"); ts1=I(br[-1],"timestamp_ns")
    fr=[r for r in fks if ts0 <= I(r,"timestamp_ns") <= ts1]

    statuses={}
    ratios=[]
    for r in fr:
        s=r["mono_status"]
        statuses[s]=statuses.get(s,0)+1
        ratios.append(F(r,"mono_inlier_ratio"))

    dur=(I(br[-1],"timestamp_ns")-I(br[0],"timestamp_ns"))*1e-9
    dx=F(l,"dx_m"); dy=F(l,"dy_m"); dz=F(l,"dz_m")
    horiz=F(l,"horizontal_m")
    speed_nom=horiz/dur if dur>0 else 0.0

    print(f"  duration={dur:.2f}s measured_horizontal={horiz*1000:.2f}mm nominal_avg={speed_nom*1000:.2f}mm/s")
    print(f"  delta=[{dx*1000:+.1f},{dy*1000:+.1f},{dz*1000:+.1f}]mm")
    print(f"  frontend keyframes={len(fr)} statuses={statuses}")
    if ratios:
        print(f"  inlier_ratio mean={statistics.mean(ratios):.3f} min={min(ratios):.3f}")

    low=statuses.get("LOW_DISPARITY",0)
    total=max(1,len(fr))
    low_frac=low/total
    print(f"  LOW_DISPARITY fraction={low_frac*100:.1f}%")

    # Detect suspicious plateaus in STATE pose while leg is active.
    plateaus=[]
    run_start=None
    prev=None
    for r in br:
        p=(F(r,"px_m"),F(r,"py_m"),F(r,"pz_m"))
        if prev is not None:
            dp=math.sqrt(sum((a-b)**2 for a,b in zip(p,prev[1])))
            if dp < 0.001:  # <1 mm per backend KF
                if run_start is None: run_start=prev[0]
            else:
                if run_start is not None and I(r,"keyframe")-run_start >= 4:
                    plateaus.append((run_start,I(r,"keyframe")-1))
                run_start=None
        prev=(I(r,"keyframe"),p)
    if run_start is not None and prev[0]-run_start>=4:
        plateaus.append((run_start,prev[0]))

    print(f"  state plateaus (<1mm/KF for >=4 KF): {plateaus if plateaus else 'none'}")

    verdict=[]
    if horiz < 0.40:
        verdict.append("FAIL_DISTANCE")
    if low_frac > 0.50 and horiz < 0.45:
        verdict.append("LIKELY_ZUPT_FREEZE")
    if plateaus and horiz < 0.45:
        verdict.append("POSE_PLATEAU")
    if not verdict:
        verdict=["PASS_GUARD"]

    print("  VERDICT:", ",".join(verdict))

print("\nInterpretation:")
print("PASS_GUARD means the strong stationary constraints did not obviously suppress the completed real motion.")
print("LIKELY_ZUPT_FREEZE/POSE_PLATEAU means LOW_DISPARITY is activating during real slow motion strongly enough to corrupt distance.")
