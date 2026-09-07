#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

if len(sys.argv) < 2:
    raise SystemExit("usage: analyze_v25_frontend_quality_crossrun.py RUN_DIR [RUN_DIR ...]")

rows=[]
for arg in sys.argv[1:]:
    root=Path(arg)
    legs=load(root/"jtzero_500mm_v25_legs.csv")
    back=load(root/"jtzero_500mm_v25_backend.csv")
    front=load(root/"jtzero_500mm_v25_frontend.csv")
    bykf={I(r,"keyframe"):r for r in back}

    print("\n================ RUN ================")
    print(root)

    for L in legs:
        leg=I(L,"leg")
        ks=I(L,"start_settled_kf")
        ke=I(L,"end_press_kf")
        s=bykf.get(ks); e=bykf.get(ke)
        if not s or not e:
            continue
        t0=I(s,"timestamp_ns"); t1=I(e,"timestamp_ns")
        fs=[r for r in front if t0<=I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1]
        valid=[r for r in fs if r["mono_status"]=="VALID"]
        low=[r for r in fs if r["mono_status"]=="LOW_DISPARITY"]
        few=[r for r in fs if r["mono_status"]=="FEW_MATCHES"]
        ratios=[F(r,"mono_inlier_ratio") for r in valid]
        tracks=[I(r,"tracked_features") for r in valid]
        put=[I(r,"mono_putatives") for r in valid]
        inl=[I(r,"mono_inliers") for r in valid]
        weak=[r for r in valid if F(r,"mono_inlier_ratio")<0.40]
        strong=[r for r in valid if F(r,"mono_inlier_ratio")>=0.70]

        rec=dict(
            run=root.name,leg=leg,direction=L["direction"],
            scale=F(L,"scale_horizontal"),dz_mm=F(L,"dz_m")*1000.0,
            kf=len(fs),valid=len(valid),low=len(low),few=len(few),
            valid_frac=(len(valid)/len(fs) if fs else float("nan")),
            inlier_mean=mean(ratios),tracked_mean=mean(tracks),
            put_mean=mean(put),inliers_mean=mean(inl),
            weak_frac=(len(weak)/len(valid) if valid else float("nan")),
            strong_frac=(len(strong)/len(valid) if valid else float("nan")),
        )
        rows.append(rec)
        print(f"LEG {leg} {L['direction']}: scale={rec['scale']:.4f} dz={rec['dz_mm']:+.1f}mm "
              f"VALID={rec['valid_frac']*100:.1f}% inlier={rec['inlier_mean']:.3f} "
              f"tracked={rec['tracked_mean']:.1f} weak<0.40={rec['weak_frac']*100:.1f}% strong>=0.70={rec['strong_frac']*100:.1f}%")

print("\n================ DIRECTION MEANS ACROSS ALL RUNS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    print(f"{d}: n={len(rr)} scale={mean([r['scale'] for r in rr]):.4f} "
          f"dz={mean([r['dz_mm'] for r in rr]):+.1f}mm "
          f"VALID={mean([r['valid_frac'] for r in rr])*100:.1f}% "
          f"inlier={mean([r['inlier_mean'] for r in rr]):.3f} "
          f"tracked={mean([r['tracked_mean'] for r in rr]):.1f} "
          f"weak={mean([r['weak_frac'] for r in rr])*100:.1f}% "
          f"strong={mean([r['strong_frac'] for r in rr])*100:.1f}%")

print("\nINTERPRETATION:")
print("- Similar frontend quality in A->B and B->A weakens a simple 'camera tracking is worse in one direction' explanation.")
print("- Systematically worse VALID fraction/inlier ratio/tracks in B->A strengthens frontend-quality asymmetry.")
print("- This file does NOT contain visual translation/pose, so it cannot prove whether the wrong Z is created in frontend or backend.")
