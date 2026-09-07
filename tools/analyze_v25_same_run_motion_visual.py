#!/usr/bin/env python3
import csv, math, sys, statistics, bisect
from pathlib import Path

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def pct(xs,p):
    if not xs: return float("nan")
    s=sorted(xs)
    x=(len(s)-1)*p
    i=int(math.floor(x)); j=int(math.ceil(x))
    if i==j: return s[i]
    return s[i]*(j-x)+s[j]*(x-i)
def norm(v): return math.sqrt(sum(x*x for x in v))

if len(sys.argv) > 2:
    raise SystemExit("usage: analyze_v25_same_run_motion_visual.py [run_dir]")

root = Path(sys.argv[1]) if len(sys.argv)==2 else Path("/home/vio")
legs = load(root/"jtzero_500mm_v25_legs.csv")
back = load(root/"jtzero_500mm_v25_backend.csv")
front = load(root/"jtzero_500mm_v25_frontend.csv")
att = load(root/"jtzero_500mm_v25_attitude.csv")

bykf={I(r,"keyframe"):r for r in back}
fk=[r for r in front if I(r,"is_keyframe")==1]
fk.sort(key=lambda r:I(r,"timestamp_ns"))
att.sort(key=lambda r:I(r,"recv_ns"))

print("================ V25 SAME-RUN MOTION / VISUAL COMPARISON ================")
print("run:", root)

rows=[]
for L in legs:
    leg=I(L,"leg")
    ks=I(L,"start_settled_kf")
    ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        continue
    t0=I(s,"timestamp_ns"); t1=I(e,"timestamp_ns")
    cb0=I(s,"callback_wall_ns"); cb1=I(e,"callback_wall_ns")

    bs=[r for r in back if ks<=I(r,"keyframe")<=ke]
    fs=[r for r in fk if t0<=I(r,"timestamp_ns")<=t1]
    ats=[r for r in att if cb0<=I(r,"recv_ns")<=cb1]

    speeds=[F(r,"speed_m_s")*1000.0 for r in bs]
    # backend acceleration proxy from consecutive speed magnitudes
    accel=[]
    for a,b in zip(bs,bs[1:]):
        dt=(I(b,"timestamp_ns")-I(a,"timestamp_ns"))*1e-9
        if dt>1e-6:
            accel.append((F(b,"speed_m_s")-F(a,"speed_m_s"))/dt)

    valid=[r for r in fs if r["mono_status"]=="VALID"]
    low=[r for r in fs if r["mono_status"]=="LOW_DISPARITY"]
    few=[r for r in fs if r["mono_status"]=="FEW_MATCHES"]
    ratios=[F(r,"mono_inlier_ratio") for r in valid]
    tracks=[I(r,"tracked_features") for r in valid]
    put=[I(r,"mono_putatives") for r in valid]
    inl=[I(r,"mono_inliers") for r in valid]

    if ats:
        rolls=[F(r,"roll_deg") for r in ats]
        pitches=[F(r,"pitch_deg") for r in ats]
        yaws=[F(r,"yaw_deg") for r in ats]
        roll_span=max(rolls)-min(rolls)
        pitch_span=max(pitches)-min(pitches)
        yaw_span=max(yaws)-min(yaws)
    else:
        roll_span=pitch_span=yaw_span=float("nan")

    xy=F(L,"horizontal_m")*1000.0
    dz=F(L,"dz_m")*1000.0
    dur=(t1-t0)*1e-9

    r=dict(
        leg=leg,direction=L["direction"],scale=xy/500.0,xy_mm=xy,dz_mm=dz,duration_s=dur,
        speed_mean_mm_s=mean(speeds),speed_p50_mm_s=pct(speeds,0.5),
        speed_p90_mm_s=pct(speeds,0.9),speed_max_mm_s=max(speeds) if speeds else float("nan"),
        accel_abs_mean_m_s2=mean([abs(x) for x in accel]),
        accel_abs_p90_m_s2=pct([abs(x) for x in accel],0.9),
        frontend_kf=len(fs),valid=len(valid),low=len(low),few=len(few),
        valid_fraction=(len(valid)/len(fs) if fs else float("nan")),
        inlier_ratio_mean=mean(ratios),inlier_ratio_p10=pct(ratios,0.1),
        tracked_mean=mean(tracks),putatives_mean=mean(put),inliers_mean=mean(inl),
        fc_roll_span_deg=roll_span,fc_pitch_span_deg=pitch_span,fc_yaw_span_deg=yaw_span,
    )
    rows.append(r)

    print(f"\nLEG {leg} {L['direction']}: scale={r['scale']:.4f} XY={xy:.2f}mm dz={dz:+.1f}mm dur={dur:.2f}s")
    print(f"  speed mean/p50/p90/max = {r['speed_mean_mm_s']:.1f}/{r['speed_p50_mm_s']:.1f}/{r['speed_p90_mm_s']:.1f}/{r['speed_max_mm_s']:.1f} mm/s")
    print(f"  accel proxy |a| mean/p90 = {r['accel_abs_mean_m_s2']:.3f}/{r['accel_abs_p90_m_s2']:.3f} m/s^2")
    print(f"  frontend VALID/LOW/FEW = {len(valid)}/{len(low)}/{len(few)}  validFrac={r['valid_fraction']*100:.1f}%")
    print(f"  VALID inlier mean/p10 = {r['inlier_ratio_mean']:.3f}/{r['inlier_ratio_p10']:.3f}  trackedMean={r['tracked_mean']:.1f}")
    print(f"  FC attitude span R/P/Y = {roll_span:.3f}/{pitch_span:.3f}/{yaw_span:.3f} deg")

print("\n================ PAIRWISE REVERSAL COMPARISON ================")
pairs=[(1,2),(3,4)]
by_leg={r["leg"]:r for r in rows}
for a,b in pairs:
    if a not in by_leg or b not in by_leg: continue
    A=by_leg[a]; B=by_leg[b]
    print(f"PAIR {a}->{b}:")
    print(f"  scale delta B-A = {B['scale']-A['scale']:+.4f}")
    print(f"  duration delta = {B['duration_s']-A['duration_s']:+.2f}s")
    print(f"  mean speed delta = {B['speed_mean_mm_s']-A['speed_mean_mm_s']:+.1f}mm/s")
    print(f"  p90 speed delta = {B['speed_p90_mm_s']-A['speed_p90_mm_s']:+.1f}mm/s")
    print(f"  VALID fraction delta = {(B['valid_fraction']-A['valid_fraction'])*100:+.1f}pp")
    print(f"  inlier mean delta = {B['inlier_ratio_mean']-A['inlier_ratio_mean']:+.3f}")
    print(f"  pitch span delta = {B['fc_pitch_span_deg']-A['fc_pitch_span_deg']:+.3f}deg")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    def m(k): return mean([r[k] for r in rr])
    print(f"{d}: scale={m('scale'):.4f} dur={m('duration_s'):.2f}s meanSpd={m('speed_mean_mm_s'):.1f} "
          f"p90Spd={m('speed_p90_mm_s'):.1f} validFrac={m('valid_fraction')*100:.1f}% "
          f"inlierMean={m('inlier_ratio_mean'):.3f} pitchSpan={m('fc_pitch_span_deg'):.3f}deg")

print("\nINTERPRETATION:")
print("- If B->A scale excess repeats together with a repeatable motion-profile difference, motion excitation remains a serious confounder.")
print("- If motion/attitude profiles are similar but B->A still has larger scale, visual geometry/mono observability becomes stronger.")
print("- If visual quality is systematically worse on B->A while motion is similar, frontend geometry/outlier handling becomes the next target.")
print("- This script compares legs within ONE archived run; it does not mix datasets.")
