#!/usr/bin/env python3
import csv, math, bisect, statistics
from pathlib import Path

H=Path("/home/vio")
BACK=H/"jtzero_500mm_v25_backend.csv"
FRONT=H/"jtzero_500mm_v25_frontend.csv"
LEGS=H/"jtzero_500mm_v25_legs.csv"
OUT=H/"jtzero_v25_bias_vs_visual_quality.csv"

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def norm(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v): return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

back=load(BACK); front=load(FRONT); legs=load(LEGS)
front=[r for r in front if I(r,"is_keyframe")==1]
front.sort(key=lambda r:I(r,"timestamp_ns"))
fts=[I(r,"timestamp_ns") for r in front]
bykf={I(r,"keyframe"):r for r in back}

def nearest_front(ts):
    j=bisect.bisect_left(fts,ts); cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(front): cand.append(front[k])
    return min(cand,key=lambda r:abs(I(r,"timestamp_ns")-ts)) if cand else None

def bin_ratio(q):
    if q < 0.20: return "<0.20"
    if q < 0.40: return "0.20-0.40"
    if q < 0.70: return "0.40-0.70"
    return ">=0.70"

rows=[]
print("================ V25 BA CHANGE vs VISUAL QUALITY ================")

for L in legs:
    leg=I(L,"leg"); ks=I(L,"start_settled_kf"); ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e: continue

    d=(F(e,"px_m")-F(s,"px_m"),F(e,"py_m")-F(s,"py_m"),0.0)
    dn=norm(d)
    if dn<1e-9: continue
    u=(d[0]/dn,d[1]/dn,0.0)

    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    vals=[]
    for r in seg:
        ba=(F(r,"bax"),F(r,"bay"),F(r,"baz"))
        R=RzRyRx(math.radians(F(r,"roll_deg")),math.radians(F(r,"pitch_deg")),math.radians(F(r,"yaw_deg")))
        ba_al=dot(mv(R,ba),u)
        fr=nearest_front(I(r,"timestamp_ns"))
        vals.append((r,ba_al,fr))

    print(f"\nLEG {leg} {L['direction']}")
    bins={}
    all_steps=[]
    for a,b in zip(vals,vals[1:]):
        r0,v0,f0=a; r1,v1,f1=b
        if not f1: continue
        dv=v1-v0
        ratio=F(f1,"mono_inlier_ratio")
        status=f1["mono_status"]
        put=I(f1,"mono_putatives")
        inl=I(f1,"mono_inliers")
        tracked=I(f1,"tracked_features")
        detected=I(f1,"detected_features")
        spd=F(r1,"speed_m_s")*1000.0
        key=(status,bin_ratio(ratio))
        acc=bins.setdefault(key,{"n":0,"abs":0.0,"signed":0.0,"rat":[],"spd":[],"put":[],"inl":[]})
        acc["n"]+=1; acc["abs"]+=abs(dv); acc["signed"]+=dv
        acc["rat"].append(ratio); acc["spd"].append(spd); acc["put"].append(put); acc["inl"].append(inl)
        all_steps.append((abs(dv),dv,I(r1,"keyframe"),status,ratio,put,inl,tracked,detected,spd))
        rows.append(dict(
            leg=leg,direction=L["direction"],keyframe=I(r1,"keyframe"),
            status=status,inlier_ratio=ratio,mono_putatives=put,mono_inliers=inl,
            tracked_features=tracked,detected_features=detected,speed_mm_s=spd,
            delta_ba_along_m_s2=dv,abs_delta_ba_along_m_s2=abs(dv)
        ))

    for key,acc in sorted(bins.items(), key=lambda kv:-kv[1]["abs"]):
        st,bn=key
        print(f"  {st:14s} ratio {bn:9s}: n={acc['n']:2d} "
              f"sum|dBA|={acc['abs']:.5f} signed={acc['signed']:+.5f} "
              f"meanRatio={mean(acc['rat']):.3f} meanSpeed={mean(acc['spd']):.1f}mm/s")

    weak=[x for x in all_steps if x[3]=="VALID" and x[4]<0.40]
    strong=[x for x in all_steps if x[3]=="VALID" and x[4]>=0.70]
    print(f"  VALID weak (<0.40): n={len(weak)} sum|dBA|={sum(x[0] for x in weak):.5f} signed={sum(x[1] for x in weak):+.5f}")
    print(f"  VALID strong(>=0.70): n={len(strong)} sum|dBA|={sum(x[0] for x in strong):.5f} signed={sum(x[1] for x in strong):+.5f}")

print("\n================ GLOBAL VALID QUALITY BINS ================")
for label,cond in [
    ("weak <0.20", lambda r:r["status"]=="VALID" and r["inlier_ratio"]<0.20),
    ("0.20-0.40", lambda r:r["status"]=="VALID" and 0.20<=r["inlier_ratio"]<0.40),
    ("0.40-0.70", lambda r:r["status"]=="VALID" and 0.40<=r["inlier_ratio"]<0.70),
    ("strong >=0.70", lambda r:r["status"]=="VALID" and r["inlier_ratio"]>=0.70),
]:
    z=[r for r in rows if cond(r)]
    if z:
        print(f"{label:14s}: n={len(z):3d} sum|dBA|={sum(r['abs_delta_ba_along_m_s2'] for r in z):.5f} "
              f"mean|dBA|={mean([r['abs_delta_ba_along_m_s2'] for r in z]):.5f} "
              f"signed={sum(r['delta_ba_along_m_s2'] for r in z):+.5f} "
              f"meanSpeed={mean([r['speed_mm_s'] for r in z]):.1f}mm/s")

if rows:
    with OUT.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

print("\nDECISION:")
print("- BA motion concentrated in low-inlier VALID frames -> weak visual geometry/outlier handling is the leading trigger.")
print("- BA motion similar in strong and weak VALID frames -> issue is broader VIO scale/bias observability, not only poor feature quality.")
print("- Directional signed BA change appears only in one quality bin -> that bin becomes the next causal target.")
print("Saved:",OUT)
