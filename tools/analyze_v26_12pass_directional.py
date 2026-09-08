#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics

SRC=Path("/home/vio/jtzero_v26_12pass_rootcause.csv")
OUT=Path("/home/vio/jtzero_v26_12pass_directional.csv")

def f(x):
    try:return float(x)
    except:return float("nan")
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def sd(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.pstdev(v) if len(v)>1 else 0.0
def corr(x,y):
    p=[(a,b) for a,b in zip(x,y) if math.isfinite(a) and math.isfinite(b)]
    if len(p)<3:return float("nan")
    x=[a for a,b in p]; y=[b for a,b in p]; mx=mean(x); my=mean(y)
    sx=sum((a-mx)**2 for a in x); sy=sum((b-my)**2 for b in y)
    return sum((a-mx)*(b-my) for a,b in p)/math.sqrt(sx*sy) if sx>0 and sy>0 else float("nan")

if not SRC.exists():
    raise SystemExit("Run tools/analyze_v26_12pass_rootcause.py first")

with SRC.open() as fh: rows=list(csv.DictReader(fh))
for r in rows:
    leg=int(r["leg"])
    # Physical sequence is alternating: odd passes go one way, even passes return.
    r["physical_direction"]="A_TO_B" if leg%2 else "B_TO_A"
    r["direction_sign"]="1" if leg%2 else "-1"
    r["abs_error_mm"]=str(abs(f(r["error_mm"])))
    r["signed_along_mm"]=str(f(r["horizontal_mm"])*(1 if leg%2 else -1))

modes=["FAST_5S","MEDIUM_7P5S","SLOW_10S"]
dirs=["A_TO_B","B_TO_A"]

print("\n"+"="*112)
print("V26 DIRECTION-AWARE 12-PASS ANALYSIS")
print("="*112)
print("MODE          DIR      LEGS       MEAN_mm   ERR_mm    STD_mm    DUR_s   SPEED")
for mode in modes:
    for d in dirs:
        rr=[r for r in rows if r["mode"]==mode and r["physical_direction"]==d]
        print(f"{mode:<13} {d:<8} {','.join(r['leg'] for r in rr):<9}"
              f" {mean([f(r['horizontal_mm']) for r in rr]):9.2f}"
              f" {mean([f(r['error_mm']) for r in rr]):+8.2f}"
              f" {sd([f(r['horizontal_mm']) for r in rr]):9.2f}"
              f" {mean([f(r['duration_s']) for r in rr]):8.3f}"
              f" {mean([f(r['mean_speed']) for r in rr]):7.4f}")

print("\n"+"="*112)
print("SPEED TREND WITHIN SAME PHYSICAL DIRECTION")
print("="*112)
for d in dirs:
    print(f"\n{d}")
    pts=[]
    for mode in modes:
        rr=[r for r in rows if r["mode"]==mode and r["physical_direction"]==d]
        dur=mean([f(r["duration_s"]) for r in rr]); sp=mean([f(r["mean_speed"]) for r in rr])
        er=mean([f(r["error_mm"]) for r in rr])
        pts.append((sp,er))
        print(f"  {mode:<13} duration={dur:6.3f}s speed={sp:.5f}m/s mean_error={er:+7.2f}mm")
    print(f"  corr(speed,error) across 3 speed groups = {corr([p[0] for p in pts],[p[1] for p in pts]):+.3f}")

# Direction-confounded features: compare odd/even separation and within-direction error correlations.
exclude={"mode","leg","horizontal_mm","error_mm","physical_direction","direction_sign","abs_error_mm","signed_along_mm"}
numeric=[k for k in rows[0] if k not in exclude]
print("\n"+"="*112)
print("DIRECTION CONFOUND SCREEN")
print("Features are ranked by odd/even mean separation normalized by pooled spread.")
print("="*112)
rank=[]
for k in numeric:
    a=[f(r[k]) for r in rows if r["physical_direction"]=="A_TO_B"]
    b=[f(r[k]) for r in rows if r["physical_direction"]=="B_TO_A"]
    if not all(math.isfinite(x) for x in a+b): continue
    pooled=math.sqrt((sd(a)**2+sd(b)**2)/2)
    effect=abs(mean(a)-mean(b))/pooled if pooled>0 else 0
    ca=corr(a,[f(r["error_mm"]) for r in rows if r["physical_direction"]=="A_TO_B"])
    cb=corr(b,[f(r["error_mm"]) for r in rows if r["physical_direction"]=="B_TO_A"])
    rank.append((effect,k,mean(a),mean(b),ca,cb))
print("FEATURE                              DIR_EFFECT   Amean        Bmean       corrA     corrB")
for effect,k,ma,mb,ca,cb in sorted(rank,reverse=True)[:25]:
    print(f"{k:<36} {effect:10.3f} {ma:12.5g} {mb:12.5g} {ca:+9.3f} {cb:+9.3f}")

print("\n"+"="*112)
print("WITHIN-DIRECTION CORRELATION WITH SIGNED ERROR")
print("="*112)
cand=[]
for k in numeric:
    ca=corr([f(r[k]) for r in rows if r["physical_direction"]=="A_TO_B"],
            [f(r["error_mm"]) for r in rows if r["physical_direction"]=="A_TO_B"])
    cb=corr([f(r[k]) for r in rows if r["physical_direction"]=="B_TO_A"],
            [f(r["error_mm"]) for r in rows if r["physical_direction"]=="B_TO_A"])
    if math.isfinite(ca) and math.isfinite(cb):
        # Stronger candidate if same sign in both directions; opposite signs are suspicious/confounded.
        same=(ca*cb)>0
        score=min(abs(ca),abs(cb)) if same else 0
        cand.append((score,same,k,ca,cb))
print("FEATURE                              same_sign   corr_A    corr_B   conservative_score")
for score,same,k,ca,cb in sorted(cand,reverse=True)[:30]:
    print(f"{k:<36} {str(same):>9} {ca:+9.3f} {cb:+9.3f} {score:18.3f}")

# Pair each A->B with adjacent B->A at same speed to expose directional asymmetry.
print("\n"+"="*112)
print("ADJACENT DIRECTION PAIRS")
print("="*112)
print("MODE          PAIR   Aerr_mm   Berr_mm   mean_err   asym(A-B)")
for mode in modes:
    rr=sorted([r for r in rows if r["mode"]==mode],key=lambda r:int(r["leg"]))
    for i in (0,2):
        a,b=rr[i],rr[i+1]
        ea,eb=f(a["error_mm"]),f(b["error_mm"])
        print(f"{mode:<13} {a['leg']}-{b['leg']:<3} {ea:+9.2f} {eb:+9.2f} {(ea+eb)/2:+10.2f} {ea-eb:+10.2f}")

with OUT.open("w",newline="") as fh:
    w=csv.DictWriter(fh,fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
print(f"\nCSV: {OUT}")
