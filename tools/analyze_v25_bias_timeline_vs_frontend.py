#!/usr/bin/env python3
import csv, math, bisect
from pathlib import Path

H=Path("/home/vio")
BACK=H/"jtzero_500mm_v25_backend.csv"
FRONT=H/"jtzero_500mm_v25_frontend.csv"
LEGS=H/"jtzero_500mm_v25_legs.csv"
OUT=H/"jtzero_v25_bias_timeline_vs_frontend.csv"

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def norm(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))

def RzRyRx(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
            (-sp,cp*sr,cp*cr))
def mv(R,v): return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))

back=load(BACK); front=load(FRONT); legs=load(LEGS)
front_kf=[r for r in front if I(r,"is_keyframe")==1]
front_kf.sort(key=lambda r:I(r,"timestamp_ns"))
fts=[I(r,"timestamp_ns") for r in front_kf]
bykf={I(r,"keyframe"):r for r in back}

def nearest_front(ts):
    j=bisect.bisect_left(fts,ts); cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(front_kf): cand.append(front_kf[k])
    return min(cand,key=lambda r:abs(I(r,"timestamp_ns")-ts)) if cand else None

print("================ V25 BA TIMELINE vs FRONTEND ================")
rows=[]

for L in legs:
    leg=I(L,"leg"); ks=I(L,"start_settled_kf"); ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        print(f"LEG {leg}: missing backend states"); continue

    d=(F(e,"px_m")-F(s,"px_m"), F(e,"py_m")-F(s,"py_m"), 0.0)
    dn=norm(d)
    if dn<1e-9:
        print(f"LEG {leg}: zero horizontal displacement"); continue
    u=(d[0]/dn,d[1]/dn,0.0)

    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    vals=[]
    for r in seg:
        ba=(F(r,"bax"),F(r,"bay"),F(r,"baz"))
        R=RzRyRx(math.radians(F(r,"roll_deg")),math.radians(F(r,"pitch_deg")),math.radians(F(r,"yaw_deg")))
        ba_al=dot(mv(R,ba),u)
        fr=nearest_front(I(r,"timestamp_ns"))
        status=fr["mono_status"] if fr else "NO_FRONT"
        ratio=F(fr,"mono_inlier_ratio") if fr else float("nan")
        vals.append((r,ba_al,status,ratio))

    start_al=vals[0][1]
    end_al=vals[-1][1]
    total_delta=end_al-start_al
    print(f"\nLEG {leg} {L['direction']} KF {ks}->{ke} BA_along {start_al:+.5f}->{end_al:+.5f} d={total_delta:+.5f}")

    # Print 0/25/50/75/100% snapshots.
    for frac in (0.0,0.25,0.5,0.75,1.0):
        idx=round(frac*(len(vals)-1))
        r,ba_al,status,ratio=vals[idx]
        print(f"  {int(frac*100):3d}% KF={I(r,'keyframe'):3d} speed={F(r,'speed_m_s')*1000:6.1f}mm/s "
              f"BA_along={ba_al:+.5f} status={status:14s} inlier={ratio:.3f}")

    # Where does BA_along move most strongly?
    steps=[]
    by_status={}
    for a,b in zip(vals,vals[1:]):
        r0,v0,s0,q0=a; r1,v1,s1,q1=b
        dv=v1-v0
        steps.append((abs(dv),dv,I(r1,"keyframe"),s1,q1,F(r1,"speed_m_s")*1000))
        acc=by_status.setdefault(s1,{"n":0,"abs":0.0,"signed":0.0})
        acc["n"]+=1; acc["abs"]+=abs(dv); acc["signed"]+=dv

    print("  largest BA_along steps:")
    for ab,dv,kf,st,rat,spd in sorted(steps,reverse=True)[:5]:
        print(f"    KF={kf:3d} dBA_along={dv:+.5f} status={st:14s} inlier={rat:.3f} speed={spd:6.1f}mm/s")

    print("  contribution by frontend status:")
    for st,acc in sorted(by_status.items(), key=lambda kv:-kv[1]["abs"]):
        print(f"    {st:14s} n={acc['n']:3d} sum|dBA|={acc['abs']:.5f} signed={acc['signed']:+.5f}")

    # Save all samples.
    for r,ba_al,status,ratio in vals:
        rows.append(dict(
            leg=leg,direction=L["direction"],keyframe=I(r,"keyframe"),
            timestamp_ns=I(r,"timestamp_ns"),speed_m_s=F(r,"speed_m_s"),
            ba_along_m_s2=ba_al,status=status,mono_inlier_ratio=ratio,
            bax=F(r,"bax"),bay=F(r,"bay"),baz=F(r,"baz"),
            px_m=F(r,"px_m"),py_m=F(r,"py_m"),pz_m=F(r,"pz_m"),
        ))

if rows:
    with OUT.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

print("\nDECISION:")
print("- BA changes mainly during VALID motion -> visual-inertial coupling / scale observability is primary.")
print("- BA changes mainly during LOW_DISPARITY or near-zero speed -> stationary-factor/bias coupling regains priority.")
print("- A sharp change at motion onset/stop -> endpoint transition logic becomes primary.")
print("Saved:",OUT)
