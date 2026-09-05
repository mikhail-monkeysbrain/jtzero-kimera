#!/usr/bin/env python3
import csv, os, math, statistics

CHAIN=os.path.expanduser("~/jtzero_kimera_chain.csv")
EVENTS=os.path.expanduser("~/jtzero_500mm_v25_events.csv")

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(r[k])
def vec(r,pfx): return (F(r,pfx+"_vx"),F(r,pfx+"_vy"),F(r,pfx+"_vz"))
def pos(r,pfx): return (F(r,pfx+"_px"),F(r,pfx+"_py"),F(r,pfx+"_pz"))
def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def unit(v):
    n=norm(v)
    return tuple(x/n for x in v) if n>1e-12 else (0.0,0.0,0.0)

chain=read(CHAIN); events=read(EVENTS)
bykf={I(r,"keyframe"):r for r in chain}
maxkf=max(bykf)

print("================ V25 RESIDUAL VELOCITY AFTER END ================")

for e in [x for x in events if x["event"]=="END"]:
    leg=int(e["leg"]); k0=I(e,"keyframe")
    r0=bykf.get(k0)
    if not r0: continue

    p0=pos(r0,"state")
    # Estimate incoming movement direction from 10 keyframes before END.
    kb=max(min(bykf),k0-10)
    rb=bykf.get(kb)
    move_dir=unit(sub(p0,pos(rb,"state"))) if rb else (0.0,0.0,0.0)

    print(f"\nLEG {leg} END KF={k0} move_dir=[{move_dir[0]:+.3f},{move_dir[1]:+.3f},{move_dir[2]:+.3f}]")
    print(" KF   dt[s]   stateV[mm/s]                 |V|   alongV  crossV   dPfromEND[mm]")

    rows=[]
    for k in range(k0, min(maxkf,k0+100)+1):
        r=bykf.get(k)
        if not r: continue
        dt=(F(r,"timestamp_ns")-F(r0,"timestamp_ns"))*1e-9
        sv=vec(r,"state")
        av=dot(sv,move_dir)
        cv=norm(sub(sv,tuple(av*x for x in move_dir)))
        dp=norm(sub(pos(r,"state"),p0))
        rows.append((k,dt,sv,norm(sv),av,cv,dp))

    picks=[]
    for target in (0,1,2,5,10,15,20):
        if not rows: continue
        q=min(rows,key=lambda x:abs(x[1]-target))
        if q not in picks: picks.append(q)
    for q in picks:
        k,dt,sv,sp,av,cv,dp=q
        print(f"{k:3d} {dt:7.2f}  [{sv[0]*1000:+7.2f},{sv[1]*1000:+7.2f},{sv[2]*1000:+7.2f}]"
              f" {sp*1000:6.2f} {av*1000:+7.2f} {cv*1000:7.2f} {dp*1000:10.2f}")

    if len(rows)>=2:
        # Compare integrated state velocity projection with observed state displacement.
        integ=[0.0,0.0,0.0]
        for a,b in zip(rows,rows[1:]):
            dt=b[1]-a[1]
            for j in range(3):
                integ[j]+=0.5*(a[2][j]+b[2][j])*dt
        obs=sub(pos(bykf[rows[-1][0]],"state"),p0)
        print("  integrated stateV over window =",f"[{integ[0]*1000:+.1f},{integ[1]*1000:+.1f},{integ[2]*1000:+.1f}] mm")
        print("  observed STATE displacement   =",f"[{obs[0]*1000:+.1f},{obs[1]*1000:+.1f},{obs[2]*1000:+.1f}] mm")

        speeds=[x[3]*1000 for x in rows]
        along=[x[4]*1000 for x in rows]
        cross=[x[5]*1000 for x in rows]
        print(f"  mean|V|={statistics.mean(speeds):.2f}mm/s mean along={statistics.mean(along):+.2f}mm/s mean cross={statistics.mean(cross):.2f}mm/s")

print("\n================ COMPONENT DOMINANCE ================")
for e in [x for x in events if x["event"]=="END"]:
    leg=int(e["leg"]); k0=I(e,"keyframe")
    vals=[]
    for k in range(k0,min(maxkf,k0+100)+1):
        r=bykf.get(k)
        if r: vals.append(tuple(abs(x)*1000 for x in vec(r,"state")))
    if vals:
        mx=[statistics.mean(v[j] for v in vals) for j in range(3)]
        print(f"LEG {leg}: mean |Vx|={mx[0]:.2f} |Vy|={mx[1]:.2f} |Vz|={mx[2]:.2f} mm/s")
