#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

root = Path(sys.argv[1]) if len(sys.argv)>1 else Path(Path("/home/vio/jtzero_p11_latest_imu_path_run.txt").read_text().strip())
rows=list(csv.DictReader((root/"p11_imu_path.csv").open()))
types=("RAW_IMU","SCALED_IMU","SCALED_IMU2","HIGHRES_IMU")
pairs=(("A1","B1"),("A2","B2"),("A3","B3"))

def norm(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def scale(v,s): return tuple(x*s for x in v)
def mean_vec(rr):
    return tuple(statistics.mean(float(r[k]) for r in rr) for k in ("ax_si","ay_si","az_si"))
def ang(a,b):
    na,nb=norm(a),norm(b)
    c=max(-1.0,min(1.0,dot(a,b)/(na*nb)))
    return math.degrees(math.acos(c))
def plateau(typ,lab):
    x=[r for r in rows if r["type"]==typ and r["recording"]=="1" and r["stage"]==lab]
    if not x: return None
    tm=max(int(r["recv_ns"]) for r in x)
    x=[r for r in x if int(r["recv_ns"])>=tm-5_000_000_000]
    return mean_vec(x)

print("================ P11 IMU PAIRWISE A/B ================")
print("run:",root)
print("Pairs: A1-B1, A2-B2, A3-B3. This avoids mixing distant plateaus into one global mean.")

all_delta={}
for typ in types:
    print(f"\n[{typ}]")
    deltas=[]
    for a_lab,b_lab in pairs:
        A=plateau(typ,a_lab); B=plateau(typ,b_lab)
        if A is None or B is None: continue
        d=sub(B,A)
        ahat=scale(A,1.0/norm(A))
        par=dot(d,ahat)
        perp=sub(d,scale(ahat,par))
        ratio=norm(B)/norm(A)
        pairang=ang(A,B)
        deltas.append(d)
        print(f"{a_lab}->{b_lab}:")
        print(f"  |A|={norm(A):.6f} |B|={norm(B):.6f}  B-A norm={norm(B)-norm(A):+.6f}  ratio={ratio:.7f}")
        print(f"  vector angle={pairang:.5f} deg")
        print(f"  d=[{d[0]:+.6f},{d[1]:+.6f},{d[2]:+.6f}] |d|={norm(d):.6f}")
        print(f"  parallel={par:+.6f}  perpendicular={norm(perp):.6f}")
        if norm(d)>0:
            ep=100*(par*par)/(norm(d)*norm(d))
            et=100*(norm(perp)*norm(perp))/(norm(d)*norm(d))
            print(f"  energy share: parallel={ep:.2f}% perpendicular={et:.2f}%")
    if deltas:
        md=tuple(statistics.mean(d[i] for d in deltas) for i in range(3))
        all_delta[typ]=md
        print(f"  mean pair delta=[{md[0]:+.6f},{md[1]:+.6f},{md[2]:+.6f}] |d|={norm(md):.6f}")

print("\n================ CROSS-IMU DELTA DIRECTION ================")
keys=[k for k in ("SCALED_IMU","SCALED_IMU2","HIGHRES_IMU") if k in all_delta]
for i in range(len(keys)):
    for j in range(i+1,len(keys)):
        a,b=keys[i],keys[j]
        print(f"{a} vs {b}: angle between mean B-A vectors = {ang(all_delta[a],all_delta[b]):.5f} deg")

print("\nINTERPRETATION:")
print("- Similar pairwise norm drop in all three A->B pairs => the magnitude shift is repeatable, not just a global-mean artifact.")
print("- Perpendicular energy near 100% => most of the full vector change is orientation/projection.")
print("- Non-zero, repeatable norm drop remains a separate smaller effect; pure rotation alone cannot create it.")
print("- Nearly identical B-A vector direction on IMU1 and IMU2 strengthens a common physical/FC factor over a single-IMU-only fault.")
