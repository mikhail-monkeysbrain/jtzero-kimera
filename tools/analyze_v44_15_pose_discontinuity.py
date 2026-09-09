#!/usr/bin/env python3
"""V44.15 — isolate the kf89 visual-pose discontinuity and compare reference.

No new run. The important V44.14 observation is not merely VALID/non-VALID:
kf89 is still VALID with excellent inlier/tracked counts, but mono translation
changes abruptly from ~horizontal to strongly vertical. This script quantifies
that discontinuity and searches the reference run for comparable events.
"""
import argparse,csv,math,bisect
from pathlib import Path

def load(p):
    if not p.exists(): return []
    with p.open(newline="") as f:return list(csv.DictReader(f))
def F(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def bounds(run):
    q=load(run/"jtzero_500mm_v25_legs.csv")
    if not q: raise RuntimeError(f"missing legs csv: {run}")
    return I(q[0],"start_settled_kf"),I(q[0],"end_press_kf")
def nearest(front,t):
    if not front:return {},float("nan")
    ts=[I(r,"timestamp_ns") for r in front]
    j=bisect.bisect_left(ts,t); c=[]
    if j<len(front):c.append((abs(ts[j]-t),front[j]))
    if j:c.append((abs(ts[j-1]-t),front[j-1]))
    if not c:return {},float("nan")
    dt,r=min(c,key=lambda x:x[0]);return r,dt/1e6

def rows(run):
    lo,hi=bounds(run)
    b=[r for r in load(run/"jtzero_500mm_v25_backend.csv") if lo<=I(r,"keyframe")<=hi]
    f=sorted([r for r in load(run/"jtzero_500mm_v25_frontend.csv") if I(r,"timestamp_ns")>0],key=lambda r:I(r,"timestamp_ns"))
    if len(b)<2:raise RuntimeError(f"insufficient backend: {run}")
    x0,y0=F(b[0],"px_m"),F(b[0],"py_m")
    raw=[]
    for n,r in enumerate(b):
        fr,dt=nearest(f,I(r,"timestamp_ns"))
        x=(F(r,"px_m")-x0)*1000;y=(F(r,"py_m")-y0)*1000
        tx,ty,tz=F(fr,"mono_body_tx"),F(fr,"mono_body_ty"),F(fr,"mono_body_tz")
        hn=math.hypot(tx,ty)
        tilt=math.degrees(math.atan2(abs(tz),hn)) if hn>1e-12 and math.isfinite(tz) else float("nan")
        norm=math.sqrt(tx*tx+ty*ty+tz*tz) if all(math.isfinite(v) for v in (tx,ty,tz)) else float("nan")
        raw.append(dict(n=n,p=n/max(1,len(b)-1),kf=I(r,"keyframe"),t=I(r,"timestamp_ns"),x=x,y=y,
          status=fr.get("mono_status",""),valid=I(fr,"mono_pose_valid"),inlier=F(fr,"mono_inlier_ratio"),tracked=F(fr,"tracked_features"),
          tx=tx,ty=ty,tz=tz,tilt=tilt,norm=norm,dt=dt))
    anchor=raw[min(len(raw)-1,max(1,round(.8*(len(raw)-1)))]
    nn=math.hypot(anchor["x"],anchor["y"]);ux,uy=(anchor["x"]/nn,anchor["y"]/nn) if nn else (1,0)
    for n,r in enumerate(raw):
        r["along"]=r["x"]*ux+r["y"]*uy
        r["d"]=0 if n==0 else r["along"]-raw[n-1]["along"]
        if n:
            p=raw[n-1]
            if all(math.isfinite(v) for v in (r["tx"],r["ty"],r["tz"],p["tx"],p["ty"],p["tz"])):
                dot=r["tx"]*p["tx"]+r["ty"]*p["ty"]+r["tz"]*p["tz"]
                den=r["norm"]*p["norm"]
                r["pose_jump_deg"]=math.degrees(math.acos(max(-1,min(1,dot/den)))) if den>1e-12 else float("nan")
            else:r["pose_jump_deg"]=float("nan")
        else:r["pose_jump_deg"]=float("nan")
    return raw

def events(R):
    return [r for r in R if r["status"]=="VALID" and r["valid"]==1 and
            ((math.isfinite(r["tilt"]) and r["tilt"]>30) or (math.isfinite(r["pose_jump_deg"]) and r["pose_jump_deg"]>30))]

def show(name,R):
    E=events(R)
    print(f"\n{name}: VALID large-geometry events={len(E)}")
    print("-"*154)
    print("prog kf dAlong status inlier tracked dt_ms | tx       ty       tz       norm   tilt   jump")
    if not E:print("(none)")
    for r in E:
        print(f"{r['p']:4.0%} {r['kf']:3d} {r['d']:+7.1f} {r['status']:6s} {r['inlier']:6.3f} {r['tracked']:7.0f} {r['dt']:6.2f} | "
              f"{r['tx']:+8.5f} {r['ty']:+8.5f} {r['tz']:+8.5f} {r['norm']:7.4f} {r['tilt']:6.1f} {r['pose_jump_deg']:6.1f}")
    return E

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--reference",required=True);ap.add_argument("--current",required=True);a=ap.parse_args()
    A=rows(Path(a.reference));B=rows(Path(a.current))
    EA=show("REFERENCE",A);EB=show("CURRENT",B)
    print("\nCURRENT REVERSAL WINDOW")
    print("-"*154)
    for r in B:
        if .80<=r["p"]<=.95:
            print(f"kf={r['kf']:3d} p={r['p']:.0%} dAlong={r['d']:+6.1f}mm {r['status'] or '-':14s} inlier={r['inlier']:.3f} "
                  f"tracked={r['tracked']:.0f} tilt={r['tilt']:.1f}deg jump={r['pose_jump_deg']:.1f}deg t=({r['tx']:+.4f},{r['ty']:+.4f},{r['tz']:+.4f})")
    bad=[r for r in EB if r["d"]<-.5]
    print("\nDISCRIMINATOR")
    print("-"*154)
    if bad:
        r=bad[0]
        print(f"first reversal-associated VALID geometry event: kf={r['kf']} dAlong={r['d']:+.1f}mm tilt={r['tilt']:.1f}deg jump={r['pose_jump_deg']:.1f}deg")
    if EB and not EA:
        print("VERDICT: current run contains a VALID large visual-translation geometry discontinuity absent from reference.")
        print("This is stronger evidence than LOW_DISPARITY: the estimator accepts an anomalous pose before rejection begins.")
    elif len(EB)>len(EA):
        print("VERDICT: large VALID visual-translation geometry events are more frequent in current than reference.")
    else:
        print("VERDICT: comparable events exist in reference; visual translation discontinuity alone is not sufficient.")
    print("NEXT: if the current-only event is confirmed, gate/diagnose the anomalous VALID pose before backend fusion; do not change focal scale.")
    print("="*154)
if __name__=="__main__":main()
