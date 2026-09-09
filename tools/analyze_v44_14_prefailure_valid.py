#!/usr/bin/env python3
"""V44.14 — forensic split of the late reversal into visual vs backend causes.

No new physical run. Uses archived frontend/backend CSVs and exact timestamp rows.
Key point: do not classify LOW_DISPARITY rows as the cause of a reversal that
already starts on preceding VALID rows.
"""
import argparse,csv,math,bisect
from pathlib import Path

def load(p):
    if not p.exists(): return []
    with p.open(newline="") as f:return list(csv.DictReader(f))
def f(r,k,d=float("nan")):
    try:return float(r.get(k,d))
    except:return d
def i(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d

def bounds(run):
    q=load(run/"jtzero_500mm_v25_legs.csv")
    if not q: raise RuntimeError("missing legs csv")
    return i(q[0],"start_settled_kf"),i(q[0],"end_press_kf")

def nearest(rows,t):
    rows=sorted(rows,key=lambda r:i(r,"timestamp_ns"))
    ts=[i(r,"timestamp_ns") for r in rows]
    j=bisect.bisect_left(ts,t); c=[]
    if j<len(rows):c.append((abs(ts[j]-t),rows[j]))
    if j:c.append((abs(ts[j-1]-t),rows[j-1]))
    return min(c,key=lambda x:x[0])[1] if c else {}

def series(run):
    lo,hi=bounds(run)
    B=[r for r in load(run/"jtzero_500mm_v25_backend.csv") if lo<=i(r,"keyframe")<=hi]
    F=load(run/"jtzero_500mm_v25_frontend.csv")
    if len(B)<2: raise RuntimeError("insufficient backend")
    x0,y0=f(B[0],"px_m"),f(B[0],"py_m")
    tmp=[]
    for n,b in enumerate(B):
        x=(f(b,"px_m")-x0)*1000;y=(f(b,"py_m")-y0)*1000
        tmp.append((x,y,b,nearest(F,i(b,"timestamp_ns"))))
    a=tmp[min(len(tmp)-1,max(1,round(.8*(len(tmp)-1))))]
    nn=math.hypot(a[0],a[1]); ux,uy=(a[0]/nn,a[1]/nn) if nn else (1,0)
    out=[]
    for n,(x,y,b,fr) in enumerate(tmp):
        along=x*ux+y*uy
        prev=out[-1] if out else None
        out.append(dict(
          p=n/max(1,len(tmp)-1),kf=i(b,"keyframe"),t=i(b,"timestamp_ns"),
          along=along,d=0 if prev is None else along-prev["along"],
          vx=f(b,"vx_mps"),vy=f(b,"vy_mps"),vz=f(b,"vz_mps"),
          status=fr.get("mono_status",""),inlier=f(fr,"mono_inlier_ratio"),
          tracked=f(fr,"tracked_features"),valid=i(fr,"mono_pose_valid"),
          tx=f(fr,"mono_body_tx"),ty=f(fr,"mono_body_ty"),tz=f(fr,"mono_body_tz"),
          front=fr))
    return out,ux,uy

def proj_vel(r,ux,uy):
    if math.isfinite(r["vx"]) and math.isfinite(r["vy"]):return 1000*(r["vx"]*ux+r["vy"]*uy)
    return float("nan")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--reference",required=True);ap.add_argument("--current",required=True)
    a=ap.parse_args()
    A,uax,uay=series(Path(a.reference));B,ubx,uby=series(Path(a.current))
    peak=max(range(len(B)),key=lambda j:B[j]["along"])
    firstneg=next((j for j in range(1,len(B)) if B[j]["d"]<-.5),None)
    print("="*150)
    print("V44.14 — PRE-FAILURE VALID-ROW FORENSIC")
    print("="*150)
    print(f"current peak: kf={B[peak]['kf']} progress={B[peak]['p']:.0%} along={B[peak]['along']:.1f}mm")
    print(f"first negative: kf={B[firstneg]['kf']} progress={B[firstneg]['p']:.0%} dAlong={B[firstneg]['d']:+.1f}mm" if firstneg else "no negative step")
    print("\nCURRENT WINDOW")
    print("-"*150)
    print("prog kf  along  dAlong status          inlier tracked valid | backend along-v mm/s | mono tx       ty       tz    horiz_t  z/h")
    for j in range(max(0,(firstneg or peak)-3),min(len(B),(firstneg or peak)+7)):
        r=B[j]; hv=math.hypot(r["tx"],r["ty"]); ratio=abs(r["tz"])/hv if hv>1e-12 else float("nan")
        print(f"{r['p']:4.0%} {r['kf']:3d} {r['along']:7.1f} {r['d']:+7.1f} {r['status'] or '-':14s} {r['inlier']:6.3f} {r['tracked']:7.0f} {r['valid']:5d} | "
              f"{proj_vel(r,ubx,uby):+18.1f} | {r['tx']:+8.5f} {r['ty']:+8.5f} {r['tz']:+8.5f} {hv:8.5f} {ratio:6.2f}")

    valid_neg=[r for r in B[1:] if r["d"]<-.5 and r["status"]=="VALID" and r["valid"]==1]
    invalid_neg=[r for r in B[1:] if r["d"]<-.5 and not (r["status"]=="VALID" and r["valid"]==1)]
    pre_loss=sum(-r["d"] for r in valid_neg)
    post_loss=sum(-r["d"] for r in invalid_neg)
    print("\nCAUSAL ORDER")
    print("-"*150)
    print(f"negative displacement accumulated on VALID pose rows : {pre_loss:.1f} mm ({len(valid_neg)} steps)")
    print(f"negative displacement accumulated on non-VALID rows : {post_loss:.1f} mm ({len(invalid_neg)} steps)")
    if firstneg is not None:
        r=B[firstneg]
        print(f"reversal begins with status={r['status']} pose_valid={r['valid']} inlier={r['inlier']:.3f} tracked={r['tracked']:.0f}")
    if pre_loss>post_loss:
        print("VERDICT: LOW_DISPARITY is too late to explain the onset. The dominant reversal is already created while the monocular pose is VALID.")
        print("NEXT BRANCH: inspect the VALID visual translation vector / backend velocity-state sign around the first negative step; do not tune focal or LOW_DISPARITY thresholds.")
    else:
        print("VERDICT: most reversal is accumulated after visual pose becomes non-VALID; rejection/hold behavior remains causal candidate.")
    print("="*150)

if __name__=="__main__":main()
