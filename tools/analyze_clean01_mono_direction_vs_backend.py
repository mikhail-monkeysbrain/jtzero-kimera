#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)

def nearest(rows,key,ts):
    return min(rows,key=lambda r:abs(I(r[key])-ts)) if rows else None

def run(root,label,last_sec):
    ev=load(root/"events.csv")
    be=load(root/"backend.csv")
    fr=load(root/"frontend.csv")
    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    t0,t1=I(s["wall_ns"]),I(e["wall_ns"])
    ax,ay=F(s["px"]),F(s["py"])
    mv=[r for r in be if t0<=I(r["callback_wall_ns"])<=t1]
    peak=max(mv,key=lambda r:(F(r["px"])-ax)**2+(F(r["py"])-ay)**2)
    ux,uy=F(peak["px"])-ax,F(peak["py"])-ay
    n=math.hypot(ux,uy);ux/=n;uy/=n

    print()
    print("="*154)
    print(label,root.name)
    print("="*154)
    print(f"{'KF':>4} {'to_END':>8} {'prog':>8} {'Vf':>9} {'STATUS':<15} "
          f"{'mono_tx':>9} {'mono_ty':>9} {'mono_tz':>9} {'mono_dir_dot_prev':>18}")
    prev_dir=None
    for b in mv:
        wall=I(b["callback_wall_ns"])
        if wall<t1-int(last_sec*1e9):continue
        x,y=F(b["px"])-ax,F(b["py"])-ay
        prog=(x*ux+y*uy)*1000
        vf=(F(b["vx"])*ux+F(b["vy"])*uy)*1000
        ff=nearest(fr,"timestamp_ns",I(b["timestamp_ns"]))
        status="-";tx=ty=tz=float("nan");dot=float("nan")
        if ff and abs(I(ff["timestamp_ns"])-I(b["timestamp_ns"]))<2_000_000:
            status=ff.get("mono_status","") or "(EMPTY)"
            if ff.get("mono_pose_valid","0")=="1":
                tx,ty,tz=F(ff["mono_tx"]),F(ff["mono_ty"]),F(ff["mono_tz"])
                q=math.sqrt(tx*tx+ty*ty+tz*tz)
                if q>0:
                    cur=(tx/q,ty/q,tz/q)
                    if prev_dir is not None:
                        dot=sum(a*b for a,b in zip(cur,prev_dir))
                    prev_dir=cur
        print(f"{I(b['keyframe']):4d} {(wall-t1)/1e9:+8.3f} {prog:8.2f} {vf:+9.2f} "
              f"{status:<15} {tx:+9.4f} {ty:+9.4f} {tz:+9.4f} {dot:+18.5f}")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bad")
    ap.add_argument("good",nargs="+")
    ap.add_argument("--last-sec",type=float,default=2.2)
    a=ap.parse_args()
    run(Path(a.bad),"BAD",a.last_sec)
    for j,p in enumerate(a.good,1):run(Path(p),f"GOOD{j}",a.last_sec)
    print()
    print("INTERPRETATION")
    print("-"*154)
    print("mono_t is a direction, not metric translation. dot≈+1 means consecutive mono directions agree; dot<0 means a direction reversal.")
    print("If backend Vf turns strongly negative while mono direction remains continuous, the catastrophic rollback is created after mono geometry.")
    print("="*154)

if __name__=="__main__":main()
