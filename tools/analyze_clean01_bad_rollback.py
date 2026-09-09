#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)

def nearest(rows,key,ts):
    return min(rows,key=lambda r:abs(I(r[key])-ts)) if rows else None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run")
    a=ap.parse_args()
    root=Path(a.run)

    ev=load(root/"events.csv")
    be=load(root/"backend.csv")
    fr=load(root/"frontend.csv")
    at=load(root/"attitude.csv")

    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    t0,t1=I(s["wall_ns"]),I(e["wall_ns"])
    mv=[r for r in be if t0<=I(r["callback_wall_ns"])<=t1]
    if len(mv)<3:raise SystemExit("not enough backend states")

    ax,ay,az=F(s["px"]),F(s["py"]),F(s["pz"])

    # Define forward axis from start to furthest horizontal backend state.
    def h2(r):
        dx=F(r["px"])-ax;dy=F(r["py"])-ay
        return dx*dx+dy*dy
    peak=max(mv,key=h2)
    ux=F(peak["px"])-ax;uy=F(peak["py"])-ay
    un=math.hypot(ux,uy)
    ux/=un;uy/=un

    print("="*154)
    print("CLEAN-01 — BAD RUN BACKEND ROLLBACK FORENSIC")
    print("="*154)
    print(f"run={root}")
    print(f"move={(t1-t0)/1e9:.3f}s backend_states={len(mv)}")
    print(f"forward axis defined by furthest backend point; peak radial H={un*1000:.2f} mm")
    print()
    print(f"{'KF':>4} {'t s':>6} {'prog mm':>9} {'dprog':>8} {'lat mm':>8} {'Vf mm/s':>9} "
          f"{'STATUS':<15} {'mono_t':>9} {'pitch°':>8} {'yaw°':>8}")
    print("-"*154)

    prev=None
    rollback_started=False
    rollback_kf=None
    total_neg=0.0

    for r in mv:
        x=F(r["px"])-ax;y=F(r["py"])-ay
        prog=(x*ux+y*uy)*1000
        lat=(-x*uy+y*ux)*1000
        vf=(F(r["vx"])*ux+F(r["vy"])*uy)*1000
        dp=0.0 if prev is None else prog-prev
        if dp<0:
            total_neg+=-dp
            if not rollback_started and dp<-2.0:
                rollback_started=True;rollback_kf=I(r["keyframe"])

        ts=I(r["timestamp_ns"])
        ff=nearest(fr,"timestamp_ns",ts)
        status="-";mt=float("nan")
        if ff and abs(I(ff["timestamp_ns"])-ts)<2_000_000:
            status=ff.get("mono_status","") or "(EMPTY)"
            mt=math.sqrt(F(ff["mono_tx"])**2+F(ff["mono_ty"])**2+F(ff["mono_tz"])**2)*1000

        aa=nearest(at,"recv_wall_ns",I(r["callback_wall_ns"]))
        pitch=yaw=float("nan")
        if aa:
            pitch=math.degrees(F(aa["pitch"]))
            yaw=math.degrees(F(aa["yaw"]))

        mark="  <-- ROLLBACK START" if rollback_kf==I(r["keyframe"]) else ""
        print(f"{I(r['keyframe']):4d} {(I(r['callback_wall_ns'])-t0)/1e9:6.2f} {prog:9.2f} {dp:+8.2f} "
              f"{lat:+8.2f} {vf:+9.2f} {status:<15} {mt:9.2f} {pitch:+8.3f} {yaw:+8.3f}{mark}")
        prev=prog

    print()
    print("SUMMARY")
    print("-"*154)
    print(f"first rollback keyframe (>2 mm negative step): {rollback_kf}")
    print(f"total negative projected backend motion during MOVE: {total_neg:.2f} mm")
    if rollback_kf is not None:
        print("Interpretation: endpoint collapse is caused by an actual backward correction/velocity phase, not by a single constant metric scale.")
    print("="*154)

if __name__=="__main__":main()
