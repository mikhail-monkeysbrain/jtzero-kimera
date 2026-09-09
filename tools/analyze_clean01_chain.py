#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path

def load(p):
    with Path(p).open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)
def nearest(rows,ts):
    return min(rows,key=lambda r:abs(I(r["timestamp_ns"])-ts))
def disp(a,b,prefix):
    ax,ay,az=[F(a[prefix+k]) for k in ("px","py","pz")]
    bx,by,bz=[F(b[prefix+k]) for k in ("px","py","pz")]
    dx,dy,dz=bx-ax,by-ay,bz-az
    h=math.hypot(dx,dy);d=math.sqrt(dx*dx+dy*dy+dz*dz)
    return dx,dy,dz,h,d
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--chain",default="/home/vio/jtzero_kimera_chain.csv")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    a=ap.parse_args()
    run=Path(a.run)
    ev=load(run/"events.csv");ch=load(a.chain)
    st=[r for r in ev if r["event"]=="MOVE_START"];en=[r for r in ev if r["event"]=="MOVE_END"]
    if len(st)!=1 or len(en)!=1:raise SystemExit("need one MOVE_START/MOVE_END")
    ts0=I(st[0]["state_timestamp_ns"]);ts1=I(en[0]["state_timestamp_ns"])
    c0=nearest(ch,ts0);c1=nearest(ch,ts1)
    print("="*118)
    print("CLEAN-01 REPLAY — BACKEND CHAIN LOCALIZATION")
    print("="*118)
    print(f"run={run}")
    print(f"event state ts START={ts0} END={ts1}")
    print(f"nearest chain START kf={c0['keyframe']} dt={(I(c0['timestamp_ns'])-ts0)/1e6:+.3f}ms")
    print(f"nearest chain END   kf={c1['keyframe']} dt={(I(c1['timestamp_ns'])-ts1)/1e6:+.3f}ms")
    print()
    for label,p in [("IMU PREDICTION","pred_"),("OPTIMIZED STATE","state_"),("INCREMENTAL OUTPUT","incr_")]:
        dx,dy,dz,h,d=disp(c0,c1,p)
        print(f"{label:20s} dP=[{dx*1000:+8.2f},{dy*1000:+8.2f},{dz*1000:+8.2f}] mm  "
              f"H={h*1000:8.2f}mm  3D={d*1000:8.2f}mm  scale={h*1000/a.truth_mm:.5f}")
    print()
    s0=math.sqrt((F(c0["state_px"])-F(c0["incr_px"]))**2+(F(c0["state_py"])-F(c0["incr_py"]))**2+(F(c0["state_pz"])-F(c0["incr_pz"]))**2)
    s1=math.sqrt((F(c1["state_px"])-F(c1["incr_px"]))**2+(F(c1["state_py"])-F(c1["incr_py"]))**2+(F(c1["state_pz"])-F(c1["incr_pz"]))**2)
    print(f"|state-increments| START={s0*1000:.2f}mm END={s1*1000:.2f}mm")
    print("\nDECISION")
    print("-"*118)
    _,_,_,hp,_=disp(c0,c1,"pred_")
    _,_,_,hs,_=disp(c0,c1,"state_")
    _,_,_,hi,_=disp(c0,c1,"incr_")
    vals={"prediction":hp*1000,"state":hs*1000,"increments":hi*1000}
    for k,v in vals.items():print(f"{k:12s}: {v:8.2f} mm")
    if abs(hs-a.truth_mm)<abs(hi-a.truth_mm)-50:
        print("LOCALIZATION: optimized state is materially closer to truth than incremental output -> output chaining is primary suspect.")
    elif abs(hp-a.truth_mm)<abs(hs-a.truth_mm)-50:
        print("LOCALIZATION: IMU prediction is materially closer to truth than optimized state -> optimization/factors are primary suspect.")
    elif max(vals.values())-min(vals.values())<30:
        print("LOCALIZATION: prediction/state/increments agree closely -> scale loss occurs before or consistently across backend state construction.")
    else:
        print("LOCALIZATION: chain branches diverge; inspect the first branch whose displacement collapses.")
    print("="*118)
if __name__=="__main__":main()
