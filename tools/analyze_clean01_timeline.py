#!/usr/bin/env python3
import argparse,csv,math,statistics
from collections import Counter
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(x): return int(x)
def F(x): return float(x)
def med(v): return statistics.median(v) if v else float("nan")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run")
    a=ap.parse_args()
    root=Path(a.run)

    ev=load(root/"events.csv")
    fr=load(root/"frontend.csv")
    be=load(root/"backend.csv")
    imu=load(root/"imu.csv")
    att=load(root/"attitude.csv")

    s=[r for r in ev if r["event"]=="MOVE_START"][0]
    e=[r for r in ev if r["event"]=="MOVE_END"][0]
    t0,t1=I(s["wall_ns"]),I(e["wall_ns"])

    fm=[r for r in fr if t0<=I(r["callback_wall_ns"])<=t1]
    bm=[r for r in be if t0<=I(r["callback_wall_ns"])<=t1]
    im=[r for r in imu if t0<=I(r["recv_wall_ns"])<=t1]
    am=[r for r in att if t0<=I(r["recv_wall_ns"])<=t1]

    print("="*120)
    print("CLEAN-01 — SAME-RUN FRONTEND / BACKEND TIMELINE FORENSIC")
    print("="*120)
    print(f"run={root}")
    print(f"move={(t1-t0)/1e9:.3f}s frontend={len(fm)} backend={len(bm)} imu={len(im)} attitude={len(am)}")

    print("\nMONO STATUS DISTRIBUTION")
    print("-"*120)
    c=Counter((r.get("mono_status") or "<empty>") for r in fm)
    for k,n in c.most_common():
        print(f"{k:32s} {n:4d}  {100*n/max(1,len(fm)):6.2f}%")

    print("\nKEYFRAME / POSE VALIDITY")
    print("-"*120)
    kf=[r for r in fm if I(r["is_keyframe"])==1]
    valid=[r for r in fm if I(r["mono_pose_valid"])==1]
    valid_kf=[r for r in kf if I(r["mono_pose_valid"])==1]
    print(f"keyframe callbacks={len(kf)}/{len(fm)} ({100*len(kf)/max(1,len(fm)):.1f}%)")
    print(f"mono_pose_valid all={len(valid)}/{len(fm)} ({100*len(valid)/max(1,len(fm)):.1f}%)")
    print(f"mono_pose_valid keyframes={len(valid_kf)}/{len(kf) if kf else 1} ({100*len(valid_kf)/max(1,len(kf)):.1f}%)")

    def qstats(rs,label):
        if not rs:
            print(label+": none"); return
        tr=[I(r["tracked"]) for r in rs]
        inn=[I(r["inliers"]) for r in rs]
        put=[I(r["putatives"]) for r in rs]
        rat=[a/b if b else 0 for a,b in zip(inn,put)]
        print(f"{label:20s} n={len(rs):3d} tracked_med={med(tr):6.1f} inliers_med={med(inn):6.1f} put_med={med(put):6.1f} ratio_med={med(rat):.3f}")

    print("\nFRONTEND QUALITY BY STATUS")
    print("-"*120)
    qstats(fm,"ALL")
    for status,_ in c.most_common():
        qstats([r for r in fm if (r.get("mono_status") or "<empty>")==status],status[:20])

    print("\nTIME BINS")
    print("-"*120)
    nb=8
    dur=t1-t0
    for j in range(nb):
        a0=t0+dur*j//nb; a1=t0+dur*(j+1)//nb
        rr=[r for r in fm if a0<=I(r["callback_wall_ns"])<a1]
        bb=[r for r in bm if a0<=I(r["callback_wall_ns"])<a1]
        if rr:
            inn=[I(r["inliers"]) for r in rr]
            put=[I(r["putatives"]) for r in rr]
            ratios=[x/y if y else 0 for x,y in zip(inn,put)]
            vv=sum(I(r["mono_pose_valid"]) for r in rr)
            st=Counter((r.get("mono_status") or "<empty>") for r in rr).most_common(1)[0][0]
        else:
            ratios=[];vv=0;st="-"
        if len(bb)>=2:
            p0=bb[0]; p1=bb[-1]
            dx=(F(p1["px"])-F(p0["px"]))*1000
            dy=(F(p1["py"])-F(p0["py"]))*1000
            dz=(F(p1["pz"])-F(p0["pz"]))*1000
            dh=math.hypot(dx,dy)
        else: dx=dy=dz=dh=float("nan")
        print(f"bin {j+1}: t={j*dur/nb/1e9:5.2f}-{(j+1)*dur/nb/1e9:5.2f}s "
              f"front={len(rr):3d} valid={vv:2d} ratio_med={med(ratios):.3f} dom={st:20s} "
              f"backend_dH={dh:7.2f}mm dZ={dz:+7.2f}mm")

    print("\nATTITUDE / IMU OBSERVATION")
    print("-"*120)
    if am:
        roll=[math.degrees(F(r["roll"])) for r in am]
        pitch=[math.degrees(F(r["pitch"])) for r in am]
        yaw=[math.degrees(F(r["yaw"])) for r in am]
        print(f"roll min/max/span  = {min(roll):+.3f}/{max(roll):+.3f}/{max(roll)-min(roll):.3f} deg")
        print(f"pitch min/max/span = {min(pitch):+.3f}/{max(pitch):+.3f}/{max(pitch)-min(pitch):.3f} deg")
        print(f"yaw min/max/span   = {min(yaw):+.3f}/{max(yaw):+.3f}/{max(yaw)-min(yaw):.3f} deg")
    if im:
        ax=[F(r["flu_ax"]) for r in im]; ay=[F(r["flu_ay"]) for r in im]; az=[F(r["flu_az"]) for r in im]
        gx=[F(r["flu_gx"]) for r in im]; gy=[F(r["flu_gy"]) for r in im]; gz=[F(r["flu_gz"]) for r in im]
        horiz=[math.hypot(x,y) for x,y in zip(ax,ay)]
        gnorm=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(gx,gy,gz)]
        print(f"IMU horiz accel RMS={math.sqrt(sum(x*x for x in horiz)/len(horiz)):.4f} m/s^2 p90={statistics.quantiles(horiz,n=10)[8]:.4f}")
        print(f"IMU gyro norm RMS={math.sqrt(sum(x*x for x in gnorm)/len(gnorm)):.5f} rad/s p90={statistics.quantiles(gnorm,n=10)[8]:.5f}")

    print("\nBACKEND STEP DISTRIBUTION")
    print("-"*120)
    steps=[]
    for p,q in zip(bm,bm[1:]):
        dx=(F(q["px"])-F(p["px"]))*1000
        dy=(F(q["py"])-F(p["py"]))*1000
        dz=(F(q["pz"])-F(p["pz"]))*1000
        dt=(I(q["timestamp_ns"])-I(p["timestamp_ns"]))/1e9
        steps.append((math.sqrt(dx*dx+dy*dy+dz*dz),math.hypot(dx,dy),dz,dt,I(q["keyframe"])))
    if steps:
        norms=[x[0] for x in steps]; horiz=[x[1] for x in steps]
        print(f"steps={len(steps)} 3D med={med(norms):.2f}mm max={max(norms):.2f}mm horiz med={med(horiz):.2f}mm")
        print("largest 8:")
        for n,h,z,dt,k in sorted(steps,reverse=True)[:8]:
            print(f"  kf={k:4d} d3={n:7.2f}mm dH={h:7.2f}mm dZ={z:+7.2f}mm dt={dt*1000:6.1f}ms")

    print("\nNO HISTORICAL DATA USED.")
    print("="*120)

if __name__=="__main__":
    main()
