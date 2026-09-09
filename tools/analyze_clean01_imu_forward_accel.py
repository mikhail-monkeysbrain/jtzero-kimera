#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

def load(p):
    with p.open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)
def med(v):return statistics.median(v) if v else float("nan")

def RzRyRx(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return (
      (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
      (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
      (-sp,   cp*sr,          cp*cr)
    )
def mv(R,v):
    return tuple(sum(R[i][j]*v[j] for j in range(3)) for i in range(3))
def nearest(rows,key,t):
    return min(rows,key=lambda r:abs(I(r[key])-t))

def analyze(root,label,last_sec,bin_s):
    ev=load(root/"events.csv")
    be=load(root/"backend.csv")
    imu=load(root/"imu.csv")
    s=next(r for r in ev if r["event"]=="MOVE_START")
    e=next(r for r in ev if r["event"]=="MOVE_END")
    t0,t1=I(s["wall_ns"]),I(e["wall_ns"])
    ax,ay=F(s["px"]),F(s["py"])
    mvbe=[r for r in be if t0<=I(r["callback_wall_ns"])<=t1]
    peak=max(mvbe,key=lambda r:(F(r["px"])-ax)**2+(F(r["py"])-ay)**2)
    ux,uy=F(peak["px"])-ax,F(peak["py"])-ay
    un=math.hypot(ux,uy);ux/=un;uy/=un

    start=max(t0,t1-int(last_sec*1e9))
    samples=[]
    for r in imu:
        tw=I(r["recv_wall_ns"])
        if not (start<=tw<=t1):continue
        b=nearest(be,"callback_wall_ns",tw)
        rr=math.radians(F(b["roll_deg"]))
        pp=math.radians(F(b["pitch_deg"]))
        yy=math.radians(F(b["yaw_deg"]))
        R=RzRyRx(rr,pp,yy)
        # Bias-correct FLU accelerometer. Horizontal world projection does not
        # require explicit gravity subtraction because gravity is vertical.
        ab=(F(r["flu_ax"])-F(b["bax"]),
            F(r["flu_ay"])-F(b["bay"]),
            F(r["flu_az"])-F(b["baz"]))
        aw=mv(R,ab)
        af=aw[0]*ux+aw[1]*uy
        samples.append((tw,af))

    integ=0.0
    for (ta,aa),(tb,ab) in zip(samples,samples[1:]):
        dt=(tb-ta)*1e-9
        if 0<dt<0.05:integ+=0.5*(aa+ab)*dt

    print()
    print("="*126)
    print(label,root.name)
    print("="*126)
    print(f"window={last_sec:.3f}s  imu_samples={len(samples)}  peak_backend_H={un*1000:.2f}mm")
    print(f"integrated projected IMU delta-v over window = {integ*1000:+.2f} mm/s")
    print()
    print(f"{'to_END s':>12} {'n':>6} {'a_forward med':>15} {'a_forward p10':>15} {'a_forward p90':>15}")
    step=int(bin_s*1e9)
    k=start
    while k<t1:
        z=[a for t,a in samples if k<=t<min(k+step,t1)]
        if z:
            zs=sorted(z)
            p10=zs[max(0,int(.10*(len(zs)-1)))]
            p90=zs[min(len(zs)-1,int(.90*(len(zs)-1)))]
            mid=(k+min(k+step,t1))//2
            print(f"{(mid-t1)/1e9:+12.3f} {len(z):6d} {med(z):+15.4f} {p10:+15.4f} {p90:+15.4f}")
        k+=step

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bad")
    ap.add_argument("good",nargs="+")
    ap.add_argument("--last-sec",type=float,default=2.2)
    ap.add_argument("--bin-sec",type=float,default=0.25)
    a=ap.parse_args()
    analyze(Path(a.bad),"BAD",a.last_sec,a.bin_sec)
    for j,p in enumerate(a.good,1):
        analyze(Path(p),f"GOOD{j}",a.last_sec,a.bin_sec)
    print()
    print("INTERPRETATION")
    print("-"*126)
    print("This projects bias-corrected FLU accelerometer samples into the backend world frame and onto the run's forward axis.")
    print("If BAD accumulates a much more negative IMU delta-v than GOOD, the rollback is driven by inertial propagation/deceleration input.")
    print("If IMU delta-v is comparable but backend velocity differs strongly, the divergence is created in visual-inertial fusion/state optimization.")
    print("="*126)
if __name__=="__main__":main()
