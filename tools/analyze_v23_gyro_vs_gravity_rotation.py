#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv) < 2:
    print("usage: analyze_v23_gyro_vs_gravity_rotation.py RUN [RUN ...]")
    sys.exit(2)

def mean(xs): return statistics.mean(xs) if xs else float("nan")
def med(xs): return statistics.median(xs) if xs else float("nan")
def norm(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def angle_deg(a,b):
    na,nb=norm(a),norm(b)
    c=max(-1.0,min(1.0,dot(a,b)/(na*nb)))
    return math.degrees(math.acos(c))
def vmean(rr,ks): return tuple(mean([float(r[k]) for r in rr]) for k in ks)

def phase_window(back,leg,phase):
    rr=[r for r in back if int(r["leg"])==leg and r["phase"]==phase]
    if not rr: return None
    ts=[int(r["timestamp_ns"]) for r in rr]
    return min(ts),max(ts)

def central(rr,key,w):
    x=sorted([r for r in rr if w[0] <= int(r[key]) <= w[1]], key=lambda r:int(r[key]))
    k=len(x)//4
    return x[k:len(x)-k] if len(x)-2*k>=3 else x

# Quaternion represents relative body orientation R_start_to_current.
# Body gyro integration: q <- q * Exp(w*dt).
def qmul(a,b):
    aw,ax,ay,az=a; bw,bx,by,bz=b
    return (
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    )
def qconj(q): return (q[0],-q[1],-q[2],-q[3])
def qnorm(q):
    n=math.sqrt(sum(x*x for x in q))
    return tuple(x/n for x in q)
def qexp(wdt):
    a=norm(wdt)
    if a<1e-15: return (1.0,0.0,0.0,0.0)
    s=math.sin(a/2.0)/a
    return (math.cos(a/2.0),wdt[0]*s,wdt[1]*s,wdt[2]*s)
def qrot(q,v):
    p=(0.0,v[0],v[1],v[2])
    z=qmul(qmul(q,p),qconj(q))
    return (z[1],z[2],z[3])

allr=[]
print("================ V23 GYRO-INTEGRATED ROTATION vs RAW GRAVITY ================")
print("Raw HIGHRES_IMU gyro/accel are both in FC FRD in the CSV.")
print("Gyro bias is estimated from both stationary endpoint windows.")
print("Physical rigid-body tilt should be supported by net gyro-integrated rotation.")

for arg in sys.argv[1:]:
    run=Path(arg)
    def rd(name):
        with (run/name).open() as f: return list(csv.DictReader(f))
    imu=rd("jtzero_500mm_v23.csv")
    back=rd("jtzero_500mm_v23_backend.csv")
    legs=rd("jtzero_500mm_v23_legs.csv")
    imu=sorted(imu,key=lambda r:int(r["mapped_ns"]))
    print("\nRUN:",run)

    for lr in legs:
        leg=int(lr["leg"]); direction=lr["direction"]
        w0=phase_window(back,leg,"SETTLE_START")
        w1=phase_window(back,leg,"SETTLE_END")
        if not w0 or not w1: continue

        s0=central(imu,"mapped_ns",w0)
        s1=central(imu,"mapped_ns",w1)
        if len(s0)<3 or len(s1)<3:
            print(f"LEG {leg} {direction}: insufficient stationary samples")
            continue

        a0=vmean(s0,("ax","ay","az")); a1=vmean(s1,("ax","ay","az"))
        raw_grav_tilt=angle_deg(a0,a1)

        g0=vmean(s0,("gx","gy","gz")); g1=vmean(s1,("gx","gy","gz"))
        bias=tuple((g0[i]+g1[i])*0.5 for i in range(3))

        t0=int(statistics.median([int(r["mapped_ns"]) for r in s0]))
        t1=int(statistics.median([int(r["mapped_ns"]) for r in s1]))
        seg=[r for r in imu if t0 <= int(r["mapped_ns"]) <= t1]

        q=(1.0,0.0,0.0,0.0)
        used=0; duration=0.0; prev=None
        for r in seg:
            t=int(r["mapped_ns"])
            if prev is not None:
                dt=(t-prev[0])*1e-9
                if 0.0 < dt <= 0.03:
                    w=(
                        0.5*(float(prev[1]["gx"])+float(r["gx"]))-bias[0],
                        0.5*(float(prev[1]["gy"])+float(r["gy"]))-bias[1],
                        0.5*(float(prev[1]["gz"])+float(r["gz"]))-bias[2],
                    )
                    q=qnorm(qmul(q,qexp(tuple(x*dt for x in w))))
                    duration+=dt; used+=1
            prev=(t,r)

        # Gravity in body coordinates evolves with inverse body rotation.
        a0u=tuple(x/norm(a0) for x in a0)
        pred=qrot(qconj(q),a0u)
        gyro_grav_tilt=angle_deg(a0u,pred)

        # Total net orientation angle and horizontal-axis component of rotation vector.
        qw=max(-1.0,min(1.0,q[0]))
        total_angle=2.0*math.acos(abs(qw))
        s=math.sqrt(max(0.0,1.0-qw*qw))
        if s<1e-12:
            rv=(0.0,0.0,0.0)
        else:
            axis=(q[1]/s,q[2]/s,q[3]/s)
            # preserve q sign near identity
            ang=2.0*math.acos(qw)
            if ang>math.pi: ang-=2.0*math.pi
            rv=tuple(axis[i]*ang for i in range(3))
        horiz_rot=math.degrees(math.hypot(rv[0],rv[1]))
        total_deg=math.degrees(total_angle)

        end_pred_err=angle_deg(pred,tuple(x/norm(a1) for x in a1))
        ratio=gyro_grav_tilt/raw_grav_tilt if raw_grav_tilt>1e-9 else float("nan")

        print(f"LEG {leg} {direction}: samples={len(seg)} integ_used={used} duration={duration:.3f}s")
        print(f"  gyro bias FRD=[{bias[0]:+.6f},{bias[1]:+.6f},{bias[2]:+.6f}] rad/s")
        print(f"  RAW gravity-vector change={raw_grav_tilt:.4f} deg")
        print(f"  gyro-predicted gravity change={gyro_grav_tilt:.4f} deg")
        print(f"  net gyro rotation total={total_deg:.4f} deg horizontal-rotvec={horiz_rot:.4f} deg")\n        print(f"  net gyro rotvec FRD=[{math.degrees(rv[0]):+.4f},{math.degrees(rv[1]):+.4f},{math.degrees(rv[2]):+.4f}] deg")
        print(f"  predicted-vs-measured end gravity error={end_pred_err:.4f} deg")
        print(f"  gyro/raw gravity ratio={ratio:.3f}")
        allr.append((direction,raw_grav_tilt,gyro_grav_tilt,end_pred_err,horiz_rot,total_deg,ratio,math.degrees(rv[0]),math.degrees(rv[1]),math.degrees(rv[2])))

print("\n================ DIRECTION SUMMARY ================")
for d in ("A->B","B->A"):
    rr=[x for x in allr if x[0]==d]
    if rr:
        print(f"{d}: n={len(rr)} RAW={med([x[1] for x in rr]):.4f}deg "
              f"GYRO-pred={med([x[2] for x in rr]):.4f}deg "
              f"end-error={med([x[3] for x in rr]):.4f}deg "
              f"horiz-rot={med([x[4] for x in rr]):.4f}deg "
              f"GYRO/RAW={med([x[6] for x in rr]):.3f} " +\n              f"rotvecFRD_med=[{med([x[7] for x in rr]):+.3f},{med([x[8] for x in rr]):+.3f},{med([x[9] for x in rr]):+.3f}]deg")

print("\n================ DECISION ================")
print("- GYRO-predicted gravity change ~= RAW gravity change, with small end error: supports a real rotation of the IMU/body between A and B.")
print("- GYRO-predicted change << RAW gravity change: raw accelerometer direction changed without corresponding rigid-body angular motion; suspect accel bias/misalignment/local sensor effect.")
print("- This still cannot distinguish whole-rig rotation from local FC/IMU flex if the IMU package itself physically rotates.")
