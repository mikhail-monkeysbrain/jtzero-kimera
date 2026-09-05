#!/usr/bin/env python3
import csv, glob, math, os, sys, statistics

G=9.81

def load(path):
    with open(path,newline="") as f:
        return list(csv.DictReader(f))

def F(x): return float(x)
def I(x): return int(float(x))

def n3(x,y,z): return math.sqrt(x*x+y*y+z*z)
def pearson(xs,ys):
    if len(xs)<3: return float("nan")
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    vx=sum((x-mx)**2 for x in xs); vy=sum((y-my)**2 for y in ys)
    if vx<=0 or vy<=0: return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys))/math.sqrt(vx*vy)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v21_*")))
rows=[]

for root in roots:
    evp=os.path.join(root,"jtzero_500mm_v21_events.csv")
    bep=os.path.join(root,"jtzero_500mm_v21_backend.csv")
    lep=os.path.join(root,"jtzero_500mm_v18_legs.csv")
    if not all(os.path.exists(p) for p in (evp,bep,lep)): continue

    events=load(evp); backend=load(bep); legs=load(lep)
    ends={I(e["leg"]):e for e in events if e["event"]=="END"}
    bykf={I(r["keyframe"]):r for r in backend}
    run=os.path.basename(root)

    for lr in legs:
        leg=I(lr["leg"]); e=ends.get(leg)
        if not e: continue
        k0=I(lr["end_press_kf"]); k1=I(lr["end_settled_kf"])
        r0=bykf.get(k0); r1=bykf.get(k1)
        if not r0 or not r1: continue
        ew=I(e["event_wall_ns"])
        seg=sorted([r for r in backend if k0<=I(r["keyframe"])<=k1], key=lambda r:I(r["keyframe"]))
        post=[r for r in seg if I(r["timestamp_ns"])>=ew]
        if len(post)<2: continue

        def v(r):
            return (F(r["vx_m_s"]),F(r["vy_m_s"]),F(r["vz_m_s"]))
        def ba(r):
            return (F(r["bax"]),F(r["bay"]),F(r["baz"]))
        def rpy(r):
            return (F(r["roll_deg"]),F(r["pitch_deg"]),F(r["yaw_deg"]))

        p0=(F(r0["px_m"]),F(r0["py_m"]),F(r0["pz_m"]))
        p1=(F(r1["px_m"]),F(r1["py_m"]),F(r1["pz_m"]))
        settle=n3(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2])*1000

        # Mean finite-difference acceleration across post-END backend states.
        acc=[]
        for a,b in zip(post[:-1],post[1:]):
            dt=(I(b["timestamp_ns"])-I(a["timestamp_ns"]))/1e9
            if dt<=0: continue
            va=v(a); vb=v(b)
            acc.append(((vb[0]-va[0])/dt,(vb[1]-va[1])/dt,(vb[2]-va[2])/dt))
        if not acc: continue
        mean_acc=tuple(sum(x[j] for x in acc)/len(acc) for j in range(3))
        rms_acc=math.sqrt(sum(n3(*x)**2 for x in acc)/len(acc))
        mean_acc_norm=n3(*mean_acc)

        b0=ba(r0); b1=ba(r1)
        dba=(b1[0]-b0[0],b1[1]-b0[1],b1[2]-b0[2])
        dba_norm=n3(*dba)

        rp0=rpy(r0); rp1=rpy(r1)
        dr=math.radians(rp1[0]-rp0[0]); dp=math.radians(rp1[1]-rp0[1])
        dtilt=math.sqrt(dr*dr+dp*dp)
        gravity_leak=G*dtilt

        # Peak velocity and duration.
        speeds=[n3(*v(r)) for r in post]
        peak_v=max(speeds)
        mean_v=sum(speeds)/len(speeds)
        dur=(I(post[-1]["timestamp_ns"])-I(post[0]["timestamp_ns"]))/1e9

        rec={
            "run":run,"leg":leg,"direction":lr["direction"],
            "xy_mm":F(lr["horizontal_m"])*1000,
            "settle_mm":settle,
            "post_duration_s":dur,
            "mean_post_v_mm_s":mean_v*1000,
            "peak_post_v_mm_s":peak_v*1000,
            "mean_dvdt_norm_m_s2":mean_acc_norm,
            "rms_dvdt_m_s2":rms_acc,
            "mean_dvdt_x":mean_acc[0],"mean_dvdt_y":mean_acc[1],"mean_dvdt_z":mean_acc[2],
            "dba_norm_m_s2":dba_norm,
            "dbax":dba[0],"dbay":dba[1],"dbaz":dba[2],
            "droll_deg":rp1[0]-rp0[0],"dpitch_deg":rp1[1]-rp0[1],
            "gravity_leak_from_delta_tilt_m_s2":gravity_leak,
        }
        rows.append(rec)

print("================ V21 FALSE VELOCITY SOURCE DIAGNOSTICS ================")
for r in rows:
    print(
        f"{r['run']} LEG {r['leg']} {r['direction']}: "
        f"settle={r['settle_mm']:.2f}mm meanV={r['mean_post_v_mm_s']:.1f} peakV={r['peak_post_v_mm_s']:.1f}mm/s "
        f"mean|dV/dt|={r['mean_dvdt_norm_m_s2']:.4f} rms|dV/dt|={r['rms_dvdt_m_s2']:.4f}m/s2 "
        f"dBA={r['dba_norm_m_s2']:.5f}m/s2 gravityLeak(dTilt)={r['gravity_leak_from_delta_tilt_m_s2']:.5f}m/s2"
    )

print("\n================ CORRELATIONS WITH SETTLE ================")
ys=[r["settle_mm"] for r in rows]
for m in [
    "mean_post_v_mm_s","peak_post_v_mm_s","post_duration_s",
    "mean_dvdt_norm_m_s2","rms_dvdt_m_s2",
    "dba_norm_m_s2","dbax","dbay","dbaz",
    "gravity_leak_from_delta_tilt_m_s2","droll_deg","dpitch_deg"
]:
    print(f"Pearson({m}, settle_mm)={pearson([r[m] for r in rows],ys):+.4f}")

print("\n================ CORRELATIONS WITH FALSE VELOCITY ================")
yv=[r["mean_post_v_mm_s"] for r in rows]
for m in [
    "mean_dvdt_norm_m_s2","rms_dvdt_m_s2",
    "dba_norm_m_s2","dbax","dbay","dbaz",
    "gravity_leak_from_delta_tilt_m_s2","droll_deg","dpitch_deg"
]:
    print(f"Pearson({m}, mean_post_v)={pearson([r[m] for r in rows],yv):+.4f}")

third=[r for r in rows if r["run"].startswith("20260905_155540")]
if third:
    print("\n================ RUN3 B->A CONTRAST ================")
    for leg in (2,6):
        r=next((x for x in third if x["leg"]==leg),None)
        if r:
            print(f"LEG{leg}: settle={r['settle_mm']:.2f}mm meanV={r['mean_post_v_mm_s']:.2f}mm/s "
                  f"mean|dV/dt|={r['mean_dvdt_norm_m_s2']:.5f} rms|dV/dt|={r['rms_dvdt_m_s2']:.5f} "
                  f"dBA={r['dba_norm_m_s2']:.5f} gravityLeak={r['gravity_leak_from_delta_tilt_m_s2']:.5f}")

out=os.path.expanduser("~/jtzero_v21_false_velocity_source.csv")
if rows:
    with open(out,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nSaved: {out}")
