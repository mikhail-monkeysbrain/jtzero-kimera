#!/usr/bin/env python3
import csv, math, re, sys
from pathlib import Path

LOG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log")
STATES = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("/home/vio/jtzero_zxy_replay_v11_CURRENT.csv")
COMBINED = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("/home/vio/jtzero_yaw_only_v15_42.csv")
OUTCSV = Path(sys.argv[4]) if len(sys.argv) > 4 else Path("/home/vio/p11_v15_54_pure_imu_forensic.csv")
OUTTXT = Path(sys.argv[5]) if len(sys.argv) > 5 else Path("/home/vio/p11_v15_54_pure_imu_forensic.txt")

for p in (LOG, STATES, COMBINED):
    if not p.exists():
        raise SystemExit(f"FATAL: missing {p}")

K_GYRO_CX = 0.014570
K_GYRO_CY = 0.082383
K_G = 9.81
K_KP = 0.35
K_ACC_TOL = 0.25
K_STATIC_GYRO = 0.035
K_STATIC_ACC_RES = 0.12
K_STATIC_HOLD = 0.25
K_MAX_CORR = 0.012
K_MAX_ERR = math.radians(5.0)
K_LP_TAU = 0.20

def vnorm(v):
    return math.sqrt(sum(x*x for x in v))

def vsub(a,b):
    return tuple(a[i]-b[i] for i in range(3))

def vcross(a,b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def vscale(v,s):
    return tuple(x*s for x in v)

def vadd(a,b):
    return tuple(a[i]+b[i] for i in range(3))

def vunit(v):
    n=vnorm(v)
    return tuple(x/n for x in v) if n>1e-12 else (0.0,0.0,0.0)

def rodrigues(v, axis, angle):
    # Rotate vector v around unit axis.
    c=math.cos(angle); s=math.sin(angle)
    axv=vcross(axis,v)
    dot=sum(axis[i]*v[i] for i in range(3))
    return tuple(v[i]*c + axv[i]*s + axis[i]*dot*(1-c) for i in range(3))

def parse_vec(line,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',line)
    return tuple(map(float,m.groups())) if m else None

def parse_scalar(line,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+)',line)
    return float(m.group(1)) if m else None

pre={}
post={}
lowdisp=set()
for line in LOG.read_text(errors="replace").splitlines():
    if "JTZERO_V15_53_PRE " in line:
        m=re.search(r'\bkf=(\d+)',line)
        if not m: continue
        k=int(m.group(1))
        pre[k]={
            "kf":k, "dt":parse_scalar(line,"dt"),
            "dp":parse_vec(line,"dp"), "dv":parse_vec(line,"dv"), "dr":parse_vec(line,"dr"),
            "predp":parse_vec(line,"predp"), "predv":parse_vec(line,"predv"),
            "ba":parse_vec(line,"ba"), "bg":parse_vec(line,"bg"),
        }
    elif "JTZERO_V15_53_POST " in line:
        m=re.search(r'\bkf=(\d+)',line)
        if not m: continue
        k=int(m.group(1))
        post[k]={"optp":parse_vec(line,"optp"),"optv":parse_vec(line,"optv"),
                 "ba":parse_vec(line,"ba"),"bg":parse_vec(line,"bg")}
    elif "JTZERO_V15_53_LOWDISP " in line:
        m=re.search(r'\bkf=(\d+)',line)
        if m: lowdisp.add(int(m.group(1)))

states={}
with STATES.open(newline="") as f:
    for r in csv.DictReader(f):
        k=int(r["keyframe"])
        states[k]={
            "ts":int(r["timestamp_ns"]),
            "p":(float(r["px_m"]),float(r["py_m"]),float(r["pz_m"])),
            "v":(float(r["vx_m_s"]),float(r["vy_m_s"]),float(r["vz_m_s"])),
            "rpy":(float(r["roll_deg"]),float(r["pitch_deg"]),float(r["yaw_deg"])),
        }

imu=[]
with COMBINED.open(newline="") as f:
    rr=csv.reader(f); hdr=next(rr,None)
    for c in rr:
        if len(c)<14 or c[0]!="IMU": continue
        imu.append({
            "source_ns":int(c[2]), "mapped_ns":int(c[3]),
            "acc_flu":(float(c[8]),-float(c[9]),-float(c[10])),
            "gyro_flu":(float(c[11]),-float(c[12]),-float(c[13])),
        })

# Reconstruct exact replay correction stream once.
last_us=0
static_time=0.0
init=False
lp_init=False
lp=(0.0,0.0,0.0)
gravity=(0.0,0.0,1.0)
for s in imu:
    us=s["source_ns"]//1000
    a=s["acc_flu"]; g=s["gyro_flu"]
    zxy=(g[0]+K_GYRO_CX*g[2], g[1]+K_GYRO_CY*g[2], g[2])
    dt=(us-last_us)*1e-6 if last_us and us>last_us else 0.0
    last_us=us
    corr=(0.0,0.0,0.0)
    if dt<=0 or dt>0.03:
        fed=zxy
    else:
        if not lp_init:
            lp=a; lp_init=True
        else:
            alpha=math.exp(-dt/K_LP_TAU)
            lp=tuple(alpha*lp[i]+(1-alpha)*a[i] for i in range(3))
        gravity_ok=abs(vnorm(a)-K_G)<=K_ACC_TOL
        gyro_quiet=vnorm(zxy)<=K_STATIC_GYRO
        accel_quiet=vnorm(vsub(a,lp))<=K_STATIC_ACC_RES
        static_sample=gravity_ok and gyro_quiet and accel_quiet
        static_time=static_time+dt if static_sample else 0.0
        confirmed=static_time>=K_STATIC_HOLD
        if not init:
            if confirmed:
                gravity=vunit(lp); init=True
            fed=zxy
        else:
            fed=zxy
            if confirmed:
                measured=vunit(lp)
                err=vcross(gravity,measured)
                if vnorm(err)<=math.sin(K_MAX_ERR):
                    corr=vscale(err,K_KP)
                    n=vnorm(corr)
                    if n>K_MAX_CORR: corr=vscale(corr,K_MAX_CORR/n)
                    fed=vsub(zxy,corr)
            theta=vscale(fed,-dt)
            ang=vnorm(theta)
            if ang>1e-12:
                gravity=vunit(rodrigues(gravity,vscale(theta,1.0/ang),ang))
    s["gyro_zxy"]=zxy
    s["gyro_fed"]=fed
    s["gravity_corr"]=corr
    s["dt"]=dt

# Sliding IMU index by state interval.
rows=[]
imu_i0=0
keys=sorted(k for k in pre if k in states)
for k in keys:
    prev_keys=[q for q in states if q<k]
    if not prev_keys: continue
    kp=max(prev_keys)
    t0=states[kp]["ts"]; t1=states[k]["ts"]
    while imu_i0<len(imu) and imu[imu_i0]["mapped_ns"]<t0: imu_i0+=1
    j=imu_i0
    seg=[]
    while j<len(imu) and imu[j]["mapped_ns"]<=t1:
        seg.append(imu[j]); j+=1
    p=pre[k]
    if not seg: continue
    def meanvec(name):
        n=len(seg)
        return tuple(sum(x[name][i] for x in seg)/n for i in range(3))
    accm=meanvec("acc_flu")
    fedm=meanvec("gyro_fed")
    corrm=meanvec("gravity_corr")
    max_corr=max(vnorm(x["gravity_corr"]) for x in seg)
    dt_sum=sum(x["dt"] for x in seg)
    yaw_int=sum(x["gyro_fed"][2]*x["dt"] for x in seg)
    dv=p["dv"]; dr=p["dr"]; predv=p["predv"]; predp=p["predp"]
    rows.append({
        "kf":k,"prev_kf":kp,"state_dt_s":(t1-t0)*1e-9,"pim_dt_s":p["dt"],
        "imu_n":len(seg),"imu_dt_sum_s":dt_sum,
        "pim_dv_x":dv[0],"pim_dv_y":dv[1],"pim_dv_z":dv[2],
        "pim_dv_xy":math.hypot(dv[0],dv[1]),"pim_dv_norm":vnorm(dv),
        "pim_dp_norm_mm":1000*vnorm(p["dp"]),
        "pim_dr_x_deg":math.degrees(dr[0]),"pim_dr_y_deg":math.degrees(dr[1]),"pim_dr_z_deg":math.degrees(dr[2]),
        "pim_dr_xy_deg":math.degrees(math.hypot(dr[0],dr[1])),"pim_dr_norm_deg":math.degrees(vnorm(dr)),
        "pred_v_xy":math.hypot(predv[0],predv[1]),"pred_v_norm":vnorm(predv),
        "pred_p_norm_m":vnorm(predp),
        "mean_acc_x":accm[0],"mean_acc_y":accm[1],"mean_acc_z":accm[2],"mean_acc_norm":vnorm(accm),
        "mean_fed_gx":fedm[0],"mean_fed_gy":fedm[1],"mean_fed_gz":fedm[2],
        "fed_yaw_integral_deg":math.degrees(yaw_int),
        "mean_gravity_corr_norm":vnorm(corrm),"max_gravity_corr_norm":max_corr,
        "lowdisp_event":1 if k in lowdisp else 0,
    })

if not rows:
    raise SystemExit("FATAL: no matched PRE/state/IMU intervals")

with OUTCSV.open("w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

top=sorted(rows,key=lambda r:r["pim_dv_xy"],reverse=True)[:10]
first_v05=next((r for r in rows if r["pred_v_xy"]>=0.5),None)
first_v10=next((r for r in rows if r["pred_v_xy"]>=1.0),None)
first_v20=next((r for r in rows if r["pred_v_xy"]>=2.0),None)

def fmtrow(r):
    return (f'KF={r["kf"]:3d} dt={r["pim_dt_s"]:.3f}s '
            f'dVxy={r["pim_dv_xy"]:.3f}m/s dV={r["pim_dv_norm"]:.3f} '
            f'dRxy={r["pim_dr_xy_deg"]:.3f}deg dRz={r["pim_dr_z_deg"]:+.3f}deg '
            f'predVxy={r["pred_v_xy"]:.3f}m/s '
            f'accMean=[{r["mean_acc_x"]:+.3f},{r["mean_acc_y"]:+.3f},{r["mean_acc_z"]:+.3f}] '
            f'corrMax={r["max_gravity_corr_norm"]:.5f} lowdisp={r["lowdisp_event"]}')

lines=[]
lines.append("================ P11 v15.54 PURE IMU FORENSIC ================")
lines.append(f"intervals={len(rows)}")
lines.append("")
lines.append("--- TOP 10 BY PIM dVxy ---")
lines.extend(fmtrow(r) for r in top)
lines.append("")
for name,r in [("FIRST_PRED_VXY_GE_0.5",first_v05),("FIRST_PRED_VXY_GE_1.0",first_v10),("FIRST_PRED_VXY_GE_2.0",first_v20)]:
    lines.append(name+"="+(fmtrow(r) if r else "NONE"))
lines.append("")
# Simple diagnostics.
maxr=top[0]
dt_bad=max(abs(r["pim_dt_s"]-r["state_dt_s"]) for r in rows if r["pim_dt_s"] is not None)
max_tilt=max(r["pim_dr_xy_deg"] for r in rows)
max_dv=max(r["pim_dv_xy"] for r in rows)
lines.append(f"MAX_PIM_DVXY={max_dv:.6f}")
lines.append(f"MAX_PIM_DRXY_DEG={max_tilt:.6f}")
lines.append(f"MAX_PIM_VS_STATE_DT_ERROR_S={dt_bad:.9f}")
lines.append(f"WORST_KF={maxr['kf']}")
if dt_bad>0.02:
    verdict="TIMING_INTERVAL_MISMATCH_SUSPECT"
elif max_tilt>2.0 and max_dv>0.5:
    verdict="PIM_TILT_GRAVITY_LEAKAGE_SUSPECT"
elif max_dv>0.5:
    verdict="PIM_LINEAR_ACCELERATION_OR_FRAME_SUSPECT"
else:
    verdict="NO_SINGLE_LARGE_PIM_INTERVAL_FOUND"
lines.append("V15_54_VERDICT="+verdict)
lines.append("RESULT: COMPLETE")
OUTTXT.write_text("\n".join(lines)+"\n")
print("\n".join(lines))
print(f"CSV: {OUTCSV}")
print(f"TXT: {OUTTXT}")
