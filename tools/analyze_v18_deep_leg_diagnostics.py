#!/usr/bin/env python3
import csv, math, statistics
from collections import Counter

H="/home/vio"
LEGS=H+"/jtzero_500mm_v18_legs.csv"
BACK=H+"/jtzero_500mm_v18_backend.csv"
IMU=H+"/jtzero_500mm_v18.csv"
ATT=H+"/jtzero_500mm_v18_attitude.csv"
RNG=H+"/jtzero_500mm_v18_range.csv"
FRONT=H+"/jtzero_500mm_v18_frontend.csv"
OUT=H+"/jtzero_v18_leg_diagnostics_deep.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read(LEGS)
be=read(BACK)
imu=[r for r in read(IMU) if r.get("type")=="IMU"]
att=read(ATT)
rng=read(RNG)
front=read(FRONT)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

def vec_stats(rows, keys):
    if not rows: return None
    vals=[[fv(r,k) for r in rows] for k in keys]
    means=[statistics.mean(v) for v in vals]
    stds=[statistics.pstdev(v) if len(v)>1 else 0.0 for v in vals]
    return means,stds

def nearest_att(t):
    return min(att,key=lambda r:abs(iv(r,"recv_ns")-t)) if att else None

def pct(xs,p):
    if not xs:return float("nan")
    s=sorted(xs); q=(len(s)-1)*p/100; a=int(math.floor(q)); b=int(math.ceil(q))
    if a==b:return s[a]
    return s[a]*(b-q)+s[b]*(q-a)

rows=[]
print("================ V18 DEEP LEG DIAGNOSTICS ================")
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    dur=(t1-t0)*1e-9
    bw=[r for r in be if t0<=iv(r,"timestamp_ns")<=t1]
    iw=[r for r in imu if t0<=iv(r,"mapped_ns")<=t1]
    fw=[r for r in front if iv(r,"is_keyframe")!=0 and t0<=iv(r,"timestamp_ns")<=t1]
    rw=[r for r in rng if t0<=iv(r,"recv_ns")<=t1]

    # progress of VIO displacement from leg start, normalized by final endpoint direction
    p0=bw[0] if bw else None
    pend=bw[-1] if bw else None
    progress=[]
    if p0 and pend:
        vx=fv(pend,"px_m")-fv(p0,"px_m")
        vy=fv(pend,"py_m")-fv(p0,"py_m")
        norm=math.hypot(vx,vy)
        if norm>1e-9:
            ux,uy=vx/norm,vy/norm
            for r in bw:
                dx=fv(r,"px_m")-fv(p0,"px_m")
                dy=fv(r,"py_m")-fv(p0,"py_m")
                along=(dx*ux+dy*uy)*1000
                frac=(iv(r,"timestamp_ns")-t0)/(t1-t0) if t1>t0 else 0
                progress.append((frac,along))
    q25=min(progress,key=lambda q:abs(q[0]-0.25))[1] if progress else float("nan")
    q50=min(progress,key=lambda q:abs(q[0]-0.50))[1] if progress else float("nan")
    q75=min(progress,key=lambda q:abs(q[0]-0.75))[1] if progress else float("nan")

    ast=vec_stats(iw,["ax","ay","az"])
    gst=vec_stats(iw,["gx","gy","gz"])

    a0=nearest_att(t0); a1=nearest_att(t1)
    droll=dpitch=dyaw=float("nan")
    if a0 and a1:
        droll=fv(a1,"roll_deg")-fv(a0,"roll_deg")
        dpitch=fv(a1,"pitch_deg")-fv(a0,"pitch_deg")
        dyaw=fv(a1,"yaw_deg")-fv(a0,"yaw_deg")

    st=Counter(r["mono_status"] for r in fw)
    ratios=[fv(r,"mono_inlier_ratio") for r in fw if fv(r,"mono_putatives")>0]
    tracked=[fv(r,"tracked_features") for r in fw]
    low=st.get("LOW_DISPARITY",0)

    rr=[fv(r,"vertical_m") for r in rw if "vertical_m" in r]
    rmean=statistics.mean(rr) if rr else float("nan")
    rstd=statistics.pstdev(rr)*1000 if len(rr)>1 else float("nan")

    xy=fv(L,"horizontal_m")*1000
    row={
      "leg":leg,"direction":L["direction"],"duration_s":dur,"xy_mm":xy,"scale":xy/500.0,
      "p25_mm":q25,"p50_mm":q50,"p75_mm":q75,
      "acc_mean_x":ast[0][0] if ast else float("nan"),
      "acc_mean_y":ast[0][1] if ast else float("nan"),
      "acc_mean_z":ast[0][2] if ast else float("nan"),
      "acc_std_x":ast[1][0] if ast else float("nan"),
      "acc_std_y":ast[1][1] if ast else float("nan"),
      "acc_std_z":ast[1][2] if ast else float("nan"),
      "gyro_std_x":gst[1][0] if gst else float("nan"),
      "gyro_std_y":gst[1][1] if gst else float("nan"),
      "gyro_std_z":gst[1][2] if gst else float("nan"),
      "droll_deg":droll,"dpitch_deg":dpitch,"dyaw_deg":dyaw,
      "front_kf":len(fw),"low_frac":low/len(fw) if fw else float("nan"),
      "inlier_mean":statistics.mean(ratios) if ratios else float("nan"),
      "inlier_p10":pct(ratios,10),
      "tracked_mean":statistics.mean(tracked) if tracked else float("nan"),
      "range_mean_m":rmean,"range_std_mm":rstd,
    }
    rows.append(row)
    tag=" GOOD-REF" if leg==5 else (" BAD" if leg in (3,6) else "")
    print(f'LEG {leg} {L["direction"]}: dur={dur:.2f}s XY={xy:.1f} scale={xy/500:.3f}{tag}')
    print(f'  progress25/50/75={q25:.1f}/{q50:.1f}/{q75:.1f} mm')
    if ast:
        print(f'  accMean=[{ast[0][0]:+.4f},{ast[0][1]:+.4f},{ast[0][2]:+.4f}] '
              f'accStd=[{ast[1][0]:.4f},{ast[1][1]:.4f},{ast[1][2]:.4f}]')
        print(f'  gyroStd=[{gst[1][0]:.5f},{gst[1][1]:.5f},{gst[1][2]:.5f}]')
    print(f'  dRPY=[{droll:+.3f},{dpitch:+.3f},{dyaw:+.3f}] '
          f'LOW={100*row["low_frac"]:.1f}% inlierMean={row["inlier_mean"]:.3f} '
          f'p10={row["inlier_p10"]:.3f} tracked={row["tracked_mean"]:.1f} '
          f'range={rmean:.4f}m±{rstd:.2f}mm')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("\n================ BAD vs GOOD REF (LEG5) ================")
ref=next(r for r in rows if r["leg"]==5)
for leg in (3,6):
    r=next(x for x in rows if x["leg"]==leg)
    print(f'LEG{leg} minus LEG5:')
    for k in ["duration_s","scale","p25_mm","p50_mm","p75_mm",
              "acc_std_x","acc_std_y","acc_std_z",
              "gyro_std_x","gyro_std_y","gyro_std_z",
              "droll_deg","dpitch_deg","dyaw_deg",
              "low_frac","inlier_mean","tracked_mean","range_std_mm"]:
        print(f'  {k}: {r[k]-ref[k]:+.6f}')

print("\nINTERPRETATION HINTS:")
print("- If p25 is already too small, scale is wrong immediately after motion start.")
print("- If p25 is reasonable but p50/p75 collapse, error accumulates during cruise.")
print("- If only final XY is bad while p75 is good, braking/end-of-leg is suspect.")
print("- Compare LEG3/6 against LEG5: similar frontend but different IMU/attitude points to inertial scale/bias.")
print("Saved:",OUT)
