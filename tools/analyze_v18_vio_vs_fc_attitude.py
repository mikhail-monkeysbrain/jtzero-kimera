#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v18_legs.csv"
BACK=H+"/jtzero_500mm_v18_backend.csv"
ATT=H+"/jtzero_500mm_v18_attitude.csv"
OUT=H+"/jtzero_v18_vio_vs_fc_attitude.csv"
G=9.80665

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read(LEGS)
be=read(BACK)
att=read(ATT)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

def nearest(rows,key,t):
    return min(rows,key=lambda r:abs(iv(r,key)-t))

def wrap(d):
    while d>180:d-=360
    while d<-180:d+=360
    return d

rows=[]
print("================ V18 VIO vs FC ATTITUDE ================")
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    b0=nearest(be,"timestamp_ns",t0)
    b1=nearest(be,"timestamp_ns",t1)
    a0=nearest(att,"recv_ns",t0)
    a1=nearest(att,"recv_ns",t1)

    vio_dr=wrap(fv(b1,"roll_deg")-fv(b0,"roll_deg"))
    vio_dp=wrap(fv(b1,"pitch_deg")-fv(b0,"pitch_deg"))
    vio_dy=wrap(fv(b1,"yaw_deg")-fv(b0,"yaw_deg"))
    fc_dr=wrap(fv(a1,"roll_deg")-fv(a0,"roll_deg"))
    fc_dp=wrap(fv(a1,"pitch_deg")-fv(a0,"pitch_deg"))
    fc_dy=wrap(fv(a1,"yaw_deg")-fv(a0,"yaw_deg"))

    er=vio_dr-fc_dr
    ep=vio_dp-fc_dp
    ey=wrap(vio_dy-fc_dy)
    tilt_err_deg=math.hypot(er,ep)
    grav_leak=G*math.sin(math.radians(tilt_err_deg))
    dur=(t1-t0)/1e9
    xy=fv(L,"horizontal_m")*1000

    aw=[r for r in att if t0<=iv(r,"recv_ns")<=t1]
    br=[r for r in be if t0<=iv(r,"timestamp_ns")<=t1]
    # relative attitude changes vs each stream's own start
    max_fc_tilt=0.0; max_vio_tilt=0.0
    if aw:
      sr,sp=fv(a0,"roll_deg"),fv(a0,"pitch_deg")
      max_fc_tilt=max(math.hypot(wrap(fv(r,"roll_deg")-sr),wrap(fv(r,"pitch_deg")-sp)) for r in aw)
    if br:
      sr,sp=fv(b0,"roll_deg"),fv(b0,"pitch_deg")
      max_vio_tilt=max(math.hypot(wrap(fv(r,"roll_deg")-sr),wrap(fv(r,"pitch_deg")-sp)) for r in br)

    row=dict(
      leg=leg,direction=L["direction"],duration_s=dur,xy_mm=xy,scale=xy/500.0,
      vio_droll_deg=vio_dr,vio_dpitch_deg=vio_dp,vio_dyaw_deg=vio_dy,
      fc_droll_deg=fc_dr,fc_dpitch_deg=fc_dp,fc_dyaw_deg=fc_dy,
      residual_droll_deg=er,residual_dpitch_deg=ep,residual_dyaw_deg=ey,
      residual_tilt_deg=tilt_err_deg,equiv_gravity_leak_m_s2=grav_leak,
      max_fc_relative_tilt_deg=max_fc_tilt,max_vio_relative_tilt_deg=max_vio_tilt
    )
    rows.append(row)
    tag=" GOOD-REF" if leg==5 else (" BAD" if leg in (3,6) else "")
    print(f'LEG {leg} {L["direction"]}: XY={xy:.1f} scale={xy/500:.3f} dur={dur:.2f}s{tag}')
    print(f'  VIO dRPY=[{vio_dr:+.3f},{vio_dp:+.3f},{vio_dy:+.3f}] deg')
    print(f'  FC  dRPY=[{fc_dr:+.3f},{fc_dp:+.3f},{fc_dy:+.3f}] deg')
    print(f'  residual tilt=[{er:+.3f},{ep:+.3f}] norm={tilt_err_deg:.3f} deg '
          f'gravity_leak≈{grav_leak:.4f}m/s²')
    print(f'  max relative tilt FC/VIO={max_fc_tilt:.3f}/{max_vio_tilt:.3f} deg')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("\n================ LEG5 vs LEG6 ================")
r5=next(r for r in rows if r["leg"]==5)
r6=next(r for r in rows if r["leg"]==6)
for k in ["duration_s","scale","residual_droll_deg","residual_dpitch_deg",
          "residual_tilt_deg","equiv_gravity_leak_m_s2",
          "max_fc_relative_tilt_deg","max_vio_relative_tilt_deg"]:
    print(f'{k}: LEG5={r5[k]:.6f} LEG6={r6[k]:.6f} delta={r6[k]-r5[k]:+.6f}')

print("\nINTERPRETATION:")
print("- FC relative tilt small + VIO relative tilt large => attitude error is internal to VIO.")
print("- Residual roll/pitch is more meaningful than absolute RPY because FC and VIO frames may have fixed offsets.")
print("- Even ~1 deg tilt error corresponds to ~0.17 m/s^2 of gravity leakage, large compared with slow-motion acceleration.")
print("Saved:",OUT)
