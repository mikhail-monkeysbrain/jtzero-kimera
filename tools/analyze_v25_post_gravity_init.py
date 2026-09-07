#!/usr/bin/env python3
import csv, math, bisect
from pathlib import Path

H=Path("/home/vio")
LEGS=H/"jtzero_500mm_v25_legs.csv"
BACK=H/"jtzero_500mm_v25_backend.csv"
ATT=H/"jtzero_500mm_v25_attitude.csv"
OUT=H/"jtzero_v25_post_gravity_init.csv"
G=9.80665

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def wrap(d):
    while d>180: d-=360
    while d<-180: d+=360
    return d
def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return [x-y for x,y in zip(a,b)]

legs=load(LEGS); be=load(BACK); att=load(ATT)
be.sort(key=lambda r:I(r,"keyframe"))
att.sort(key=lambda r:I(r,"recv_ns"))
att_ts=[I(r,"recv_ns") for r in att]
bykf={I(r,"keyframe"):r for r in be}

def nearest_att(wall_ns):
    j=bisect.bisect_left(att_ts,wall_ns)
    cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(att): cand.append(att[k])
    return min(cand,key=lambda r:abs(I(r,"recv_ns")-wall_ns)) if cand else None

print("================ V25 POST-GRAVITY-INIT FORENSIC ================")
rows=[]
for L in legs:
    leg=I(L,"leg")
    ks=I(L,"start_settled_kf")
    ke=I(L,"end_press_kf")
    s=bykf.get(ks); e=bykf.get(ke)
    if not s or not e:
        print(f"LEG{leg}: missing backend state KF {ks}->{ke}")
        continue

    a0=nearest_att(I(s,"callback_wall_ns"))
    a1=nearest_att(I(e,"callback_wall_ns"))
    if not a0 or not a1:
        print(f"LEG{leg}: missing FC attitude match")
        continue

    vio_dr=wrap(F(e,"roll_deg")-F(s,"roll_deg"))
    vio_dp=wrap(F(e,"pitch_deg")-F(s,"pitch_deg"))
    vio_dy=wrap(F(e,"yaw_deg")-F(s,"yaw_deg"))
    fc_dr=wrap(F(a1,"roll_deg")-F(a0,"roll_deg"))
    fc_dp=wrap(F(a1,"pitch_deg")-F(a0,"pitch_deg"))
    fc_dy=wrap(F(a1,"yaw_deg")-F(a0,"yaw_deg"))
    er=vio_dr-fc_dr
    ep=vio_dp-fc_dp
    ey=wrap(vio_dy-fc_dy)
    tilt_err=math.hypot(er,ep)
    grav_leak=G*math.sin(math.radians(tilt_err))

    ba0=[F(s,k) for k in ("bax","bay","baz")]
    ba1=[F(e,k) for k in ("bax","bay","baz")]
    bg0=[F(s,k) for k in ("bgx","bgy","bgz")]
    bg1=[F(e,k) for k in ("bgx","bgy","bgz")]
    dba=sub(ba1,ba0); dbg=sub(bg1,bg0)

    xy=F(L,"horizontal_m")*1000.0
    dz=F(L,"dz_m")*1000.0 if "dz_m" in L else (F(e,"pz_m")-F(s,"pz_m"))*1000.0
    scale=xy/500.0

    row=dict(
        leg=leg,direction=L["direction"],xy_mm=xy,dz_mm=dz,scale=scale,
        vio_droll_deg=vio_dr,vio_dpitch_deg=vio_dp,vio_dyaw_deg=vio_dy,
        fc_droll_deg=fc_dr,fc_dpitch_deg=fc_dp,fc_dyaw_deg=fc_dy,
        residual_droll_deg=er,residual_dpitch_deg=ep,residual_dyaw_deg=ey,
        residual_tilt_deg=tilt_err,equiv_gravity_leak_m_s2=grav_leak,
        start_bax=ba0[0],start_bay=ba0[1],start_baz=ba0[2],
        end_bax=ba1[0],end_bay=ba1[1],end_baz=ba1[2],
        delta_bax=dba[0],delta_bay=dba[1],delta_baz=dba[2],delta_ba_norm=norm(dba),
        delta_bg_norm=norm(dbg),
        fc_match_start_ms=(I(a0,"recv_ns")-I(s,"callback_wall_ns"))/1e6,
        fc_match_end_ms=(I(a1,"recv_ns")-I(e,"callback_wall_ns"))/1e6,
    )
    rows.append(row)

    print(f"LEG {leg} {L['direction']}: XY={xy:.2f} mm scale={scale:.4f} dz={dz:+.2f} mm")
    print(f"  VIO dRPY=[{vio_dr:+.3f},{vio_dp:+.3f},{vio_dy:+.3f}] deg")
    print(f"  FC  dRPY=[{fc_dr:+.3f},{fc_dp:+.3f},{fc_dy:+.3f}] deg")
    print(f"  residual dRP=[{er:+.3f},{ep:+.3f}] tilt={tilt_err:.3f} deg "
          f"gravity_leak≈{grav_leak:.4f} m/s^2")
    print(f"  BA start=[{ba0[0]:+.5f},{ba0[1]:+.5f},{ba0[2]:+.5f}] "
          f"end=[{ba1[0]:+.5f},{ba1[1]:+.5f},{ba1[2]:+.5f}] "
          f"|dBA|={norm(dba):.5f}")
    print(f"  FC match start/end={row['fc_match_start_ms']:+.1f}/{row['fc_match_end_ms']:+.1f} ms")

if not rows:
    raise SystemExit("No analyzable legs")

print("\n================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["direction"]==d]
    if not rr: continue
    avg=lambda k:sum(r[k] for r in rr)/len(rr)
    print(f"{d}: XY={avg('xy_mm'):.2f} mm scale={avg('scale'):.4f} "
          f"dz={avg('dz_mm'):+.2f} mm residualTilt={avg('residual_tilt_deg'):.3f} deg "
          f"gravityLeak≈{avg('equiv_gravity_leak_m_s2'):.4f} m/s^2 "
          f"|dBA|={avg('delta_ba_norm'):.5f}")

with OUT.open("w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)

print("\nINTERPRETATION:")
print("- FC nearly stationary while VIO R/P changes => remaining attitude error is internal to VIO, not physical stand tilt.")
print("- Large residual tilt correlated with dz/scale => prioritize attitude/bias coupling or camera-body geometry.")
print("- BA returning to large XY values after exact init => Hypothesis 1B gains support.")
print("- BA remains small but VIO attitude still moves => Hypothesis 2/visual geometry gains priority.")
print("Saved:",OUT)
