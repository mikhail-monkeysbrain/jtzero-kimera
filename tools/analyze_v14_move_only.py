#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
LEGS=H+"/jtzero_500mm_v13_legs.csv"
BACKEND=H+"/jtzero_500mm_v13_backend.csv"
CAM=H+"/jtzero_500mm_v13_camera.csv"
OUT=H+"/jtzero_v14_move_only_diagnostics.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read(LEGS); be=read(BACKEND); cam=read(CAM)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}
be=sorted(be,key=lambda r:iv(r,"timestamp_ns"))
cam=sorted(cam,key=lambda r:iv(r,"corrected_timestamp_ns"))

def nearest_backend(t):
    return min(be,key=lambda r:abs(iv(r,"timestamp_ns")-t))

rows=[]
for L in legs:
    leg=iv(L,"leg")
    sk=iv(L,"start_settled_kf"); ek=iv(L,"end_press_kf")
    t0,t1=kf_ts[sk],kf_ts[ek]
    bw=[r for r in be if t0<=iv(r,"timestamp_ns")<=t1]
    cw=[r for r in cam if t0<=iv(r,"corrected_timestamp_ns")<=t1]
    sel=[r for r in cw if iv(r,"selected")!=0]
    ts=[iv(r,"corrected_timestamp_ns") for r in sel]
    gaps=[((b-a)*1e-6,a,b) for a,b in zip(ts,ts[1:]) if b>a]
    maxgap=max(gaps,key=lambda q:q[0]) if gaps else (float("nan"),0,0)

    path=0.; max_step=0.; max_step_speed=0.; prev=None
    for r in bw:
        if prev is not None:
            dx=fv(r,"px_m")-fv(prev,"px_m");dy=fv(r,"py_m")-fv(prev,"py_m");dz=fv(r,"pz_m")-fv(prev,"pz_m")
            dp=math.sqrt(dx*dx+dy*dy+dz*dz)
            dt=(iv(r,"timestamp_ns")-iv(prev,"timestamp_ns"))*1e-9
            path+=dp; max_step=max(max_step,dp)
            if dt>0:max_step_speed=max(max_step_speed,dp/dt)
        prev=r

    gdisp=gspeed=gfrac=float("nan"); gphase=""
    if maxgap[1]:
        a=nearest_backend(maxgap[1]); b=nearest_backend(maxgap[2])
        dx=fv(b,"px_m")-fv(a,"px_m");dy=fv(b,"py_m")-fv(a,"py_m");dz=fv(b,"pz_m")-fv(a,"pz_m")
        gdisp=1000*math.sqrt(dx*dx+dy*dy+dz*dz)
        dt=(iv(b,"timestamp_ns")-iv(a,"timestamp_ns"))*1e-9
        gspeed=gdisp/dt if dt>0 else float("nan")
        gfrac=(maxgap[1]-t0)/(t1-t0) if t1>t0 else float("nan")
        gphase=f'{gfrac*100:.1f}%'

    duration=(t1-t0)*1e-9
    xy=fv(L,"horizontal_m")*1000
    rows.append(dict(
      leg=leg,direction=L["direction"],xy_mm=xy,error_mm=xy-500,
      move_duration_s=duration,selected_frames=len(sel),selected_fps=len(sel)/duration if duration>0 else 0,
      max_selected_gap_ms=maxgap[0],gap_at_move_percent=gphase,
      backend_disp_during_gap_mm=gdisp,backend_speed_during_gap_mm_s=gspeed,
      backend_path3d_mm=path*1000,path_to_endpoint_ratio=path/(fv(L,"distance3d_m")) if fv(L,"distance3d_m")>0 else float("nan"),
      max_backend_step_mm=max_step*1000,max_backend_step_speed_mm_s=max_step_speed*1000
    ))

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V14 MOVE-ONLY DIAGNOSTICS ================")
for r in rows:
    mark="  <-- OUTLIER" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: XY={r["xy_mm"]:.2f} err={r["error_mm"]:+.2f} '
          f'move={r["move_duration_s"]:.2f}s sel={r["selected_frames"]} fps={r["selected_fps"]:.2f} '
          f'maxGap={r["max_selected_gap_ms"]:.2f}ms at={r["gap_at_move_percent"]} '
          f'dPgap={r["backend_disp_during_gap_mm"]:.1f}mm Vgap={r["backend_speed_during_gap_mm_s"]:.1f}mm/s '
          f'path={r["backend_path3d_mm"]:.1f}mm ratio={r["path_to_endpoint_ratio"]:.3f} '
          f'maxStep={r["max_backend_step_mm"]:.1f}mm stepV={r["max_backend_step_speed_mm_s"]:.1f}mm/s{mark}')

print("\n================ B->A COMPARISON ================")
for r in rows:
    if r["leg"] in (2,4,6):
        print(f'LEG {r["leg"]}: fps={r["selected_fps"]:.2f} gap={r["max_selected_gap_ms"]:.2f}ms '
              f'gap_pos={r["gap_at_move_percent"]} dPgap={r["backend_disp_during_gap_mm"]:.1f}mm '
              f'path={r["backend_path3d_mm"]:.1f}mm ratio={r["path_to_endpoint_ratio"]:.3f}')
print("Saved:",OUT)
