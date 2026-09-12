#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,math,statistics
from pathlib import Path

CAM_Z=0.050
RNG_Z=0.071

def ff(r,k,d=float("nan")):
    try:return float(r.get(k,""))
    except:return d

def ii(r,k,d=0):
    try:return int(float(r.get(k,"")))
    except:return d

def q(xs,p):
    xs=sorted(x for x in xs if math.isfinite(x))
    if not xs:return float("nan")
    return xs[min(len(xs)-1,max(0,round((len(xs)-1)*p)))]

def summarize(name,rows):
    v=[r for r in rows if ii(r,"valid")==1 and ii(r,"flow_sent")==1 and 0<ff(r,"dt_s")<0.2]
    if not v:
        print(name+": no valid samples"); return None
    dt=[ff(r,"dt_s") for r in v]
    rng=[ff(r,"luna_m") for r in v]
    h=[x-(CAM_Z-RNG_Z) for x in rng]
    fx=[ff(r,"flow_send_x") for r in v]
    fy=[ff(r,"flow_send_y") for r in v]
    # legacy LOS native metric
    ix=sum(a*b*t for a,b,t in zip(fx,h,dt))
    iy=sum(a*b*t for a,b,t in zip(fy,h,dt))
    # pixel displacement integral, independent of range
    du=sum(ff(r,"du_px",0.0) for r in v)
    dv=sum(ff(r,"dv_px",0.0) for r in v)
    # fixed-height counterfactual using this leg's median h
    hm=statistics.median(h)
    ix_fix=sum(a*hm*t for a,t in zip(fx,dt))
    iy_fix=sum(a*hm*t for a,t in zip(fy,dt))
    inl=[ff(r,"inliers") for r in v]
    rat=[ff(r,"inlier_ratio") for r in v]
    gy=[math.hypot(ff(r,"fc_gyro_x",0),ff(r,"fc_gyro_y",0)) for r in v]
    rp=[(math.degrees(ff(r,"fc_roll",0)),math.degrees(ff(r,"fc_pitch",0))) for r in v]
    out=dict(n=len(v),time=sum(dt),range_med=statistics.median(rng),range_min=min(rng),range_max=max(rng),
             h_med=hm,los=math.hypot(ix,iy),fix=math.hypot(ix_fix,iy_fix),du=du,dv=dv,
             inl_med=statistics.median(inl),ratio_med=statistics.median(rat),gyro_p95=q(gy,.95),
             roll_span=max(x for x,_ in rp)-min(x for x,_ in rp),
             pitch_span=max(y for _,y in rp)-min(y for _,y in rp))
    print(f"{name}:")
    print(f"  samples={out['n']} dt_sum={out['time']:.3f} s")
    print(f"  Luna med/min/max={out['range_med']:.3f}/{out['range_min']:.3f}/{out['range_max']:.3f} m; camera_h_med={out['h_med']:.3f} m")
    print(f"  LOS metric={out['los']*1000:.1f} mm; fixed-height={out['fix']*1000:.1f} mm")
    print(f"  pixel sum du/dv={out['du']:+.1f}/{out['dv']:+.1f} px")
    print(f"  inliers median={out['inl_med']:.1f}; inlier_ratio median={out['ratio_med']:.3f}")
    print(f"  gyroXY p95={out['gyro_p95']:.4f} rad/s; roll/pitch span={out['roll_span']:.2f}/{out['pitch_span']:.2f} deg")
    return out

def main():
    ap=argparse.ArgumentParser(description="A/B directional asymmetry forensic")
    ap.add_argument("csv",type=Path)
    a=ap.parse_args()
    rows=list(csv.DictReader(a.csv.open(newline="")))
    events=[(j,ii(r,"return_event")) for j,r in enumerate(rows) if ii(r,"return_event") in (1,2,3)]
    ia=next((j for j,e in events if e==1),None)
    ib=next((j for j,e in events if e==2 and ia is not None and j>ia),None)
    ih=next((j for j,e in events if e==3 and ib is not None and j>ib),None)
    print("===== A/B НАПРАВЛЕННАЯ АСИММЕТРИЯ OPTICAL FLOW =====")
    print("events:",events)
    if ia is None or ib is None or ih is None:
        raise SystemExit("Не найдены A/B/H события")
    ab=summarize("A -> B",rows[ia:ib+1])
    ba=summarize("B -> A(H)",rows[ib:ih+1])
    if not ab or not ba:return
    print()
    print("СРАВНЕНИЕ:")
    print(f"  metric B/A = {ba['los']/ab['los']:.4f}")
    print(f"  fixed-height B/A = {ba['fix']/ab['fix']:.4f}")
    pixA=math.hypot(ab['du'],ab['dv']); pixB=math.hypot(ba['du'],ba['dv'])
    print(f"  pixel-integral B/A = {pixB/pixA:.4f}")
    print(f"  camera-height B/A = {ba['h_med']/ab['h_med']:.4f}")
    print()
    if abs(ba['fix']/ab['fix']-1)>0.05 and abs(pixB/pixA-1)>0.05:
        print("ВЫВОД: асимметрия уже есть в пиксельном/угловом optical flow; высота не является главным объяснением.")
    elif abs(ba['los']/ab['los']-1)>0.05 and abs(ba['fix']/ab['fix']-1)<0.03:
        print("ВЫВОД: основная асимметрия связана с высотой/range, а не с пиксельным flow.")
    else:
        print("ВЫВОД: причина смешанная или мала; сравните pixel-integral, fixed-height и attitude показатели.")

if __name__=="__main__":
    main()
