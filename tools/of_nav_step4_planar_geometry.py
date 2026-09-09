#!/usr/bin/env python3
"""STEP 4/8 — exact planar ground-intersection estimator on archived flow.

This replaces the small-angle dx=h*du/f approximation with a geometric model:
- current calibrated K + radtan distortion,
- current camera->body rotation,
- FC attitude for each frame boundary,
- ray/ground-plane intersection at the measured camera height,
- one representative median-flow correspondence per forensic row.

No Kimera backend state and no empirical 500-mm scale coefficient are used.
"""
import argparse,csv,math,re,statistics
from pathlib import Path

def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def rx(a):
    c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def ry(a):
    c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def rz(a):
    c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def rpy(r,p,y): return mm(rz(y),mm(ry(p),rx(r)))

def unwrap_deg(v):
    if not v:return []
    out=[v[0]]
    for x in v[1:]:
        y=x
        while y-out[-1]>180:y-=360
        while y-out[-1]<-180:y+=360
        out.append(y)
    return out

def interp(xs,ys,x):
    if x<=xs[0]:return ys[0]
    if x>=xs[-1]:return ys[-1]
    lo,hi=0,len(xs)-1
    while hi-lo>1:
        m=(lo+hi)//2
        if xs[m]<=x:lo=m
        else:hi=m
    d=xs[hi]-xs[lo]; a=0.0 if d==0 else (x-xs[lo])/d
    return ys[lo]*(1-a)+ys[hi]*a

def read_camera(path):
    txt=path.read_text()
    mi=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt)
    md=re.search(r'distortion_coefficients:\s*\[([^\]]+)\]',txt)
    mt=re.search(r'T_BS:\s*\n\s*cols:\s*4\s*\n\s*rows:\s*4\s*\n\s*data:\s*\[([^\]]+)\]',txt,re.S)
    if not (mi and md and mt): raise RuntimeError("camera yaml parse failed")
    fx,fy,cx,cy=[float(x.strip()) for x in mi.group(1).split(",")[:4]]
    D=[float(x.strip()) for x in md.group(1).split(",")]
    vals=[float(x.strip()) for x in mt.group(1).replace("\n"," ").split(",")]
    RBC=[[vals[r*4+c] for c in range(3)] for r in range(3)]
    return fx,fy,cx,cy,D,RBC

def read_att(path):
    rows=list(csv.reader(path.open()))
    try: float(rows[0][0]); hdr=None; data=rows
    except: hdr=rows[0]; data=rows[1:]
    if hdr:
        idx={k:i for i,k in enumerate(hdr)}
        ti=idx.get("recv_ns",0);ri=idx.get("roll_deg",2);pi=idx.get("pitch_deg",3);yi=idx.get("yaw_deg",4)
    else: ti,ri,pi,yi=0,2,3,4
    T=[float(r[ti]) for r in data]
    RR=[float(r[ri]) for r in data]
    PP=[float(r[pi]) for r in data]
    YY=unwrap_deg([float(r[yi]) for r in data])
    return T,RR,PP,YY

def R_WB(t,T,RR,PP,YY):
    # FC source is FRD NED-like attitude; project convention uses FLU body.
    # Same sign mapping validated in prior V44 rotation discriminator.
    return rpy(math.radians(interp(T,RR,t)),
               math.radians(-interp(T,PP,t)),
               math.radians(-interp(T,YY,t)))

def undistort_norm(u,v,fx,fy,cx,cy,D,use_dist=True):
    xd=(u-cx)/fx; yd=(v-cy)/fy
    if not use_dist:return [xd,yd,1.0]
    k1,k2,p1,p2=(D+[0,0,0,0])[:4]
    k3=D[4] if len(D)>4 else 0.0
    x,y=xd,yd
    # fixed-point inverse of OpenCV radtan
    for _ in range(10):
        r2=x*x+y*y
        radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        dx=2*p1*x*y+p2*(r2+2*x*x)
        dy=p1*(r2+2*y*y)+2*p2*x*y
        if abs(radial)<1e-12:break
        x=(xd-dx)/radial
        y=(yd-dy)/radial
    return [x,y,1.0]

def ground_vector(ray_cam,RWC,height):
    rw=mv(RWC,ray_cam)
    if abs(rw[2])<1e-9:return None
    lam=-height/rw[2]
    if lam<=0:return None
    return [lam*rw[0],lam*rw[1],lam*rw[2]]

def estimate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,D,RBC,height_mode,use_dist):
    sx=sy=path=0.0; bad=0; steps=[]
    hs=[float(r["height_m"]) for r in fr]
    for i,row in enumerate(fr):
        h0=height_mode if isinstance(height_mode,float) else hs[i]
        h1=height_mode if isinstance(height_mode,float) else hs[min(i+1,len(hs)-1)]
        R0=mm(R_WB(bounds[i],T,RR,PP,YY),RBC)
        R1=mm(R_WB(bounds[i+1],T,RR,PP,YY),RBC)
        u0=float(row["c0x"]); v0=float(row["c0y"])
        u1=u0+float(row["med_flow_x_px"])
        v1=v0+float(row["med_flow_y_px"])
        q0=undistort_norm(u0,v0,fx,fy,cx,cy,D,use_dist)
        q1=undistort_norm(u1,v1,fx,fy,cx,cy,D,use_dist)
        g0=ground_vector(q0,R0,h0); g1=ground_vector(q1,R1,h1)
        if g0 is None or g1 is None:
            bad+=1;continue
        # Same world ground point: C0+g0=C1+g1 => C1-C0=g0-g1.
        dx=g0[0]-g1[0]; dy=g0[1]-g1[1]
        sx+=dx;sy+=dy;path+=math.hypot(dx,dy);steps.append(math.hypot(dx,dy))
    return dict(x=sx,y=sy,net=math.hypot(sx,sy),path=path,bad=bad,steps=steps)

def simple_flow(fr,fx,fy,h):
    sx=sy=0.0
    for r in fr:
        sx+=-float(r["med_flow_x_px"])*h/fx
        sy+=-float(r["med_flow_y_px"])*h/fy
    return math.hypot(sx,sy)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--physical-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    R=Path(a.run).expanduser(); cam=Path(a.camera_yaml)
    if not cam.is_absolute():cam=Path.cwd()/cam
    fx,fy,cx,cy,D,RBC=read_camera(cam)
    fr=list(csv.DictReader((R/"jtzero_v43_camera_forensic.csv").open()))
    ev=list(csv.DictReader((R/"jtzero_500mm_v25_events.csv").open()))
    T,RR,PP,YY=read_att(R/"jtzero_500mm_v25_attitude.csv")
    start,end=float(ev[0]["event_wall_ns"]),float(ev[1]["event_wall_ns"])
    N=len(fr);bounds=[start+(end-start)*i/N for i in range(N+1)]
    truth=a.truth_mm/1000.0; H=a.physical_height_mm/1000.0
    hs=[float(r["height_m"]) for r in fr]

    cases=[
      ("simple median-flow fixed H (no attitude)", None),
      ("plane rays fixed H, distortion OFF", estimate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,D,RBC,H,False)),
      ("plane rays fixed H, distortion ON", estimate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,D,RBC,H,True)),
      ("plane rays logged H, distortion OFF", estimate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,D,RBC,"logged",False)),
      ("plane rays logged H, distortion ON", estimate(fr,bounds,T,RR,PP,YY,fx,fy,cx,cy,D,RBC,"logged",True)),
    ]

    print("="*126)
    print("STEP 4/8 — HEIGHT SEMANTICS + EXACT PLANAR PROJECTION")
    print("="*126)
    print(f"run={R} rows={N} truth={a.truth_mm:.1f}mm physical camera-plane height={a.physical_height_mm:.1f}mm")
    print(f"logged H mean/median/min/max={statistics.mean(hs)*1000:.2f}/{statistics.median(hs)*1000:.2f}/{min(hs)*1000:.1f}/{max(hs)*1000:.1f}mm")
    print(f"K fx/fy={fx:.3f}/{fy:.3f} D={[round(x,6) for x in D]}")
    print()
    raw=simple_flow(fr,fx,fy,H)
    print(f"{cases[0][0]:45s}: net={raw*1000:8.2f}mm err={(raw-truth)*1000:+7.2f}mm scale={raw/truth:.5f}")
    for name,z in cases[1:]:
        print(f"{name:45s}: net={z['net']*1000:8.2f}mm err={(z['net']-truth)*1000:+7.2f}mm scale={z['net']/truth:.5f} "
              f"x={z['x']*1000:+8.2f} y={z['y']*1000:+8.2f} path={z['path']*1000:8.2f} bad={z['bad']}")

    # Bound what simple slant/tilt correction can possibly do.
    roll=[interp(T,RR,t) for t in bounds]
    pitch=[interp(T,PP,t) for t in bounds]
    tilt=[math.radians(math.hypot(r,p)) for r,p in zip(roll,pitch)]
    cmin=min(math.cos(x) for x in tilt); cmean=statistics.mean(math.cos(x) for x in tilt)
    print()
    print("HEIGHT-SEMANTICS BOUND")
    print("-"*126)
    print(f"FC body tilt magnitude max={max(math.degrees(x) for x in tilt):.3f}deg; mean cos(tilt)={cmean:.6f}, min cos={cmin:.6f}")
    print(f"If TF-Luna were a slant range aligned with body Z, multiplying by cos(tilt) could change scale by only {(1-cmean)*100:.3f}% on average.")
    print("That is far below a ~6% residual unless the sensor-to-plane geometry/height source itself is wrong.")

    best=min([(abs(raw-truth),cases[0][0],raw)]+[(abs(z['net']-truth),name,z['net']) for name,z in cases[1:]])
    print()
    print("STEP 4 DECISION")
    print("-"*126)
    print(f"best same-run geometric branch: {best[1]} -> {best[2]*1000:.2f}mm ({(best[2]/truth-1)*100:+.2f}%)")
    print("- If exact plane intersection + distortion materially reduces bias, that model becomes the Step-4 implementation candidate.")
    print("- If all exact-plane branches remain ~6% high, height/projection semantics are insufficient; Step 4 closes them and Step 5 must test repeatability before hardware decisions.")
    print("- No empirical endpoint scale correction is applied.")
    print("="*126)

if __name__=="__main__":
    main()
