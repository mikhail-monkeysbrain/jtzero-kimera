#!/usr/bin/env python3
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

def read_camera(path):
    txt=Path(path).read_text()
    mi=re.search(r'intrinsics:\s*\[([^\]]+)\]',txt)
    md=re.search(r'distortion_coefficients:\s*\[([^\]]+)\]',txt)
    mt=re.search(r'T_BS:\s*\n\s*cols:\s*4\s*\n\s*rows:\s*4\s*\n\s*data:\s*\[([^\]]+)\]',txt,re.S)
    if not (mi and md and mt): raise RuntimeError("camera yaml parse failed")
    fx,fy,cx,cy=[float(x.strip()) for x in mi.group(1).split(",")[:4]]
    D=[float(x.strip()) for x in md.group(1).split(",")]
    vals=[float(x.strip()) for x in mt.group(1).replace("\n"," ").split(",")]
    RBC=[[vals[r*4+c] for c in range(3)] for r in range(3)]
    return fx,fy,cx,cy,D,RBC

def undist(u,v,fx,fy,cx,cy,D):
    xd=(u-cx)/fx; yd=(v-cy)/fy
    k1,k2,p1,p2=(D+[0,0,0,0])[:4]; k3=D[4] if len(D)>4 else 0.0
    x,y=xd,yd
    for _ in range(12):
        r2=x*x+y*y
        radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        dx=2*p1*x*y+p2*(r2+2*x*x)
        dy=p1*(r2+2*y*y)+2*p2*x*y
        if abs(radial)<1e-12: break
        x=(xd-dx)/radial; y=(yd-dy)/radial
    return [x,y,1.0]

def load_att(path):
    rows=list(csv.DictReader(Path(path).open()))
    out=[]
    for r in rows:
        out.append((int(r["recv_mono_ns"]),float(r["roll"]),float(r["pitch"]),float(r["yaw"])))
    return out

def nearest_att(A,t):
    lo,hi=0,len(A)
    while lo<hi:
        m=(lo+hi)//2
        if A[m][0]<t: lo=m+1
        else: hi=m
    if lo==0:return A[0]
    if lo==len(A):return A[-1]
    a,b=A[lo-1],A[lo]
    return a if t-a[0]<=b[0]-t else b

def ground(ray,Rwc,h):
    rw=mv(Rwc,ray)
    if abs(rw[2])<1e-12:return None
    lam=-h/rw[2]
    if lam<=0:return None
    return [lam*rw[0],lam*rw[1],lam*rw[2]]

def med(v):
    return statistics.median(v) if v else float("nan")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    ap.add_argument("--camera-yaml",default="params/JTZeroMonoFLU/LeftCameraParams.yaml")
    ap.add_argument("--truth-mm",type=float,default=500.0)
    ap.add_argument("--fixed-height-mm",type=float,default=185.5)
    a=ap.parse_args()
    R=Path(a.run).expanduser()
    fx,fy,cx,cy,D,RBC=read_camera(a.camera_yaml)
    tracks=list(csv.DictReader((R/"of_replay"/"tracks.csv").open()))
    pairs=list(csv.DictReader((R/"of_replay"/"pairs.csv").open()))
    A=load_att(R/"attitude.csv")
    if not tracks or not pairs: raise RuntimeError("run OF replay first")

    by={}
    for r in tracks: by.setdefault(int(r["pair_id"]),[]).append(r)

    def integrate(mode,height_mm=None,fmult=1.0):
        sx=sy=path=0.0; used=0; bad=0
        for p in pairs:
            pid=int(p["pair_id"]); rows=by.get(pid,[])
            if len(rows)<20: continue
            t0=int(p["t0_ns"]); t1=int(p["t1_ns"])
            a0=nearest_att(A,t0); a1=nearest_att(A,t1)
            R0=mm(rpy(a0[1],a0[2],a0[3]),RBC)
            R1=mm(rpy(a1[1],a1[2],a1[3]),RBC)
            dxs=[]; dys=[]
            for r in rows:
                h=(height_mm if height_mm is not None else float(r["range_cm"])*10.0)/1000.0
                if mode=="small":
                    dx=-h*float(r["res_dx_px"])/(fx*fmult)
                    dy=-h*float(r["res_dy_px"])/(fy*fmult)
                elif mode=="raw":
                    dx=-h*(float(r["x1"])-float(r["x0"]))/(fx*fmult)
                    dy=-h*(float(r["y1"])-float(r["y0"]))/(fy*fmult)
                else:
                    q0=undist(float(r["x0"]),float(r["y0"]),fx*fmult,fy*fmult,cx,cy,D)
                    q1=undist(float(r["x1"]),float(r["y1"]),fx*fmult,fy*fmult,cx,cy,D)
                    g0=ground(q0,R0,h); g1=ground(q1,R1,h)
                    if g0 is None or g1 is None: bad+=1; continue
                    dx=g0[0]-g1[0]; dy=g0[1]-g1[1]
                dxs.append(dx); dys.append(dy)
            if len(dxs)<20: continue
            dx=statistics.median(dxs); dy=statistics.median(dys)
            sx+=dx;sy+=dy;path+=math.hypot(dx,dy);used+=1
        return dict(x=sx*1000,y=sy*1000,net=math.hypot(sx,sy)*1000,path=path*1000,pairs=used,bad=bad)

    cases=[
      ("CURRENT small-angle + logged Luna", integrate("small")),
      ("NO rotation compensation + logged Luna", integrate("raw")),
      (f"CURRENT small-angle + fixed {a.fixed_height_mm:.1f} mm", integrate("small",a.fixed_height_mm)),
      ("EXACT plane + logged Luna", integrate("plane")),
      (f"EXACT plane + fixed {a.fixed_height_mm:.1f} mm", integrate("plane",a.fixed_height_mm)),
    ]

    current=cases[0][1]["net"]
    required_h=a.fixed_height_mm*a.truth_mm/cases[2][1]["net"] if cases[2][1]["net"] else float("nan")
    required_fmult=current/a.truth_mm if a.truth_mm else float("nan")

    print("="*118)
    print("R1 6/6 — SAME-DATASET SCALE LOCALIZATION")
    print("="*118)
    print(f"run={R}")
    print(f"truth={a.truth_mm:.3f} mm  K fx/fy={fx:.6f}/{fy:.6f}  fixed-height probe={a.fixed_height_mm:.3f} mm")
    print("-"*118)
    for name,z in cases:
        print(f"{name:48s}: net={z['net']:9.3f} mm  err={z['net']-a.truth_mm:+8.3f}  "
              f"x={z['x']:+9.3f} y={z['y']:+9.3f} path={z['path']:9.3f} pairs={z['pairs']} bad={z['bad']}")
    print()
    print("DISCRIMINATORS")
    print("-"*118)
    print(f"required isotropic focal multiplier for CURRENT branch = {required_fmult:.6f}")
    print(f"equivalent fx/fy = {fx*required_fmult:.3f}/{fy*required_fmult:.3f}")
    print(f"height required for exact 500 mm within fixed-height small-angle branch = {required_h:.3f} mm")
    print(f"logged-Luna -> fixed-height change = {cases[2][1]['net']-current:+.3f} mm")
    print(f"small-angle -> exact-plane (logged height) change = {cases[3][1]['net']-current:+.3f} mm")
    print(f"rotation compensation effect = {cases[1][1]['net']-current:+.3f} mm")
    print()
    print("READING")
    print("-"*118)
    print("If exact-plane remains near the current result, small-angle projection is not the ~10% cause.")
    print("If fixed camera height removes only part of the error, TF-Luna/camera offset alone is insufficient.")
    print("If removing rotation compensation changes the endpoint strongly, attitude/extrinsic compensation remains a primary branch.")
    print("Do not apply the required focal multiplier; it is a diagnostic equivalent only.")
    print("="*118)

if __name__=="__main__": main()
