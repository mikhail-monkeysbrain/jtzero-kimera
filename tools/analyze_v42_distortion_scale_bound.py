#!/usr/bin/env python3
from pathlib import Path
import argparse, ast, math, re, statistics

p=argparse.ArgumentParser(description="V42 lens-distortion scale bound for raw-pixel camera-only estimator")
p.add_argument("--camera-yaml", default="params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003/LeftCameraParams.yaml")
p.add_argument("--camera-net-mm", type=float, default=566.61)
p.add_argument("--truth-mm", type=float, default=500.0)
p.add_argument("--used-height-mm", type=float, default=185.0)
p.add_argument("--direct-height-mm", type=float, default=180.0)
args=p.parse_args()

txt=Path(args.camera_yaml).read_text()

def parse_list(key):
    m=re.search(r"^\s*"+re.escape(key)+r"\s*:\s*\[([^\]]+)\]",txt,re.M)
    if not m: raise SystemExit(f"Не найден {key} в {args.camera_yaml}")
    return [float(x.strip()) for x in m.group(1).split(",")]

fx,fy,cx,cy=parse_list("intrinsics")
k1,k2,p1,p2,k3=parse_list("distortion_coefficients")
W,H=640,480

def distort(u,v):
    x=(u-cx)/fx; y=(v-cy)/fy
    r2=x*x+y*y
    radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)
    yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y
    return fx*xd+cx, fy*yd+cy

def sv2x2(a,b,c,d):
    # singular values of [[a,b],[c,d]]
    s1=a*a+b*b+c*c+d*d
    det=(a*d-b*c)**2
    disc=max(0.0,s1*s1-4*det)
    l1=(s1+math.sqrt(disc))/2
    l2=(s1-math.sqrt(disc))/2
    return math.sqrt(max(0,l1)),math.sqrt(max(0,l2))

vals=[]
horizontal=[]
vertical=[]
eps=1e-3
for j in range(41):
    v=(H-1)*j/40
    for i in range(55):
        u=(W-1)*i/54
        up=distort(u+eps,v); um=distort(u-eps,v)
        vp=distort(u,v+eps); vm=distort(u,v-eps)
        a=(up[0]-um[0])/(2*eps); c=(up[1]-um[1])/(2*eps)
        b=(vp[0]-vm[0])/(2*eps); d=(vp[1]-vm[1])/(2*eps)
        smax,smin=sv2x2(a,b,c,d)
        vals.extend([smax,smin])
        horizontal.append(math.hypot(a,c))
        vertical.append(math.hypot(b,d))

def pct(x,q):
    z=sorted(x)
    if not z:return float("nan")
    t=(len(z)-1)*q/100
    a=int(math.floor(t)); b=int(math.ceil(t))
    if a==b:return z[a]
    return z[a]*(b-t)+z[b]*(t-a)

required=args.camera_net_mm/args.truth_mm
direct_net=args.camera_net_mm*(args.direct_height_mm/args.used_height_mm)
required_after_height=direct_net/args.truth_mm

print("="*108)
print("V42 — LENS DISTORTION SCALE BOUND")
print("="*108)
print(f"camera YAML: {args.camera_yaml}")
print(f"intrinsics fx/fy = {fx:.3f} / {fy:.3f} px")
print(f"distortion k1/k2/p1/p2/k3 = {k1:+.6f} {k2:+.6f} {p1:+.6f} {p2:+.6f} {k3:+.6f}")
print()
print(f"CAMERA-ONLY / truth scale at h={args.used_height_mm:.1f} mm: {required:.4f}  ({(required-1)*100:+.2f}%)")
print(f"After replacing h with direct {args.direct_height_mm:.1f} mm:        {required_after_height:.4f}  ({(required_after_height-1)*100:+.2f}%)")
print()
print("LOCAL RAW-PIXEL DISTORTION JACOBIAN SCALE ACROSS 640x480")
print("-"*108)
print(f"all-direction singular scale: min={min(vals):.4f}  median={pct(vals,50):.4f}  p95={pct(vals,95):.4f}  max={max(vals):.4f}")
print(f"horizontal local scale:       min={min(horizontal):.4f}  median={pct(horizontal,50):.4f}  p95={pct(horizontal,95):.4f}  max={max(horizontal):.4f}")
print(f"vertical local scale:         min={min(vertical):.4f}  median={pct(vertical,50):.4f}  p95={pct(vertical,95):.4f}  max={max(vertical):.4f}")

max_excess=(max(vals)-1)*100
p95_excess=(pct(vals,95)-1)*100
need=(required_after_height-1)*100
print()
print("RECONCILIATION")
print("-"*108)
print(f"Residual camera-only excess after direct height: {need:+.2f}%")
print(f"95th-percentile local distortion excess:         {p95_excess:+.2f}%")
print(f"maximum sampled local distortion excess:         {max_excess:+.2f}%")
if need > max_excess + 0.5:
    verdict="DISTORTION ALONE TOO SMALL"
elif need > p95_excess + 1.0:
    verdict="DISTORTION COULD CONTRIBUTE, BUT FULL ERROR WOULD REQUIRE EDGE-DOMINATED TRACKS"
else:
    verdict="DISTORTION MAGNITUDE IS LARGE ENOUGH TO BE A PLAUSIBLE MAJOR CONTRIBUTOR"
print(f"VERDICT: {verdict}")
print()
print("INTERPRETATION")
print("-"*108)
print("1) V42 camera-only tracks raw distorted pixels but converts tx/ty to metres with pinhole fx/fy.")
print("2) This test computes the local pixel-motion scale introduced by the actual OV9281 distortion calibration.")
print("3) It is an upper-bound screen: it does NOT know where V42's tracked features were located.")
print("4) If the required +scale is near only the extreme field-of-view bound, the next physical test must log feature locations")
print("   and compare raw-pixel vs undistorted-pixel motion on the same single A→B pass.")
print("="*108)
