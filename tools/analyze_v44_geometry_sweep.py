#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

FX,FY=568.53170752165227,569.68005562865858
CX,CY=315.98271077441063,239.88148589100641
RBC=[[0.012724080,0.995473080,0.094188300],
     [0.998083560,-0.006939440,-0.061490300],
     [-0.060558320,0.094790200,-0.993653620]]

def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mt(A): return [list(x) for x in zip(*A)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def rx(a):
 c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def ry(a):
 c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def rz(a):
 c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def rpy(r,p,y): return mm(rz(y),mm(ry(p),rx(r)))
def interp(xs,ys,x):
 if x<=xs[0]: return ys[0]
 if x>=xs[-1]: return ys[-1]
 lo,hi=0,len(xs)-1
 while hi-lo>1:
  m=(lo+hi)//2
  if xs[m]<=x: lo=m
  else: hi=m
 t=(x-xs[lo])/(xs[hi]-xs[lo])
 return ys[lo]+t*(ys[hi]-ys[lo])
def unwrap(v):
 out=[v[0]]
 for x in v[1:]:
  while x-out[-1]>180:x-=360
  while x-out[-1]<-180:x+=360
  out.append(x)
 return out

p=argparse.ArgumentParser()
p.add_argument("--run",required=True);p.add_argument("--truth-mm",type=float,default=500.)
a=p.parse_args(); R=Path(a.run); truth=a.truth_mm/1000
fr=list(csv.DictReader((R/"jtzero_v43_camera_forensic.csv").open()))
ev=list(csv.DictReader((R/"jtzero_500mm_v25_events.csv").open()))
att=list(csv.reader((R/"jtzero_500mm_v25_attitude.csv").open()))
if not fr or len(ev)<2 or not att: raise SystemExit("missing run data")
try: int(att[0][0]); hdr=None; data=att
except ValueError: hdr=att[0]; data=att[1:]
if hdr:
 idx={k:i for i,k in enumerate(hdr)}; ti=idx.get("recv_ns",0);ri=idx.get("roll_deg",2);pi=idx.get("pitch_deg",3);yi=idx.get("yaw_deg",4)
else: ti,ri,pi,yi=0,2,3,4
T=[float(x[ti]) for x in data]; RR=[float(x[ri]) for x in data]; PP=[float(x[pi]) for x in data]; YY=unwrap([float(x[yi]) for x in data])
start,end=float(ev[0]["event_wall_ns"]),float(ev[1]["event_wall_ns"]); N=len(fr)
bounds=[start+(end-start)*i/N for i in range(N+1)]
obsx=sum(float(x["med_flow_x_px"]) for x in fr); obsy=sum(float(x["med_flow_y_px"]) for x in fr)
tx=sum(float(x["tx_px"]) for x in fr); ty=sum(float(x["ty_px"]) for x in fr)

# Enumerate convention/extrinsic alternatives as discriminators, not calibration candidates.
cases=[]
for ename,E in [("R_BC",RBC),("R_CB",mt(RBC))]:
 for pname,sp,sy in [("FRD->FLU", -1,-1),("raw_FC",1,1),("flip_pitch", -1,1),("flip_yaw",1,-1)]:
  rotx=roty=0.
  for i,row in enumerate(fr):
   def pose(t):
    rr=math.radians(interp(T,RR,t)); pp=math.radians(sp*interp(T,PP,t)); yy=math.radians(sy*interp(T,YY,t))
    return rpy(rr,pp,yy)
   A,B=pose(bounds[i]),pose(bounds[i+1])
   rel=mm(mt(E),mm(mt(B),mm(A,E)))
   u,v=float(row["c0x"]),float(row["c0y"]); q=mv(rel,[(u-CX)/FX,(v-CY)/FY,1])
   if abs(q[2])<1e-12: continue
   rotx+=FX*q[0]/q[2]+CX-u; roty+=FY*q[1]/q[2]+CY-v
  cx,cy=obsx-rotx,obsy-roty
  n=math.hypot(cx,cy)
  cases.append((ename,pname,rotx,roty,cx,cy,n))

print("="*118);print("V44 — GEOMETRY / CONVENTION / HEIGHT / FOCAL SWEEP (SAME PHYSICAL RUN)");print("="*118)
print(f"run={R} rows={N} truth={a.truth_mm:.1f} mm")
print(f"affine sum=({tx:+.3f},{ty:+.3f}) norm={math.hypot(tx,ty):.3f} px")
print(f"median-flow sum=({obsx:+.3f},{obsy:+.3f}) norm={math.hypot(obsx,obsy):.3f} px")
print("\nROTATION CONVENTION SWEEP")
for en,pn,rxs,rys,cx,cy,n in cases:
 raw180=math.hypot(obsx*.180/FX,obsy*.180/FY)
 cor180=math.hypot(cx*.180/FX,cy*.180/FY)
 print(f"{en:4s} {pn:11s}: rot=({rxs:+7.2f},{rys:+7.2f})px corrected=({cx:+8.2f},{cy:+8.2f})px h180={cor180*1000:7.2f}mm residual={(cor180/truth-1)*100:+6.2f}% explained={(raw180-cor180)*1000:+6.2f}mm")

# The project convention used by the prior analyzer is the primary branch.
primary=next(x for x in cases if x[0]=="R_BC" and x[1]=="FRD->FLU")
cx,cy=primary[4],primary[5]
print("\nPRIMARY R_BC + FRD->FLU: HEIGHT/FOCAL RECONCILIATION")
for h in (.175,.180,.185,.190,.195):
 dx=cx*h/FX; dy=cy*h/FY; net=math.hypot(dx,dy); k=net/truth
 reqfx=FX*k; reqfy=FY*k
 print(f"h={h*1000:3.0f}mm net={net*1000:7.2f}mm residual={(k-1)*100:+6.2f}% required isotropic focal k={k:.5f} fx={reqfx:.2f} fy={reqfy:.2f}")
reqh=truth/math.hypot(cx/FX,cy/FY)
print(f"required height with calibrated focal: {reqh*1000:.2f} mm")
# Axis-wise truth decomposition cannot know true X/Y endpoint components; report measured metric components and focal sensitivity.
mx,my=cx*.180/FX,cy*.180/FY
print(f"at h=180mm corrected components: x={mx*1000:+.2f}mm y={my*1000:+.2f}mm net={math.hypot(mx,my)*1000:.2f}mm")
print("\nDECISION")
print("- Convention variants are sensitivity checks. Do not select a variant merely because it lands near 500 mm.")
print("- If only implausible convention/extrinsic variants close the residual, reject them as calibration fixes.")
print("- If the independently measured optical-center height is near the required height, height geometry is sufficient.")
print("- Otherwise the remaining isotropic focal factor is the effective-geometry target to verify independently (runtime crop/resize/intrinsics), not a value to tune blindly.")
print("="*118)
