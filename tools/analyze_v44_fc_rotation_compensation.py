#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path

FX=568.53170752165227; FY=569.68005562865858
CX=315.98271077441063; CY=239.88148589100641
# Active V43 camera-to-body rotation R_BC from LeftCameraParams.yaml.
RBC=[
 [0.012724080, 0.995473080, 0.094188300],
 [0.998083560,-0.006939440,-0.061490300],
 [-0.060558320,0.094790200,-0.993653620],
]

def mm(A,B):
 return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
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
def unwrap_deg(v):
 out=[v[0]]
 for x in v[1:]:
  y=x
  while y-out[-1]>180:y-=360
  while y-out[-1]<-180:y+=360
  out.append(y)
 return out

p=argparse.ArgumentParser(description="V44 rotation-compensated camera-only estimate from same run")
p.add_argument("--run",required=True);p.add_argument("--truth-mm",type=float,default=500.0)
a=p.parse_args();R=Path(a.run)
forensic=list(csv.DictReader((R/"jtzero_v43_camera_forensic.csv").open()))
events=list(csv.DictReader((R/"jtzero_500mm_v25_events.csv").open()))
att=list(csv.reader((R/"jtzero_500mm_v25_attitude.csv").open()))
if not forensic or len(events)<2 or not att: raise SystemExit("missing run data")

# attitude CSV may have a header or be raw positional rows:
header=None
try:
 int(att[0][0])
 data=att
except:
 header=att[0]; data=att[1:]
if header:
 idx={k:i for i,k in enumerate(header)}
 ti=idx.get("recv_ns",0);ri=idx.get("roll_deg",2);pi=idx.get("pitch_deg",3);yi=idx.get("yaw_deg",4)
else:
 ti,ri,pi,yi=0,2,3,4
T=[float(r[ti]) for r in data]; RR=[float(r[ri]) for r in data]; PP=[float(r[pi]) for r in data]; YY=unwrap_deg([float(r[yi]) for r in data])

start=float(events[0]["event_wall_ns"]); end=float(events[1]["event_wall_ns"])
N=len(forensic)
bounds=[start+(end-start)*i/N for i in range(N+1)]

obsx=obsy=rotx=roty=corrx=corry=0.0
rot_norms=[]
for i,row in enumerate(forensic):
 # FC ATTITUDE is FRD; map relative attitude into project FLU: roll same, pitch/yaw sign flip.
 def pose(t):
  rr=math.radians(interp(T,RR,t)); pp=math.radians(-interp(T,PP,t)); yy=math.radians(-interp(T,YY,t))
  return rpy(rr,pp,yy)
 Rw0=pose(bounds[i]); Rw1=pose(bounds[i+1])
 # coordinates of a static ray in C1 from its C0 coordinates.
 Rrel=mm(mt(RBC),mm(mt(Rw1),mm(Rw0,RBC)))
 u=float(row["c0x"]);v=float(row["c0y"])
 ray=[(u-CX)/FX,(v-CY)/FY,1.0]
 q=mv(Rrel,ray)
 if abs(q[2])<1e-9: continue
 ur=FX*q[0]/q[2]+CX; vr=FY*q[1]/q[2]+CY
 rfx=ur-u; rfy=vr-v
 ofx=float(row["med_flow_x_px"]); ofy=float(row["med_flow_y_px"])
 cfx=ofx-rfx; cfy=ofy-rfy
 obsx+=ofx;obsy+=ofy;rotx+=rfx;roty+=rfy;corrx+=cfx;corry+=cfy
 rot_norms.append(math.hypot(rfx,rfy))

print("="*108);print("V44 — FC-ATTITUDE ROTATION COMPENSATION ON REAL CAMERA FLOW");print("="*108)
print(f"run={R} rows={N} duration={(end-start)/1e9:.3f}s")
print(f"observed median-flow sum:       ({obsx:+.3f},{obsy:+.3f}) px norm={math.hypot(obsx,obsy):.3f}")
print(f"predicted rotation-only sum:    ({rotx:+.3f},{roty:+.3f}) px norm={math.hypot(rotx,roty):.3f}")
print(f"rotation-compensated flow sum:  ({corrx:+.3f},{corry:+.3f}) px norm={math.hypot(corrx,corry):.3f}")
print(f"rotation per-step mean/max norm: {sum(rot_norms)/len(rot_norms):.4f}/{max(rot_norms):.4f} px")
truth=a.truth_mm/1000
for h in (0.180,0.185):
 raw=math.hypot(obsx*h/FX,obsy*h/FY)
 cor=math.hypot(corrx*h/FX,corry*h/FY)
 print(f"h={h*1000:.0f} mm: raw={raw*1000:.2f} mm ({(raw/truth-1)*100:+.2f}%), "
       f"rot-corrected={cor*1000:.2f} mm ({(cor/truth-1)*100:+.2f}%)")
print("\nINTERPRETATION")
print("- If rotation correction removes most of the +5..8%, camera-only bias is primarily physical attitude rotation being counted as translation.")
print("- If the result barely changes, physical rotation cannot explain the residual scale and effective geometry/focal-height remains primary.")
print("- This is an offline discriminator using the SAME physical run; no new A->B is required.")
print("="*108)
