#!/usr/bin/env python3
import csv, math, re
from pathlib import Path
CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv'); LOG=Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log')
CX,CY=.014570,.082383; G=9.81

def norm(v):return math.sqrt(sum(x*x for x in v))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def unit(v):
 n=norm(v);return tuple(x/n for x in v)
def angle(a,b):return math.degrees(math.acos(max(-1,min(1,dot(unit(a),unit(b))))))
def mean(vs):return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3)) if vs else (float('nan'),)*3
def vec(s,k):
 m=re.search(r'\b'+k+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',s);return tuple(map(float,m.groups())) if m else None
def sc(s,k):
 m=re.search(r'\b'+k+r'=([-+0-9.eE]+)',s);return float(m.group(1)) if m else float('nan')
def mm(A,B):return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A,v):return tuple(sum(A[i][j]*v[j] for j in range(3)) for i in range(3))
def tr(A):return [[A[j][i] for j in range(3)] for i in range(3)]
def I():return [[1.,0,0],[0,1.,0],[0,0,1.]]
def exp_so3(w):
 th=norm(w)
 if th<1e-12:return I()
 x,y,z=(q/th for q in w);c=math.cos(th);s=math.sin(th);C=1-c
 return [[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]]
def align(a,b):
 # R*a=b, shortest rotation.
 ua,ub=unit(a),unit(b);v=cross(ua,ub);s=norm(v);c=max(-1,min(1,dot(ua,ub)))
 if s<1e-12:return I()
 axis=tuple(x/s for x in v);return exp_so3(tuple(x*math.acos(c) for x in axis))

imu=[]
with CSV.open(errors='replace') as f:
 rd=csv.reader(f);next(rd,None)
 for c in rd:
  if len(c)<14 or c[0]!='IMU':continue
  try:t=int(c[3])*1e-9;a=(float(c[8]),-float(c[9]),-float(c[10]));g0=(float(c[11]),-float(c[12]),-float(c[13]))
  except:continue
  g=(g0[0]+CX*g0[2],g0[1]+CY*g0[2],g0[2]);imu.append((t,a,g))
t0=imu[0][0];imu=[(t-t0,a,g) for t,a,g in imu]
act=[i for i,x in enumerate(imu) if norm(x[2])>.10];ty0,ty1=imu[min(act)][0],imu[max(act)][0]
p=[];t=0
for line in LOG.open(errors='replace'):
 if 'JTZERO_V15_53_PRE ' not in line:continue
 q={'kf':int(sc(line,'kf')),'dt':sc(line,'dt'),'dr':vec(line,'dr'),'pv':vec(line,'predv'),'ba':vec(line,'ba'),'bg':vec(line,'bg')};t+=q['dt'];q['t']=t;p.append(q)
if not p:raise SystemExit('no PIM')
ba=p[0]['ba'];bg=p[0]['bg']
# Initial attitude exactly as Kimera gravity alignment: measured gravity=-mean accel -> global gravity -Z.
init=[x[1] for x in imu if x[0]<=min(2.0,ty0-.5)];ma0=mean(init);R=align(tuple(-x for x in ma0),(0,0,-G))
prevv=(0,0,0);prevt=0;rows=[]
for q in p:
 # Compose actual PIM deltaR into propagated body attitude.
 R=mm(R,exp_so3(q['dr']))
 z=[x for x in imu if prevt<x[0]<=q['t']];prevt=q['t'];ma=mean([x[1] for x in z])
 ap=tuple((q['pv'][i]-prevv[i])/q['dt'] for i in range(3));prevv=q['pv']
 # Gravity-direction mismatch between propagated attitude and contemporaneous accel direction.
 # At rest, R*unit(a_body) should be +Z (specific force); horizontal part predicts false acceleration.
 aw=mv(R,tuple(ma[i]-ba[i] for i in range(3)))
 amodel=(aw[0],aw[1],aw[2]-G)
 gdir=mv(R,unit(ma));tilterr=math.degrees(math.atan2(math.hypot(gdir[0],gdir[1]),gdir[2]))
 predh=(amodel[0],amodel[1]); obsh=(ap[0],ap[1]);
 magp=math.hypot(*predh);mago=math.hypot(*obsh)
 direrr=math.degrees(math.acos(max(-1,min(1,(predh[0]*obsh[0]+predh[1]*obsh[1])/(magp*mago))))) if magp*mago>1e-12 else float('nan')
 ph='PRE' if q['t']<ty0 else ('YAW' if q['t']<=ty1 else 'POST')
 rows.append((q,ph,tilterr,amodel,ap,magp,mago,direrr))

def avg(xs):return sum(xs)/len(xs) if xs else float('nan')
def report(name,z):
 print(f'{name}: N={len(z)} T={z[0][0]["t"]:.3f}..{z[-1][0]["t"]:.3f} TILTERR_MEAN={avg([r[2] for r in z]):.4f}deg TILTERR_END={z[-1][2]:.4f}deg PRED_AH_MEAN={avg([r[5] for r in z]):.5f} OBS_AH_MEAN={avg([r[6] for r in z]):.5f} RATIO={avg([r[5] for r in z])/avg([r[6] for r in z]):.4f} DIRERR_MEAN={avg([r[7] for r in z if math.isfinite(r[7])]):.3f}deg')
print('================ B11 ROTATION-GRAVITY CLOSURE AUDIT ================')
print(f'YAW_SEC=[{ty0:.3f},{ty1:.3f}] INITIAL_BA={ba} INITIAL_BG={bg}')
print(f'INITIAL_MEAN_ACCEL={ma0} INITIAL_TILT={angle(ma0,(0,0,1)):.6f}deg')
for ph in ('PRE','YAW','POST'):
 z=[r for r in rows if r[1]==ph];report(ph,z)
 if ph!='YAW':
  n=len(z)//3
  for tag,b in [('EARLY',z[:n]),('MID',z[n:2*n]),('LATE',z[2*n:])]:report(ph+'_'+tag,b)
print('\n--- POST SAMPLES ---')
z=[r for r in rows if r[1]=='POST'];step=max(1,len(z)//8)
for r in z[::step]:
 q,ph,te,am,ap,mp,mo,de=r;print(f'KF={q["kf"]} T={q["t"]:.3f} TILTERR={te:.4f}deg MODEL_Axy=[{am[0]:+.4f},{am[1]:+.4f}] {mp:.4f} OBS_Axy=[{ap[0]:+.4f},{ap[1]:+.4f}] {mo:.4f} DIRERR={de:.2f}deg')
# Closure score: magnitude ratio near 1 and direction error small means propagated R quantitatively explains Axy.
valid=[r for r in rows if r[1]=='POST' and math.isfinite(r[7])]
ratio=avg([r[5] for r in valid])/avg([r[6] for r in valid]);derr=avg([r[7] for r in valid])
print('\n--- SUMMARY ---')
print(f'POST_MODEL_OBS_AH_RATIO={ratio:.6f}')
print(f'POST_MODEL_OBS_DIRERR_MEAN_DEG={derr:.6f}')
if .75<=ratio<=1.25 and derr<25:ver='ROTATION_GRAVITY_PROJECTION_QUANTITATIVELY_CLOSES'
elif .5<=ratio<=1.5 and derr<45:ver='ROTATION_GRAVITY_PROJECTION_PARTIALLY_CLOSES'
else:ver='ROTATION_GRAVITY_PROJECTION_DOES_NOT_CLOSE'
print('B11_ROTATION_GRAVITY_VERDICT='+ver)
print('RESULT: COMPLETE')
