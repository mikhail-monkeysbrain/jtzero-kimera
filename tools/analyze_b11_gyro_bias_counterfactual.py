#!/usr/bin/env python3
import csv,math,re
from pathlib import Path
CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv');LOG=Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log');CX,CY=.014570,.082383;G=9.81

def n(v):return math.sqrt(sum(x*x for x in v))
def mean(vs):return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3))
def cross(a,b):return(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def unit(v):q=n(v);return tuple(x/q for x in v)
def mm(A,B):return[[sum(A[i][k]*B[k][j] for k in range(3))for j in range(3)]for i in range(3)]
def mv(A,v):return tuple(sum(A[i][j]*v[j] for j in range(3))for i in range(3))
def exp(w):
 t=n(w)
 if t<1e-12:return[[1.,0,0],[0,1.,0],[0,0,1.]]
 x,y,z=(q/t for q in w);c=math.cos(t);s=math.sin(t);C=1-c
 return[[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]]
def align(a,b):
 a,b=unit(a),unit(b);v=cross(a,b);s=n(v);c=max(-1,min(1,dot(a,b)))
 if s<1e-12:return[[1.,0,0],[0,1.,0],[0,0,1.]]
 ax=tuple(x/s*math.acos(c) for x in v);return exp(ax)
def vec(s,k):
 m=re.search(r'\b'+k+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',s);return tuple(map(float,m.groups()))if m else None
def sc(s,k):
 m=re.search(r'\b'+k+r'=([-+0-9.eE]+)',s);return float(m.group(1))if m else float('nan')
imu=[]
with CSV.open(errors='replace')as f:
 r=csv.reader(f);next(r,None)
 for c in r:
  if len(c)<14 or c[0]!='IMU':continue
  try:t=int(c[3])*1e-9;a=(float(c[8]),-float(c[9]),-float(c[10]));g0=(float(c[11]),-float(c[12]),-float(c[13]))
  except:continue
  g=(g0[0]+CX*g0[2],g0[1]+CY*g0[2],g0[2]);imu.append((t,a,g))
t0=imu[0][0];imu=[(t-t0,a,g)for t,a,g in imu];act=[i for i,x in enumerate(imu)if n(x[2])>.1];ys,ye=imu[min(act)][0],imu[max(act)][0]
p=[];t=0
for l in LOG.open(errors='replace'):
 if 'JTZERO_V15_53_PRE 'not in l:continue
 q={'kf':int(sc(l,'kf')),'dt':sc(l,'dt'),'dr':vec(l,'dr'),'pv':vec(l,'predv'),'ba':vec(l,'ba'),'bg':vec(l,'bg')};t+=q['dt'];q['t']=t;p.append(q)
ba,bg=p[0]['ba'],p[0]['bg'];ma0=mean([x[1]for x in imu if x[0]<=2]);R0=align(tuple(-x for x in ma0),(0,0,-G))
# Counterfactual corrections to gyro bias. Includes measured PRE/POST stationary residual and axis sweeps.
pre_g=mean([x[2]for x in imu if x[0]<ys]);post_g=mean([x[2]for x in imu if x[0]>ye])
cands=[('CURRENT',(0,0,0)),('REMOVE_PRE_MEAN',pre_g),('REMOVE_POST_MEAN',post_g),('REMOVE_PREPOST_AVG',tuple((pre_g[i]+post_g[i])/2 for i in range(3)))]
for d in (-.003,-.002,-.001, .001,.002,.003):
 for i,ax in enumerate('XYZ'):
  v=[0.,0.,0.];v[i]=d;cands.append((f'DBG_{ax}_{d:+.3f}',tuple(v)))

def run(extra):
 R=[row[:]for row in R0];prev=0.;til=[];ah=[]
 for q in p:
  z=[x for x in imu if prev<x[0]<=q['t']];prev=q['t'];mg=mean([x[2]for x in z]);ma=mean([x[1]for x in z])
  # Integrate corrected raw gyro directly; extra is additional bias removal in corrected FLU gyro.
  w=tuple((mg[i]-bg[i]-extra[i])*q['dt'] for i in range(3));R=mm(R,exp(w));aw=mv(R,tuple(ma[i]-ba[i]for i in range(3)));A=(aw[0],aw[1],aw[2]-G)
  if q['t']>ye:
   gd=mv(R,unit(ma));til.append(math.degrees(math.atan2(math.hypot(gd[0],gd[1]),gd[2])));ah.append(math.hypot(A[0],A[1]))
 return sum(til)/len(til),til[-1],sum(ah)/len(ah),ah[-1]
res=[]
for name,x in cands:res.append((name,x,*run(x)))
res.sort(key=lambda r:r[4])
print('================ B11 GYRO-BIAS COUNTERFACTUAL ================')
print(f'YAW_SEC=[{ys:.3f},{ye:.3f}] REPORTED_BG={bg}')
print(f'PRE_CORRECTED_GYRO_MEAN={pre_g}')
print(f'POST_CORRECTED_GYRO_MEAN={post_g}')
print('NOTE=Analytical counterfactual only; no production parameter change.')
print('\n--- BEST BY POST MEAN MODELED Axy ---')
for r in res[:12]:print(f'{r[0]} EXTRA=[{r[1][0]:+.6f},{r[1][1]:+.6f},{r[1][2]:+.6f}] POST_TILT_MEAN={r[2]:.4f} END={r[3]:.4f}deg POST_AH_MEAN={r[4]:.5f} END={r[5]:.5f}')
cur=next(r for r in res if r[0]=='CURRENT');best=res[0]
print('\n--- SUMMARY ---')
print(f'CURRENT_POST_AH_MEAN={cur[4]:.6f}')
print(f'BEST_MODE={best[0]} BEST_POST_AH_MEAN={best[4]:.6f} IMPROVEMENT={(1-best[4]/cur[4])*100:.2f}%')
if best[4]<cur[4]*.35:ver='GYRO_BIAS_CAN_EXPLAIN_MOST_ROTATION_GRAVITY_ERROR'
elif best[4]<cur[4]*.70:ver='GYRO_BIAS_EXPLAINS_SUBSTANTIAL_PART'
else:ver='GYRO_BIAS_ALONE_INSUFFICIENT'
print('B11_GYRO_BIAS_VERDICT='+ver);print('RESULT: COMPLETE')
