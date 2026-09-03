#!/usr/bin/env python3
import csv,math
from pathlib import Path
P=Path('/home/vio/jtzero_yaw_only_v15_42.csv');CX,CY=.014570,.082383

def n(v):return math.sqrt(sum(x*x for x in v))
def mean(vs):return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3))
def sub(a,b):return tuple(a[i]-b[i] for i in range(3))
def sd(vs,m):return tuple(math.sqrt(sum((v[i]-m[i])**2 for v in vs)/max(1,len(vs)-1)) for i in range(3))
def fmt(v):return '['+','.join(f'{x:+.7f}' for x in v)+']'
a=[]
with P.open(errors='replace') as f:
 r=csv.reader(f);next(r,None)
 for c in r:
  if len(c)<14 or c[0]!='IMU':continue
  try:t=int(c[3])*1e-9;ac=(float(c[8]),-float(c[9]),-float(c[10]));g0=(float(c[11]),-float(c[12]),-float(c[13]))
  except:continue
  g=(g0[0]+CX*g0[2],g0[1]+CY*g0[2],g0[2]);a.append((t,ac,g))
t0=a[0][0];a=[(t-t0,x,g) for t,x,g in a];act=[i for i,x in enumerate(a) if n(x[2])>.1];ys,ye=a[min(act)][0],a[max(act)][0]
# Conservative quiet margins around yaw.
regions={'PRE_EARLY':(0,5),'PRE_LATE':(10,ys-.5),'POST_EARLY':(ye+.5,ye+4.5),'POST_LATE':(ye+7, min(a[-1][0],ye+12))}
print('================ B11 FINAL BIAS VALIDATION ================')
print(f'YAW_SEC=[{ys:.3f},{ye:.3f}]')
st={}
for name,(lo,hi) in regions.items():
 z=[x for x in a if lo<=x[0]<=hi];gs=[x[2] for x in z];acs=[x[1] for x in z];mg=mean(gs);sg=sd(gs,mg);ma=mean(acs);sa=sd(acs,ma);st[name]=(mg,sg,ma,sa,len(z))
 print(f'{name}: T={lo:.3f}..{hi:.3f} N={len(z)} GYRO_MEAN={fmt(mg)} GYRO_SD={fmt(sg)} ACC_MEAN={fmt(ma)} ACC_SD={fmt(sa)}')
print('\n--- STATIONARY GYRO MEAN CHANGES ---')
for x,y in [('PRE_EARLY','PRE_LATE'),('PRE_LATE','POST_EARLY'),('POST_EARLY','POST_LATE'),('PRE_EARLY','POST_LATE')]:
 d=sub(st[y][0],st[x][0]);print(f'{x}_TO_{y}={fmt(d)} NORM={n(d):.7f} rad/s ({math.degrees(n(d)):.4f} deg/s)')
# Estimate attitude error accumulated if PRE mean is treated as bias but POST mean persists.
post_bias=sub(st['POST_LATE'][0],st['PRE_EARLY'][0]);dur=regions['POST_LATE'][1]-ye
ang=tuple(math.degrees(x*dur) for x in post_bias);mag=math.sqrt(ang[0]**2+ang[1]**2)
print('\n--- COUNTERFACTUAL POST ATTITUDE ERROR FROM BIAS CHANGE ---')
print(f'POST_MINUS_PRE_GYRO={fmt(post_bias)}')
print(f'POST_DURATION_USED={dur:.3f}s')
print(f'PREDICTED_ROLL_PITCH_ERROR=[{ang[0]:+.4f},{ang[1]:+.4f}]deg MAG={mag:.4f}deg')
print(f'PREDICTED_GRAVITY_Axy={9.81*math.sin(math.radians(mag)):.6f} m/s2')
# Decide whether stationary mean changed materially relative to its own noise and is large enough to create >0.1 m/s2 gravity projection.
noise=math.hypot(st['POST_LATE'][1][0],st['POST_LATE'][1][1]);signal=math.hypot(post_bias[0],post_bias[1]);snr=signal/noise if noise else float('inf');pred=9.81*math.sin(math.radians(mag))
print('\n--- SUMMARY ---')
print(f'POST_BIAS_XY_CHANGE={signal:.7f} rad/s')
print(f'POST_GYRO_XY_SD_NORM={noise:.7f} rad/s')
print(f'BIAS_CHANGE_TO_SAMPLE_SD_RATIO={snr:.4f}')
print(f'PREDICTED_Axy_FROM_STATIONARY_BIAS_CHANGE={pred:.6f}')
if pred>=.15:ver='STATIONARY_GYRO_OFFSET_CHANGE_IS_LARGE_ENOUGH_TO_EXPLAIN_MAJOR_DRIFT'
elif pred>=.05:ver='STATIONARY_GYRO_OFFSET_CHANGE_CAN_EXPLAIN_PARTIAL_DRIFT'
else:ver='STATIONARY_GYRO_OFFSET_CHANGE_TOO_SMALL'
print('B11_FINAL_VERDICT='+ver)
print('NOTE=This establishes observability/scale, not whether the offset is FC sensor bias, preprocessing, temperature, or another upstream effect.')
print('RESULT: COMPLETE')
