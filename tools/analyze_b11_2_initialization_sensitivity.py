#!/usr/bin/env python3
import csv, math, statistics
from pathlib import Path

CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv')
G=9.81

with CSV.open(newline='') as f:
    rows=list(csv.DictReader(f))
if not rows: raise SystemExit('empty CSV')

def pick(r,names):
    for n in names:
        if n in r and r[n] not in ('',None): return float(r[n])
    raise KeyError(names)
def tsec(r):
    for n in ('source_ns','timestamp_ns','ts_ns','mapped_ns'):
        if n in r and r[n]: return float(r[n])*1e-9
    for n in ('time_usec','time_us'):
        if n in r and r[n]: return float(r[n])*1e-6
    raise KeyError('time')

def acc(r):
    # HIGHRES_IMU source is FRD; Kimera feed is FLU = [x,-y,-z].
    x=pick(r,('xacc','accel_x','ax')); y=pick(r,('yacc','accel_y','ay')); z=pick(r,('zacc','accel_z','az'))
    return (x,-y,-z)
def gyr(r):
    x=pick(r,('xgyro','gyro_x','gx')); y=pick(r,('ygyro','gyro_y','gy')); z=pick(r,('zgyro','gyro_z','gz'))
    return (x,-y,-z)

def norm(v): return math.sqrt(sum(x*x for x in v))
def mean(vs): return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def unit(v):
    n=norm(v); return tuple(x/n for x in v)
def mmul(A,v): return tuple(sum(A[i][j]*v[j] for j in range(3)) for i in range(3))
def transpose(A): return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))
def rodrigues(axis,ang):
    x,y,z=unit(axis); c=math.cos(ang); s=math.sin(ang); C=1-c
    return ((c+x*x*C,x*y*C-z*s,x*z*C+y*s),(y*x*C+z*s,c+y*y*C,y*z*C-x*s),(z*x*C-y*s,z*y*C+x*s,c+z*z*C))
def align(a,b):
    ua,ub=unit(a),unit(b); c=max(-1,min(1,dot(ua,ub))); ax=cross(ua,ub); s=norm(ax)
    if s<1e-12: return ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
    return rodrigues(ax,math.atan2(s,c))
def rxy(roll,pitch):
    cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch)
    Rx=((1,0,0),(0,cr,-sr),(0,sr,cr)); Ry=((cp,0,sp),(0,1,0),(-sp,0,cp))
    return tuple(tuple(sum(Ry[i][k]*Rx[k][j] for k in range(3)) for j in range(3)) for i in range(3))

data=[]
for r in rows:
    try:data.append((tsec(r),acc(r),gyr(r)))
    except Exception: pass
if len(data)<100: raise SystemExit('could not parse IMU rows')
t0=data[0][0]
# Same broad pre-still region as B11.1; sensitivity windows are prefixes of it.
pre=[x for x in data if x[0]-t0 <= 15.0]
if len(pre)<100: pre=data[:min(len(data),3000)]

print('================ B11.2 INITIALIZATION SENSITIVITY AUDIT ================')
print(f'CSV={CSV}')
print(f'IMU_ROWS={len(data)} PRE_ROWS={len(pre)} PRE_DURATION_SEC={pre[-1][0]-pre[0][0]:.3f}')
print('NOTE=This is a deterministic analytical audit of Kimera InitializationFromImu equations; no backend build.')

# Kimera InitializationFromImu: measured_gravity=-mean_acc, align to [0,0,-g],
# local_gravity=R^T*g_world, ba=mean_acc+local_gravity.
def audit_window(sec,gmag=G,dr=0.0,dp=0.0):
    w=[x for x in pre if x[0]-pre[0][0] <= sec]
    if len(w)<10:return None
    ma=mean([x[1] for x in w]); mg=mean([x[2] for x in w])
    R=align(tuple(-x for x in ma),(0,0,-gmag))
    if dr or dp:
        P=rxy(math.radians(dr),math.radians(dp))
        R=tuple(tuple(sum(P[i][k]*R[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    lg=mmul(transpose(R),(0,0,-gmag))
    ba=tuple(ma[i]+lg[i] for i in range(3))
    # Residual world acceleration for all PRE samples after bias correction.
    aw=[]
    for _,a,_ in pre:
        ac=tuple(a[i]-ba[i] for i in range(3))
        rw=mmul(R,ac)
        aw.append((rw[0],rw[1],rw[2]-gmag)) # specific force rotated + gravity [0,0,-g]
    m=mean(aw); rms=math.sqrt(sum(norm(v)**2 for v in aw)/len(aw))
    mh=math.hypot(m[0],m[1])
    tilt=math.degrees(math.acos(max(-1,min(1,ma[2]/norm(ma)))))
    return len(w),ma,mg,ba,m,mh,rms,tilt

print('\n--- INIT WINDOW SWEEP ---')
print('WINDOW_SEC N BA_X BA_Y BA_Z RES_AX RES_AY RES_AZ RES_H RMS')
window_results=[]
for sec in (0.20,0.50,1.0,2.0,3.0,5.0,8.0,10.0,15.0):
    q=audit_window(sec)
    if not q: continue
    n,ma,mg,ba,m,mh,rms,tilt=q; window_results.append((sec,mh,rms))
    print(f'{sec:.2f} {n} {ba[0]:+.6f} {ba[1]:+.6f} {ba[2]:+.6f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f}')

print('\n--- GRAVITY MAGNITUDE SWEEP (2s init) ---')
print('G RES_AX RES_AY RES_AZ RES_H RMS')
g_results=[]
for gm in (9.75,9.78,9.81,9.84,9.87,9.90):
    q=audit_window(2.0,gm); m=q[4]; mh=q[5]; rms=q[6]; g_results.append((gm,mh,rms))
    print(f'{gm:.3f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f}')

print('\n--- INITIAL ROLL/PITCH PERTURBATION (2s init, g=9.81) ---')
print('DR_DEG DP_DEG RES_AX RES_AY RES_AZ RES_H RMS')
rp_results=[]
for dr,dp in ((0,0),(-1,0),(-.5,0),(-.25,0),(.25,0),(.5,0),(1,0),(0,-1),(0,-.5),(0,-.25),(0,.25),(0,.5),(0,1)):
    q=audit_window(2.0,G,dr,dp); m=q[4]; mh=q[5]; rms=q[6]; rp_results.append((dr,dp,mh,rms))
    print(f'{dr:+.2f} {dp:+.2f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f}')

# Current Kimera-reported bias from v15.53, compare with analytical windows.
reported=(-0.001237220,0.006167700,0.063696600)
print('\n--- REPORTED KIMERA INITIAL BIAS MATCH ---')
print(f'REPORTED_BA={reported[0]:+.9f},{reported[1]:+.9f},{reported[2]:+.9f}')
best=None
for sec in (0.20,0.50,1.0,2.0,3.0,5.0,8.0,10.0,15.0):
    q=audit_window(sec)
    if not q:continue
    ba=q[3]; err=norm(tuple(ba[i]-reported[i] for i in range(3)))
    if best is None or err<best[0]:best=(err,sec,ba)
print(f'BEST_MATCH_WINDOW_SEC={best[1]:.2f}')
print(f'BEST_MATCH_BA_ERROR={best[0]:.9f}')
print(f'BEST_MATCH_BA={best[2][0]:+.9f},{best[2][1]:+.9f},{best[2][2]:+.9f}')

bw=min(window_results,key=lambda x:x[1]); bg=min(g_results,key=lambda x:x[1]); br=min(rp_results,key=lambda x:x[2])
print('\n--- SUMMARY ---')
print(f'BEST_WINDOW_BY_MEAN_HORIZONTAL_RESIDUAL_SEC={bw[0]:.2f} RES_H={bw[1]:.6f}')
print(f'BEST_GRAVITY_MAG_BY_MEAN_HORIZONTAL_RESIDUAL={bg[0]:.3f} RES_H={bg[1]:.6f}')
print(f'BEST_RP_BY_MEAN_HORIZONTAL_RESIDUAL=DR{br[0]:+.2f}_DP{br[1]:+.2f} RES_H={br[2]:.6f}')
base=[x for x in rp_results if x[0]==0 and x[1]==0][0][2]
if bw[1] < 0.35*base:
    verdict='INITIALIZATION_WINDOW_SENSITIVE'
elif br[2] < 0.35*base:
    verdict='SUBDEGREE_INITIAL_ATTITUDE_SENSITIVE'
else:
    verdict='INITIAL_ALIGNMENT_NOT_DOMINANT_IN_PRE_STILL_MEAN'
print('B11_2_ANALYTICAL_VERDICT='+verdict)
print('NEXT=Use result to select a small backend replay A/B; do not tune production parameters from this analytical sweep alone.')
print('RESULT: COMPLETE')
