#!/usr/bin/env python3
import csv, math, re
from pathlib import Path

CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv')
LOG=Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log')
G=9.81


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
def parse_vec(line,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',line)
    return tuple(map(float,m.groups())) if m else None

# v15.42 combined recorder format is positional, not a flat DictReader IMU table.
# It is the same format already validated by analyze_b11_1_b11_2.py and replay v11:
#   c[0]  = "IMU"
#   c[3]  = mapped timestamp [ns]
#   c[8:11]  = accel in FC FRD
#   c[11:14] = gyro in FC FRD
# Kimera feed converts FRD -> FLU: [x,-y,-z].
data=[]
with CSV.open(errors='replace', newline='') as f:
    rd=csv.reader(f)
    next(rd,None)
    for c in rd:
        if len(c)<14 or c[0] != 'IMU':
            continue
        try:
            t=float(c[3])*1e-9
            ax,ay,az=map(float,c[8:11])
            gx,gy,gz=map(float,c[11:14])
        except Exception:
            continue
        data.append((t,(ax,-ay,-az),(gx,-gy,-gz)))

if len(data)<100:
    raise SystemExit(f'could not parse IMU rows: parsed={len(data)} expected>100')

t0=data[0][0]
data=[(t-t0,a,g) for t,a,g in data]

# Detect yaw so the PRE set matches the actual stationary initialization region,
# rather than assuming the first N seconds blindly.
CX,CY=0.014570,0.082383
def corrected_gyro(g): return (g[0]+CX*g[2], g[1]+CY*g[2], g[2])
active=[x[0] for x in data if norm(corrected_gyro(x[2]))>0.10]
if not active:
    raise SystemExit('could not detect yaw phase')
yaw_start=min(active)
pre=[x for x in data if x[0] < max(0.0,yaw_start-0.5)]
if len(pre)<100:
    raise SystemExit(f'too few pre-yaw rows: {len(pre)}')

# Read the actual initial bias from the true PURE_IMU replay instead of hardcoding it.
reported=None
if LOG.exists():
    for line in LOG.open(errors='replace'):
        if 'JTZERO_V15_53_PRE kf=1 ' in line:
            reported=parse_vec(line,'ba')
            break
if reported is None:
    reported=(-0.001237220,0.006167700,0.063696600)

print('================ B11.2 INITIALIZATION SENSITIVITY AUDIT ================')
print(f'CSV={CSV}')
print(f'IMU_ROWS={len(data)} PRE_ROWS={len(pre)} PRE_DURATION_SEC={pre[-1][0]-pre[0][0]:.3f}')
print(f'YAW_START_SEC={yaw_start:.3f}')
print('NOTE=Deterministic analytical audit of Kimera InitializationFromImu equations; no backend build.')

# Kimera InitializationFromImu:
# measured_gravity=-mean_acc
# R aligns measured gravity to [0,0,-g]
# local_gravity=R^T*g_world
# ba=mean_acc+local_gravity
# For a stationary sample, predicted world acceleration is R*(a-ba)+g_world.
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
    aw=[]
    for _,a,_ in pre:
        ac=tuple(a[i]-ba[i] for i in range(3))
        rw=mmul(R,ac)
        aw.append((rw[0],rw[1],rw[2]-gmag))
    m=mean(aw); rms=math.sqrt(sum(norm(v)**2 for v in aw)/len(aw)); mh=math.hypot(m[0],m[1])
    return len(w),ma,mg,ba,m,mh,rms,R

print('\n--- INIT WINDOW SWEEP ---')
print('WINDOW_SEC N BA_X BA_Y BA_Z RES_AX RES_AY RES_AZ RES_H RMS')
window_results=[]
for sec in (0.20,0.50,1.0,2.0,3.0,5.0,8.0,10.0,12.0,15.0):
    q=audit_window(sec)
    if not q: continue
    n,ma,mg,ba,m,mh,rms,R=q; window_results.append((sec,mh,rms,ba,m))
    print(f'{sec:.2f} {n} {ba[0]:+.6f} {ba[1]:+.6f} {ba[2]:+.6f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f}')

print('\n--- GRAVITY MAGNITUDE SWEEP (2s init) ---')
print('G RES_AX RES_AY RES_AZ RES_H RMS BA_X BA_Y BA_Z')
g_results=[]
for gm in (9.72,9.75,9.78,9.81,9.84,9.87,9.90,9.93):
    q=audit_window(2.0,gm); ba=q[3]; m=q[4]; mh=q[5]; rms=q[6]; g_results.append((gm,mh,rms,ba,m))
    print(f'{gm:.3f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f} {ba[0]:+.6f} {ba[1]:+.6f} {ba[2]:+.6f}')

print('\n--- INITIAL ROLL/PITCH PERTURBATION (2s init, g=9.81) ---')
print('DR_DEG DP_DEG RES_AX RES_AY RES_AZ RES_H RMS')
rp_results=[]
for dr,dp in ((0,0),(-1,0),(-.5,0),(-.25,0),(.25,0),(.5,0),(1,0),(0,-1),(0,-.5),(0,-.25),(0,.25),(0,.5),(0,1),(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)):
    q=audit_window(2.0,G,dr,dp); m=q[4]; mh=q[5]; rms=q[6]; rp_results.append((dr,dp,mh,rms,m))
    print(f'{dr:+.2f} {dp:+.2f} {m[0]:+.6f} {m[1]:+.6f} {m[2]:+.6f} {mh:.6f} {rms:.6f}')

print('\n--- REPORTED KIMERA INITIAL BIAS MATCH ---')
print(f'REPORTED_BA={reported[0]:+.9f},{reported[1]:+.9f},{reported[2]:+.9f}')
best=None
for sec in (0.20,0.50,1.0,2.0,3.0,5.0,8.0,10.0,12.0,15.0):
    q=audit_window(sec)
    if not q:continue
    ba=q[3]; err=norm(tuple(ba[i]-reported[i] for i in range(3)))
    if best is None or err<best[0]:best=(err,sec,ba)
print(f'BEST_MATCH_WINDOW_SEC={best[1]:.2f}')
print(f'BEST_MATCH_BA_ERROR={best[0]:.9f}')
print(f'BEST_MATCH_BA={best[2][0]:+.9f},{best[2][1]:+.9f},{best[2][2]:+.9f}')

# Also report the full pre-yaw mean. This is useful for separating init-window
# bias from slow temperature/mechanical changes during the later still period.
full_ma=mean([x[1] for x in pre])
print(f'FULL_PRE_MEAN_ACCEL_FLU={full_ma[0]:+.9f},{full_ma[1]:+.9f},{full_ma[2]:+.9f}')
print(f'FULL_PRE_MEAN_ACCEL_NORM={norm(full_ma):.9f}')

bw=min(window_results,key=lambda x:x[1]); bg=min(g_results,key=lambda x:x[1]); br=min(rp_results,key=lambda x:x[2])
base=[x for x in rp_results if x[0]==0 and x[1]==0][0][2]
print('\n--- SUMMARY ---')
print(f'BASE_2S_RES_H={base:.6f}')
print(f'BEST_WINDOW_BY_MEAN_HORIZONTAL_RESIDUAL_SEC={bw[0]:.2f} RES_H={bw[1]:.6f}')
print(f'BEST_GRAVITY_MAG_BY_MEAN_HORIZONTAL_RESIDUAL={bg[0]:.3f} RES_H={bg[1]:.6f}')
print(f'BEST_RP_BY_MEAN_HORIZONTAL_RESIDUAL=DR{br[0]:+.2f}_DP{br[1]:+.2f} RES_H={br[2]:.6f}')
if bw[1] < 0.35*base:
    verdict='INITIALIZATION_WINDOW_SENSITIVE'
elif br[2] < 0.35*base:
    verdict='SUBDEGREE_INITIAL_ATTITUDE_SENSITIVE'
elif bg[1] < 0.35*base:
    verdict='GRAVITY_MAGNITUDE_SENSITIVE'
else:
    verdict='INITIAL_ALIGNMENT_NOT_DOMINANT_IN_PRE_STILL_MEAN'
print('B11_2_ANALYTICAL_VERDICT='+verdict)
print('NEXT=Select at most 1-2 backend replay A/B modes from the analytical winner; do not tune production parameters from this sweep alone.')
print('RESULT: COMPLETE')
