#!/usr/bin/env python3
import csv, math, re
from pathlib import Path

CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv')
LOG=Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log')
CX,CY=0.014570,0.082383

def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def pct(xs,p):
    if not xs:return math.nan
    s=sorted(xs); k=(len(s)-1)*p; i=int(k); f=k-i
    return s[i]*(1-f)+s[min(i+1,len(s)-1)]*f

def vec(s,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',s)
    return tuple(map(float,m.groups())) if m else None
def scalar(s,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+)',s)
    return float(m.group(1)) if m else math.nan

# Raw replay IMU, same parsing/corrections as prior B11 audits.
imu=[]
with CSV.open(errors='replace') as f:
    rd=csv.reader(f); next(rd,None)
    for c in rd:
        if len(c)<14 or c[0]!='IMU': continue
        try:
            t=int(c[3])*1e-9
            ax,ay,az=map(float,c[8:11]); gx,gy,gz=map(float,c[11:14])
        except: continue
        a=(ax,-ay,-az); gr=(gx,-gy,-gz)
        gf=(gr[0]+CX*gr[2],gr[1]+CY*gr[2],gr[2])
        imu.append((t,a,gf))
if len(imu)<100: raise SystemExit('B11 ERROR: too few IMU rows')
t0=imu[0][0]; imu=[(t-t0,a,g) for t,a,g in imu]
active=[i for i,x in enumerate(imu) if norm(x[2])>0.10]
if not active: raise SystemExit('B11 ERROR: yaw not found')
ty0,ty1=imu[min(active)][0],imu[max(active)][0]

pre=[]; post=[]
for line in LOG.open(errors='replace'):
    if 'JTZERO_V15_53_PRE ' in line:
        pre.append(dict(kf=int(scalar(line,'kf')),dt=scalar(line,'dt'),dp=vec(line,'dp'),dv=vec(line,'dv'),dr=vec(line,'dr'),predp=vec(line,'predp'),predv=vec(line,'predv'),ba=vec(line,'ba'),bg=vec(line,'bg')))
    elif 'JTZERO_V15_53_POST ' in line:
        post.append(dict(kf=int(scalar(line,'kf')),optp=vec(line,'optp'),optv=vec(line,'optv')))
if len(pre)<100: raise SystemExit('B11 ERROR: too few PIM rows')

# Map KF intervals to elapsed time using cumulative PIM dt. First PRE is first propagation after initialization.
t=0.0
for q in pre:
    t += q['dt']; q['t']=t
p0=pre[0]['predp']; v0=(0.,0.,0.)

def phase(t):
    # CSV and PIM elapsed clocks both start at recording/pipeline beginning closely enough for phase-scale audit.
    if t < ty0: return 'PRE_STILL'
    if t <= ty1: return 'YAW'
    return 'POST_STILL'

# Recursive world acceleration implied directly by predicted velocity increments.
prevv=v0; prevp=(0.,0.,0.); rows=[]
for q in pre:
    dt=q['dt']; v=q['predv']; p=q['predp']
    a=tuple((v[i]-prevv[i])/dt for i in range(3))
    dp_step=sub(p,prevp)
    rows.append((q,phase(q['t']),a,norm(a),math.hypot(a[0],a[1]),norm(v),norm(sub(p,p0)),norm(dp_step)))
    prevv=v; prevp=p

print('================ B11 PROPAGATION TIMELINE AUDIT ================')
print(f'CSV={CSV}')
print(f'PURE_IMU_LOG={LOG}')
print(f'IMU_ROWS={len(imu)} PIM_ROWS={len(pre)}')
print(f'YAW_RAW_SEC=[{ty0:.3f},{ty1:.3f}]')
print(f'PIM_TOTAL_SEC={pre[-1]["t"]:.3f}')
print('NOTE=Acceleration below is implied from recursive pim.predict velocity, not raw accelerometer.')

for ph in ('PRE_STILL','YAW','POST_STILL'):
    z=[r for r in rows if r[1]==ph]
    if not z: continue
    ah=[r[4] for r in z]; aa=[r[3] for r in z]
    first=z[0][0]; last=z[-1][0]
    dv=norm(sub(last['predv'],first['predv']))
    dp=norm(sub(last['predp'],first['predp']))
    print(f'\n--- {ph} ---')
    print(f'KF_RANGE={first["kf"]}..{last["kf"]} TIME_SEC={first["t"]:.3f}..{last["t"]:.3f} N={len(z)}')
    print(f'IMPLIED_A_NORM_P50={pct(aa,.5):.6f} P95={pct(aa,.95):.6f} MAX={max(aa):.6f}')
    print(f'IMPLIED_A_HORIZ_P50={pct(ah,.5):.6f} P95={pct(ah,.95):.6f} MAX={max(ah):.6f}')
    print(f'PHASE_DELTA_V_MPS={dv:.6f}')
    print(f'PHASE_DELTA_P_M={dp:.6f}')
    print(f'END_V_NORM_MPS={norm(last["predv"]):.6f}')
    print(f'END_P_FROM_START_M={norm(sub(last["predp"],p0)):.6f}')

print('\n--- THRESHOLD TIMELINE ---')
for th in (.02,.05,.10,.20,.50,1.0,2.0,5.0):
    h=next((r for r in rows if norm(r[0]['predv'])>=th),None)
    if h: print(f'VEL_GE_{th:.2f}_MPS=KF{h[0]["kf"]} T={h[0]["t"]:.3f} PHASE={h[1]}')
for th in (.03,.10,.30,1.,5.,10.,50.):
    h=next((r for r in rows if norm(sub(r[0]['predp'],p0))>=th),None)
    if h: print(f'POS_GE_{th:.2f}_M=KF{h[0]["kf"]} T={h[0]["t"]:.3f} PHASE={h[1]}')

# Find largest implied acceleration intervals and rotation increments.
print('\n--- TOP IMPLIED ACCELERATION INTERVALS ---')
for r in sorted(rows,key=lambda x:x[3],reverse=True)[:12]:
    q,ph,a,an,ah,_,_,_=r
    print(f'KF={q["kf"]} T={q["t"]:.3f} PHASE={ph} A=[{a[0]:+.5f},{a[1]:+.5f},{a[2]:+.5f}] |A|={an:.5f} AH={ah:.5f} DR_DEG={math.degrees(norm(q["dr"])):.3f}')

# Compare first and late pre-still acceleration to detect stationary drift evolution.
pre_rows=[r for r in rows if r[1]=='PRE_STILL']
def block_mean(z):
    if not z:return (math.nan,)*3
    return tuple(sum(r[2][i] for r in z)/len(z) for i in range(3))
if pre_rows:
    n=max(3,min(15,len(pre_rows)//4))
    a0=block_mean(pre_rows[:n]); a1=block_mean(pre_rows[-n:])
    print('\n--- PRE-STILL EVOLUTION ---')
    print(f'BLOCK_N={n}')
    print(f'EARLY_MEAN_A=[{a0[0]:+.6f},{a0[1]:+.6f},{a0[2]:+.6f}] NORM={norm(a0):.6f} H={math.hypot(a0[0],a0[1]):.6f}')
    print(f'LATE_MEAN_A=[{a1[0]:+.6f},{a1[1]:+.6f},{a1[2]:+.6f}] NORM={norm(a1):.6f} H={math.hypot(a1[0],a1[1]):.6f}')
    print(f'EARLY_TO_LATE_A_DELTA={norm(sub(a1,a0)):.6f}')

# Conservative decision: if velocity already exceeds 1 m/s before raw yaw, yaw cannot be primary origin.
yaw_primary = not any(r for r in rows if r[1]=='PRE_STILL' and norm(r[0]['predv'])>=1.0)
if yaw_primary:
    verdict='DRIFT_REMAINS_SMALL_BEFORE_YAW_ROTATION_PATH_PRIMARY_SUSPECT'
else:
    verdict='LARGE_DRIFT_ALREADY_PRESENT_BEFORE_YAW_ROTATION_NOT_PRIMARY_ORIGIN'
print('\n--- SUMMARY ---')
print('B11_PROPAGATION_VERDICT='+verdict)
print('NEXT=Use phase/timeline result to choose propagation correction target; no production tuning from PURE_IMU alone.')
print('RESULT: COMPLETE')
