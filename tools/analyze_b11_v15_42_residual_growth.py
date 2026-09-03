#!/usr/bin/env python3
import csv, math, re
from pathlib import Path

CSV=Path('/home/vio/jtzero_yaw_only_v15_42.csv')
LOG=Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log')
CX,CY=0.014570,0.082383

def norm(v): return math.sqrt(sum(x*x for x in v))
def mean(vs): return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3)) if vs else (math.nan,)*3
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def angle(a,b):
    na,nb=norm(a),norm(b)
    if na==0 or nb==0:return math.nan
    return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))/(na*nb)))))
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

def corr(a,b):
    if len(a)<3:return math.nan
    ma=sum(a)/len(a); mb=sum(b)/len(b)
    da=[x-ma for x in a]; db=[x-mb for x in b]
    den=math.sqrt(sum(x*x for x in da)*sum(x*x for x in db))
    return sum(x*y for x,y in zip(da,db))/den if den else math.nan

imu=[]
with CSV.open(errors='replace') as f:
    rd=csv.reader(f); next(rd,None)
    for c in rd:
        if len(c)<14 or c[0]!='IMU':continue
        try:
            t=int(c[3])*1e-9; a=(float(c[8]),-float(c[9]),-float(c[10])); g0=(float(c[11]),-float(c[12]),-float(c[13]))
        except:continue
        g=(g0[0]+CX*g0[2],g0[1]+CY*g0[2],g0[2])
        imu.append((t,a,g))
if len(imu)<100:raise SystemExit('B11 ERROR: too few IMU rows')
t0=imu[0][0]; imu=[(t-t0,a,g) for t,a,g in imu]
active=[i for i,x in enumerate(imu) if norm(x[2])>.10]
if not active:raise SystemExit('B11 ERROR: yaw not found')
ty0,ty1=imu[min(active)][0],imu[max(active)][0]

pim=[]; t=0.
for line in LOG.open(errors='replace'):
    if 'JTZERO_V15_53_PRE ' not in line:continue
    q=dict(kf=int(scalar(line,'kf')),dt=scalar(line,'dt'),dp=vec(line,'dp'),dv=vec(line,'dv'),dr=vec(line,'dr'),predp=vec(line,'predp'),predv=vec(line,'predv'),ba=vec(line,'ba'),bg=vec(line,'bg'))
    t+=q['dt'];q['t']=t;pim.append(q)
if len(pim)<100:raise SystemExit('B11 ERROR: too few PIM rows')

def phase(t):
    if t<ty0:return 'PRE_STILL'
    if t<=ty1:return 'YAW'
    return 'POST_STILL'

def imu_window(t1,t2):return [x for x in imu if t1 < x[0] <= t2]

# Per-PIM interval: raw accel/gyro statistics + recursive predicted world acceleration.
rows=[]; prevv=(0.,0.,0.); prevt=0.
for q in pim:
    z=imu_window(prevt,q['t']); prevt=q['t']
    ma=mean([x[1] for x in z]); mg=mean([x[2] for x in z]);
    a_pred=tuple((q['predv'][i]-prevv[i])/q['dt'] for i in range(3));prevv=q['predv']
    rows.append(dict(q=q,ph=phase(q['t']),ma=ma,mg=mg,an=norm(a_pred),ah=math.hypot(a_pred[0],a_pred[1]),ap=a_pred,accnorm=norm(ma),gyronorm=norm(mg),tilt=angle(ma,(0,0,1)),drdeg=math.degrees(norm(q['dr']))))

print('================ B11 v15.42 RESIDUAL GROWTH AUDIT ================')
print(f'IMU_ROWS={len(imu)} PIM_ROWS={len(pim)} YAW_SEC=[{ty0:.3f},{ty1:.3f}] TOTAL_SEC={pim[-1]["t"]:.3f}')
print('NOTE=Single-dataset audit only; no comparison to old runs and no backend rebuild.')

# Split each still phase into thirds to expose gradual changes, not only phase averages.
def thirds(ph):
    z=[r for r in rows if r['ph']==ph]
    n=len(z)
    return [('EARLY',z[:n//3]),('MID',z[n//3:2*n//3]),('LATE',z[2*n//3:])]

def report(name,z):
    if not z:return
    ma=mean([r['ma'] for r in z]);mg=mean([r['mg'] for r in z]);
    ah=[r['ah'] for r in z]; an=[r['an'] for r in z]; dr=[r['drdeg'] for r in z]
    print(f'{name}: T={z[0]["q"]["t"]:.3f}..{z[-1]["q"]["t"]:.3f} N={len(z)} '
          f'AxyP50={pct(ah,.5):.6f} AxyP95={pct(ah,.95):.6f} '
          f'ApredP50={pct(an,.5):.6f} rawA=[{ma[0]:+.5f},{ma[1]:+.5f},{ma[2]:+.5f}] |rawA|={norm(ma):.6f} '
          f'rawTilt={angle(ma,(0,0,1)):.4f}deg gyroMean=[{mg[0]:+.6f},{mg[1]:+.6f},{mg[2]:+.6f}] |gyro|={norm(mg):.6f} DRp50={pct(dr,.5):.4f}deg')

print('\n--- WITHIN-PHASE EVOLUTION ---')
for ph in ('PRE_STILL','YAW','POST_STILL'):
    print(f'[{ph}]')
    if ph=='YAW':report('ALL',[r for r in rows if r['ph']==ph])
    else:
        for tag,z in thirds(ph):report(tag,z)

print('\n--- RAW SENSOR PRE vs POST ---')
pre=[r for r in rows if r['ph']=='PRE_STILL'];post=[r for r in rows if r['ph']=='POST_STILL']
for key,label in [('ma','ACCEL'),('mg','GYRO')]:
    a=mean([r[key] for r in pre]);b=mean([r[key] for r in post]);d=sub(b,a)
    print(f'{label}_PRE_MEAN=[{a[0]:+.7f},{a[1]:+.7f},{a[2]:+.7f}]')
    print(f'{label}_POST_MEAN=[{b[0]:+.7f},{b[1]:+.7f},{b[2]:+.7f}]')
    print(f'{label}_POST_MINUS_PRE=[{d[0]:+.7f},{d[1]:+.7f},{d[2]:+.7f}] NORM={norm(d):.7f}')
print(f'ACCEL_GRAVITY_DIRECTION_PRE_POST_ANGLE_DEG={angle(mean([r["ma"] for r in pre]),mean([r["ma"] for r in post])):.6f}')

print('\n--- CORRELATIONS ACROSS ALL KF INTERVALS ---')
ah=[r['ah'] for r in rows]
for vals,name in [([r['q']['t'] for r in rows],'TIME'),([r['gyronorm'] for r in rows],'GYRO_NORM'),([r['drdeg'] for r in rows],'PIM_DR_DEG'),([r['accnorm'] for r in rows],'RAW_ACCEL_NORM'),([r['tilt'] for r in rows],'RAW_ACCEL_TILT')]:
    print(f'CORR_Axy_{name}={corr(ah,vals):+.6f}')

print('\n--- POST-STILL: DOES RESIDUAL KEEP GROWING WHILE ROTATION IS QUIET? ---')
quiet=[r for r in post if r['drdeg']<0.10]
print(f'POST_QUIET_KF={len(quiet)}/{len(post)}')
if quiet:
    print(f'POST_QUIET_Axy_P50={pct([r["ah"] for r in quiet],.5):.6f} P95={pct([r["ah"] for r in quiet],.95):.6f}')
    print(f'CORR_POST_QUIET_Axy_TIME={corr([r["ah"] for r in quiet],[r["q"]["t"] for r in quiet]):+.6f}')
    print(f'CORR_POST_QUIET_Axy_RAW_ACCEL_TILT={corr([r["ah"] for r in quiet],[r["tilt"] for r in quiet]):+.6f}')

# Identify whether raw measured gravity direction itself changed enough to account for post state.
prema=mean([r['ma'] for r in pre]);postma=mean([r['ma'] for r in post]); gravchange=angle(prema,postma)
postmed=pct([r['ah'] for r in post],.5); equivalent=math.degrees(math.asin(min(1,postmed/9.81)))
print('\n--- SUMMARY ---')
print(f'POST_MEDIAN_Axy_EQUIVALENT_TILT_DEG={equivalent:.6f}')
print(f'MEASURED_PRE_POST_GRAVITY_DIRECTION_CHANGE_DEG={gravchange:.6f}')
if gravchange > .5*equivalent:
    verdict='REAL_OR_MEASURED_BODY_TILT_CHANGE_IS_LARGE_ENOUGH_TO_MATTER'
else:
    verdict='PREDICTED_RESIDUAL_GROWS_MORE_THAN_MEASURED_GRAVITY_DIRECTION_CHANGE'
print('B11_V15_42_RESIDUAL_VERDICT='+verdict)
print('NEXT=Use raw-vs-predicted evolution to decide whether to target changing body attitude, gyro propagation, or accelerometer/bias model.')
print('RESULT: COMPLETE')
