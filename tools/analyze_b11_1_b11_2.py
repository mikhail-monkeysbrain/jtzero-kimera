#!/usr/bin/env python3
import csv, math, re, sys
from pathlib import Path

CSV = Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/vio/jtzero_yaw_only_v15_42.csv')
LOG = Path(sys.argv[2]) if len(sys.argv)>2 else Path('/home/vio/jtzero_backend_factor_v15_53/PURE_IMU.log')
G = 9.81
CX, CY = 0.014570, 0.082383

def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mean(vs): return tuple(sum(v[i] for v in vs)/len(vs) for i in range(3)) if vs else (math.nan,)*3
def angle(a,b):
    na,nb=norm(a),norm(b)
    if na==0 or nb==0:return math.nan
    return math.degrees(math.acos(max(-1,min(1,sum(x*y for x,y in zip(a,b))/(na*nb)))))
def vec(line,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)',line)
    return tuple(map(float,m.groups())) if m else None
def scalar(line,key):
    m=re.search(r'\b'+re.escape(key)+r'=([-+0-9.eE]+)',line)
    return float(m.group(1)) if m else math.nan

def pct(xs,p):
    if not xs:return math.nan
    s=sorted(xs); k=(len(s)-1)*p; i=int(k); f=k-i
    return s[i]*(1-f)+s[min(i+1,len(s)-1)]*f

# Raw CSV -> exactly the accel frame fed by replay before Kimera: FRD -> FLU.
imu=[]
with CSV.open(errors='replace') as f:
    rd=csv.reader(f); next(rd,None)
    for c in rd:
        if len(c)<14 or c[0] != 'IMU': continue
        try:
            t=int(c[3])*1e-9
            ax,ay,az=map(float,c[8:11]); gx,gy,gz=map(float,c[11:14])
        except: continue
        a=(ax,-ay,-az)
        gr=(gx,-gy,-gz)
        gf=(gr[0]+CX*gr[2], gr[1]+CY*gr[2], gr[2])
        imu.append((t,a,gr,gf))
if len(imu)<100: raise SystemExit('B11 ERROR: too few IMU rows')
t0=imu[0][0]
imu=[(t-t0,a,gr,gf) for t,a,gr,gf in imu]

# Automatic phase detection from corrected gyro. Threshold deliberately low enough for hand yaw.
w=[norm(x[3]) for x in imu]
active=[i for i,x in enumerate(w) if x>0.10]
if not active: raise SystemExit('B11 ERROR: yaw phase not found')
i0,i1=min(active),max(active)
ty0,ty1=imu[i0][0],imu[i1][0]
# keep clean margins around detected yaw
pre=[x for x in imu if x[0] < max(0,ty0-0.5)]
yaw=[x for x in imu if ty0 <= x[0] <= ty1]
post=[x for x in imu if x[0] > ty1+0.5]

def astats(name,rows):
    aa=[x[1] for x in rows]; mm=mean(aa); mags=[norm(x) for x in aa]
    residual=norm(mm)-G
    tilt=angle(mm,(0,0,G))
    horiz=math.hypot(mm[0],mm[1])
    dyn=[norm(sub(x,mm)) for x in aa]
    print(f'{name}_N={len(rows)}')
    print(f'{name}_MEAN_ACCEL_FLU=[{mm[0]:.6f},{mm[1]:.6f},{mm[2]:.6f}]')
    print(f'{name}_MEAN_NORM={norm(mm):.6f}')
    print(f'{name}_NORM_MINUS_G={residual:+.6f}')
    print(f'{name}_HORIZONTAL_SPECIFIC_FORCE={horiz:.6f}')
    print(f'{name}_GRAVITY_TILT_FROM_PLUS_Z_DEG={tilt:.6f}')
    print(f'{name}_DYNAMIC_RESIDUAL_P50={pct(dyn,.5):.6f}')
    print(f'{name}_DYNAMIC_RESIDUAL_P95={pct(dyn,.95):.6f}')
    return mm

print('================ B11.1/B11.2 IMU + PREINTEGRATION AUDIT ================')
print(f'CSV={CSV}')
print(f'PURE_IMU_LOG={LOG}')
print(f'IMU_ROWS={len(imu)}')
print(f'YAW_DETECTED_SEC=[{ty0:.3f},{ty1:.3f}] DURATION={ty1-ty0:.3f}')
print(f'GYRO_NORM_P50={pct(w,.5):.6f} GYRO_NORM_P95={pct(w,.95):.6f} GYRO_NORM_MAX={max(w):.6f}')
pre_a=astats('PRE_STILL',pre)
yaw_a=astats('YAW',yaw)
post_a=astats('POST_STILL',post)
print(f'PRE_TO_POST_GRAVITY_VECTOR_ANGLE_DEG={angle(pre_a,post_a):.6f}')
print(f'PRE_TO_POST_ACCEL_MEAN_DELTA={norm(sub(post_a,pre_a)):.6f}')

# Initial gravity sign/frame tests. Stationary accelerometer should be opposite world gravity
# when body is initially close to level: +Z specific force with n_gravity=-Z.
plus_err=angle(pre_a,(0,0,G)); minus_err=angle(pre_a,(0,0,-G))
print(f'INITIAL_ACCEL_ANGLE_TO_PLUS_Z_DEG={plus_err:.6f}')
print(f'INITIAL_ACCEL_ANGLE_TO_MINUS_Z_DEG={minus_err:.6f}')
print('EXPECTED_KIMERA_WORLD_GRAVITY=[0,0,-9.81]')
print('EXPECTED_LEVEL_STATIONARY_SPECIFIC_FORCE_BODY=[0,0,+9.81]')
print('GRAVITY_SIGN_SANITY='+('PASS' if plus_err < minus_err else 'FAIL'))

# Parse v15.53 true PURE_IMU preintegration/prediction trace.
prelog=[]; postlog=[]
for line in LOG.open(errors='replace'):
    if 'JTZERO_V15_53_PRE ' in line:
        prelog.append({'kf':int(scalar(line,'kf')),'dt':scalar(line,'dt'),'dp':vec(line,'dp'),'dv':vec(line,'dv'),'dr':vec(line,'dr'),'predp':vec(line,'predp'),'predv':vec(line,'predv'),'ba':vec(line,'ba'),'bg':vec(line,'bg')})
    elif 'JTZERO_V15_53_POST ' in line:
        postlog.append({'kf':int(scalar(line,'kf')),'optp':vec(line,'optp'),'optv':vec(line,'optv'),'ba':vec(line,'ba'),'bg':vec(line,'bg')})
print(f'PIM_PRE_ROWS={len(prelog)} PIM_POST_ROWS={len(postlog)}')
if not prelog: raise SystemExit('B11 ERROR: no JTZERO_V15_53_PRE rows')

# Per-KF implied average specific-force magnitude from local deltaV/dt and kinematic deltaP residual.
# deltaV is preintegrated local velocity increment; this is intentionally frame-agnostic for magnitude checks.
dvdt=[]; dpkin=[]; dts=[]; drdeg=[]
for q in prelog:
    dt=q['dt'];
    if not math.isfinite(dt) or dt<=0: continue
    dts.append(dt); dvdt.append(norm(q['dv'])/dt); drdeg.append(math.degrees(norm(q['dr'])))
    # For constant local specific force: dp ~= 0.5*dv*dt. Difference exposes non-constant/rotating force within interval.
    dpkin.append(norm(sub(q['dp'],tuple(.5*x*dt for x in q['dv']))))
print(f'PIM_DT_P50={pct(dts,.5):.6f} P95={pct(dts,.95):.6f} MAX={max(dts):.6f}')
print(f'PIM_DV_OVER_DT_P50={pct(dvdt,.5):.6f} P95={pct(dvdt,.95):.6f} MAX={max(dvdt):.6f}')
print(f'PIM_DP_MINUS_HALF_DV_DT_MM_P50={1000*pct(dpkin,.5):.3f} P95={1000*pct(dpkin,.95):.3f} MAX={1000*max(dpkin):.3f}')
print(f'PIM_DR_DEG_P50={pct(drdeg,.5):.6f} P95={pct(drdeg,.95):.6f} MAX={max(drdeg):.6f}')

# Locate where absolute predicted velocity/position starts running away.
p0=prelog[0]['predp']; v0=prelog[0]['predv']
thresholds_v=[0.05,0.10,0.25,0.50,1.0,2.0,5.0]
thresholds_p=[0.03,0.05,0.10,0.30,1.0,5.0,10.0,50.0]
for th in thresholds_v:
    hit=next((q for q in prelog if norm(sub(q['predv'],v0))>=th),None)
    print(f'FIRST_PRED_DV_GE_{th:.2f}_MPS_KF='+(str(hit['kf']) if hit else 'NONE'))
for th in thresholds_p:
    hit=next((q for q in prelog if norm(sub(q['predp'],p0))>=th),None)
    print(f'FIRST_PRED_DP_GE_{th:.2f}_M_KF='+(str(hit['kf']) if hit else 'NONE'))

last=prelog[-1]
print(f'PRED_FINAL_DELTA_P_MM={1000*norm(sub(last["predp"],p0)):.3f}')
print(f'PRED_FINAL_DELTA_V_MPS={norm(sub(last["predv"],v0)):.6f}')

# Bias constancy and initial values.
ba0,bg0=prelog[0]['ba'],prelog[0]['bg']; ba1,bg1=prelog[-1]['ba'],prelog[-1]['bg']
print(f'INITIAL_ACCEL_BIAS=[{ba0[0]:.9f},{ba0[1]:.9f},{ba0[2]:.9f}]')
print(f'INITIAL_GYRO_BIAS=[{bg0[0]:.9f},{bg0[1]:.9f},{bg0[2]:.9f}]')
print(f'ACCEL_BIAS_CHANGE={norm(sub(ba1,ba0)):.9f}')
print(f'GYRO_BIAS_CHANGE={norm(sub(bg1,bg0)):.9f}')

# Decision tree intentionally conservative.
sign_ok=plus_err < 20 and plus_err < minus_err
norm_ok=abs(norm(pre_a)-G) < 0.30
static_horiz=math.hypot(pre_a[0],pre_a[1])
if not sign_ok:
    verdict='GRAVITY_SIGN_OR_FRAME_MISMATCH_STRONGLY_SUSPECT'
elif not norm_ok:
    verdict='ACCEL_SCALE_OR_GRAVITY_MAGNITUDE_MISMATCH_SUSPECT'
elif static_horiz > 0.60:
    verdict='INITIAL_GRAVITY_ALIGNMENT_OR_REAL_STATIC_TILT_NEEDS_SEPARATION'
else:
    verdict='INITIAL_GRAVITY_SIGN_AND_MAGNITUDE_PLAUSIBLE_DRIFT_DEVELOPS_DURING_PROPAGATION'
print('B11_1_B11_2_VERDICT='+verdict)
print('RESULT: COMPLETE')
