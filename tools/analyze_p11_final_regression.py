#!/usr/bin/env python3
import csv, math, os, statistics
from collections import defaultdict

FC='/home/vio/jtzero_p11_final_fc.csv'
ATT='/home/vio/jtzero_p11_final_attitude.csv'
CAM='/home/vio/jtzero_p11_final_camera.csv'
MAIN='/home/vio/jtzero_p11_final.csv'
REF1542=0.0020182
REF13=0.0004921

for p in (FC,ATT,CAM,MAIN):
    if not os.path.exists(p): raise SystemExit(f'FATAL: missing {p}')

def f(x,default=float('nan')):
    try:return float(x)
    except:return default

def finite(v):return [x for x in v if math.isfinite(x)]
def mean(v):
    v=finite(v);return statistics.fmean(v) if v else float('nan')
def sd(v):
    v=finite(v);return statistics.stdev(v) if len(v)>1 else float('nan')
def norm2(x,y):return math.hypot(x,y)
def read(p):
    with open(p,newline='') as h:return list(csv.DictReader(h))
def pick(r,*names):
    for n in names:
        if n in r and r[n]!='':return r[n]
    return ''
def val(r,kind):
    aliases={
      'gx':('gx_rad_s','gx','xgyro_rad_s','xgyro'),
      'gy':('gy_rad_s','gy','ygyro_rad_s','ygyro'),
      'gz':('gz_rad_s','gz','zgyro_rad_s','zgyro'),
      'roll':('roll_deg','roll'),'pitch':('pitch_deg','pitch'),'yaw':('yaw_deg','yaw'),
      'lat':('transport_latency_ms',),
    }
    return f(pick(r,*aliases[kind]))
def block_means(rows,kind,sec=1.0):
    good=[r for r in rows if math.isfinite(f(r.get('recv_rpi_ns'))) and math.isfinite(val(r,kind))]
    if not good:return []
    t0=min(f(r['recv_rpi_ns']) for r in good); bins=defaultdict(list)
    for r in good:bins[int((f(r['recv_rpi_ns'])-t0)/(sec*1e9))].append(val(r,kind))
    return [mean(bins[k]) for k in sorted(bins)]
def phase_rows(rows,ph):return [r for r in rows if r.get('phase')==ph]

def fmt(x,n=7):return f'{x:+.{n}f}' if math.isfinite(x) else 'NaN'

fc=read(FC);att=read(ATT);cam=read(CAM);main=read(MAIN)
print('=== P11 FINAL ONE-SHOT ANALYSIS v2 ===')
print('FC columns:',list(fc[0].keys()) if fc else [])
print('rows FC/ATT/CAM/MAIN:',len(fc),len(att),len(cam),len(main))
if not fc or 'type' not in fc[0] or 'phase' not in fc[0]:raise SystemExit('FATAL: FC CSV lacks type/phase')

bytype=defaultdict(list);byphase=defaultdict(list)
for r in fc:bytype[r['type']].append(r);byphase[r['phase']].append(r)
print('\n=== STREAM COUNTS ===')
for k in sorted(bytype):print(f'{k:24s} {len(bytype[k])}')
print('\n=== PHASE COUNTS ===')
for k in sorted(byphase):print(f'{k:24s} {len(byphase[k])}')

stationary=['PRE','YAW1_HOLD','HOME1','YAW2_HOLD','HOME2','YAW3_HOLD','FINAL']
streams=['HIGHRES_IMU','SCALED_IMU','SCALED_IMU2']
stats={}
print('\n=== GYRO STATIONARY PHASE MEANS rad/s ===')
for st in streams:
    rows=bytype.get(st,[]);print(f'-- {st} --')
    for ph in stationary:
        q=phase_rows(rows,ph)
        if not q:continue
        gx=[val(r,'gx') for r in q];gy=[val(r,'gy') for r in q];gz=[val(r,'gz') for r in q]
        bmx=block_means(q,'gx');bmy=block_means(q,'gy')
        stats[(st,ph)]=(mean(gx),mean(gy),mean(gz))
        print(f'{ph:10s} G=[{fmt(mean(gx))},{fmt(mean(gy))},{fmt(mean(gz))}] SD=[{sd(gx):.7f},{sd(gy):.7f},{sd(gz):.7f}] blockSDxy=[{sd(bmx):.7f},{sd(bmy):.7f}] n={len(q)}')

print('\n=== PRE -> HOME/FINAL GYRO XY SHIFTS ===')
shifts={}
for st in streams:
    if (st,'PRE') not in stats:continue
    p=stats[(st,'PRE')]
    for ph in ['HOME1','HOME2','FINAL']:
        if (st,ph) not in stats:continue
        a=stats[(st,ph)];dx=a[0]-p[0];dy=a[1]-p[1];n=norm2(dx,dy);shifts[(st,ph)]=n
        print(f'{st:12s} {ph:6s} dGxy=[{dx:+.7f},{dy:+.7f}] norm={n:.7f} ratio_v15_42={n/REF1542:.3f} ratio_v13={n/REF13:.3f}')

# Integrate HIGHRES yaw. Start integration at PRE, not ZERO, to match operator-relative motion.
hr=bytype.get('HIGHRES_IMU',[])
pre=phase_rows(hr,'PRE')
if pre:
    bias=mean([val(r,'gz') for r in pre]); active=False;yaw=0.;last=None;ends={};prev=None
    for r in hr:
        ph=r['phase']
        if ph=='PRE' and not active:active=True;last=f(r['recv_rpi_ns']);prev=ph
        if not active:continue
        t=f(r['recv_rpi_ns']);gz=val(r,'gz')
        if last is not None:
            dt=(t-last)*1e-9
            if 0<dt<.1 and math.isfinite(gz):yaw+=(gz-bias)*dt*180/math.pi
        last=t
        if prev is not None and ph!=prev:ends[prev]=yaw
        prev=ph
    if prev:ends[prev]=yaw
    print('\n=== HIGHRES RAW-YAW INTEGRATION ===')
    print(f'PRE gz bias={bias:+.8f} rad/s')
    for ph in ['PRE','YAW1_OUT','YAW1_HOLD','YAW1_HOME','HOME1','YAW2_OUT','YAW2_HOLD','YAW2_HOME','HOME2','YAW3_OUT','YAW3_HOLD','YAW3_HOME','FINAL']:
        if ph in ends:print(f'{ph:10s} end_yaw={ends[ph]:+8.2f} deg')

# ATTITUDE FC diagnostics are already degrees in *_deg columns.
ar=bytype.get('ATTITUDE',[])
if ar:
    print('\n=== FC ATTITUDE PHASE MEANS deg (absolute EKF attitude) ===')
    for ph in stationary:
        q=phase_rows(ar,ph)
        if q:print(f'{ph:10s} R={mean([val(r,"roll") for r in q]):+.3f} P={mean([val(r,"pitch") for r in q]):+.3f} Y={mean([val(r,"yaw") for r in q]):+.3f}')

v=bytype.get('VIBRATION',[])
if v:
    vx=[f(r.get('vibration_x')) for r in v];vy=[f(r.get('vibration_y')) for r in v];vz=[f(r.get('vibration_z')) for r in v]
    c0=max([f(r.get('clipping_0'),0) for r in v] or [0]);c1=max([f(r.get('clipping_1'),0) for r in v] or [0]);c2=max([f(r.get('clipping_2'),0) for r in v] or [0])
    print('\n=== VIBRATION ===');print(f'mean=[{mean(vx):.4f},{mean(vy):.4f},{mean(vz):.4f}] max=[{max(finite(vx)):.4f},{max(finite(vy)):.4f},{max(finite(vz)):.4f}] clipping_max=[{int(c0)},{int(c1)},{int(c2)}]')

if cam:
    seq=[int(float(r['sequence'])) for r in cam if r.get('sequence','')!=''];gaps=[]
    for a,b in zip(seq,seq[1:]):
        if b>a+1:gaps.append((a,b,b-a-1))
    print('\n=== CAMERA DROPS ===');print(f'frames={len(seq)} gap_events={len(gaps)} missing_frames={sum(g[2] for g in gaps)} max_gap={max([g[2] for g in gaps] or [0])}');print('top gaps:',sorted(gaps,key=lambda x:x[2],reverse=True)[:10])

lat=finite([f(r.get('transport_latency_ms')) for r in main])
if lat:
    sl=sorted(lat)
    def pct(q):return sl[int(round((len(sl)-1)*q))]
    # Also report trimmed latency so rare clock/timesync discontinuities do not dominate the mean.
    trimmed=[x for x in lat if x<100]
    print('\n=== HIGHRES TRANSPORT LATENCY ===');print(f'all: mean={mean(lat):.3f} ms p50={pct(.5):.3f} p95={pct(.95):.3f} p99={pct(.99):.3f} max={max(lat):.3f}');print(f'<100ms: n={len(trimmed)}/{len(lat)} mean={mean(trimmed):.3f} ms')

print('\n=== QUICK VERDICT ===')
for st in streams:
    n=shifts.get((st,'FINAL'),float('nan'))
    if not math.isfinite(n):tag='INVALID_ANALYSIS'
    elif n>=.0010:tag='V15_42_LIKE'
    elif n>=.0003:tag='MODERATE'
    else:tag='SMALL'
    print(f'{st}: FINAL dGxy={fmt(n)} -> {tag}')
print(f'REFERENCE v15.42 dGxy={REF1542:.7f} rad/s; v13 dGxy≈{REF13:.7f} rad/s')
