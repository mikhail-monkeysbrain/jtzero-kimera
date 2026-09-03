#!/usr/bin/env python3
import csv, math, os, statistics, sys
from collections import defaultdict

FC='/home/vio/jtzero_p11_final_fc.csv'
ATT='/home/vio/jtzero_p11_final_attitude.csv'
CAM='/home/vio/jtzero_p11_final_camera.csv'
MAIN='/home/vio/jtzero_p11_final.csv'

for p in (FC,ATT,CAM,MAIN):
    if not os.path.exists(p):
        raise SystemExit(f'FATAL: missing {p}')

def f(x, default=float('nan')):
    try: return float(x)
    except: return default

def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.fmean(v) if v else float('nan')

def sd(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.stdev(v) if len(v)>1 else float('nan')

def norm2(x,y): return math.hypot(x,y)

def block_means(rows, key, sec=1.0):
    if not rows: return []
    t0=min(f(r.get('recv_rpi_ns')) for r in rows)
    bins=defaultdict(list)
    for r in rows:
        t=f(r.get('recv_rpi_ns'))
        v=f(r.get(key))
        if math.isfinite(t) and math.isfinite(v): bins[int((t-t0)/(sec*1e9))].append(v)
    return [mean(bins[k]) for k in sorted(bins)]

def read(path):
    with open(path,newline='') as h: return list(csv.DictReader(h))

fc=read(FC); att=read(ATT); cam=read(CAM); main=read(MAIN)
print('=== P11 FINAL ONE-SHOT ANALYSIS ===')
print('FC columns:', list(fc[0].keys()) if fc else [])
print('rows FC/ATT/CAM/MAIN:',len(fc),len(att),len(cam),len(main))

# Accept multiple possible column names defensively.
def pick(row,*names):
    for n in names:
        if n in row: return row[n]
    return ''

type_key='type' if fc and 'type' in fc[0] else None
phase_key='phase' if fc and 'phase' in fc[0] else None
if not type_key or not phase_key:
    raise SystemExit('FATAL: FC CSV lacks type/phase columns')

bytype=defaultdict(list); byphase=defaultdict(list)
for r in fc:
    bytype[r[type_key]].append(r); byphase[r[phase_key]].append(r)
print('\n=== STREAM COUNTS ===')
for k in sorted(bytype): print(f'{k:24s} {len(bytype[k])}')
print('\n=== PHASE COUNTS ===')
for k in sorted(byphase): print(f'{k:24s} {len(byphase[k])}')

# Gyro statistics per stream and stationary phases.
stationary=['PRE','YAW1_HOLD','HOME1','YAW2_HOLD','HOME2','YAW3_HOLD','FINAL']
streams=['HIGHRES_IMU','SCALED_IMU','SCALED_IMU2']
print('\n=== GYRO STATIONARY PHASE MEANS rad/s ===')
stats={}
for st in streams:
    rows=bytype.get(st,[])
    if not rows: continue
    print(f'-- {st} --')
    for ph in stationary:
        q=[r for r in rows if r[phase_key]==ph]
        if not q: continue
        gx=[f(pick(r,'gx','xgyro_rad_s','xgyro')) for r in q]
        gy=[f(pick(r,'gy','ygyro_rad_s','ygyro')) for r in q]
        gz=[f(pick(r,'gz','zgyro_rad_s','zgyro')) for r in q]
        bm_x=block_means(q,'gx') if 'gx' in q[0] else []
        bm_y=block_means(q,'gy') if 'gy' in q[0] else []
        stats[(st,ph)]=(mean(gx),mean(gy),mean(gz))
        print(f'{ph:10s} G=[{mean(gx):+.7f},{mean(gy):+.7f},{mean(gz):+.7f}] SD=[{sd(gx):.7f},{sd(gy):.7f},{sd(gz):.7f}] blockSDxy=[{sd(bm_x):.7f},{sd(bm_y):.7f}] n={len(q)}')

print('\n=== PRE -> HOME/FINAL GYRO XY SHIFTS ===')
for st in streams:
    if (st,'PRE') not in stats: continue
    p=stats[(st,'PRE')]
    for ph in ['HOME1','HOME2','FINAL']:
        if (st,ph) not in stats: continue
        a=stats[(st,ph)]; dx=a[0]-p[0]; dy=a[1]-p[1]
        print(f'{st:12s} {ph:6s} dGxy=[{dx:+.7f},{dy:+.7f}] norm={norm2(dx,dy):.7f} rad/s  ratio_to_v15_42={norm2(dx,dy)/0.0020182:.3f}')

# Integrate HIGHRES gz with PRE bias using recv timestamps, report phase-end yaw.
hr=bytype.get('HIGHRES_IMU',[])
if hr:
    pre=[r for r in hr if r[phase_key]=='PRE']
    bias=mean([f(pick(r,'gz','zgyro_rad_s','zgyro')) for r in pre])
    yaw=0.0; last=None; ends={}
    phase_prev=None
    for r in hr:
        t=f(r.get('recv_rpi_ns')); gz=f(pick(r,'gz','zgyro_rad_s','zgyro'))
        ph=r[phase_key]
        if last is not None:
            dt=(t-last)*1e-9
            if 0<dt<0.1 and math.isfinite(gz): yaw+=(gz-bias)*dt*180/math.pi
        last=t
        if phase_prev is not None and ph!=phase_prev: ends[phase_prev]=yaw
        phase_prev=ph
    if phase_prev: ends[phase_prev]=yaw
    print('\n=== HIGHRES RAW-YAW INTEGRATION ===')
    print(f'PRE gz bias={bias:+.8f} rad/s')
    for ph in ['PRE','YAW1_OUT','YAW1_HOLD','YAW1_HOME','HOME1','YAW2_OUT','YAW2_HOLD','YAW2_HOME','HOME2','YAW3_OUT','YAW3_HOLD','YAW3_HOME','FINAL']:
        if ph in ends: print(f'{ph:10s} end_yaw={ends[ph]:+8.2f} deg')

# ATTITUDE relative range and phase means.
if att:
    print('\n=== ATTITUDE RELATIVE R/P/Y PHASE MEANS ===')
    # Map phases by FC diag ATTITUDE timestamps if available.
    ar=bytype.get('ATTITUDE',[])
    for ph in stationary:
        q=[r for r in ar if r[phase_key]==ph]
        if not q: continue
        rr=[f(pick(r,'roll')) for r in q]; pp=[f(pick(r,'pitch')) for r in q]; yy=[f(pick(r,'yaw')) for r in q]
        print(f'{ph:10s} R={mean(rr):+.3f} P={mean(pp):+.3f} Y={mean(yy):+.3f} deg')

# Vibration and clipping.
v=bytype.get('VIBRATION',[])
if v:
    vx=[f(pick(r,'vibration_x')) for r in v]; vy=[f(pick(r,'vibration_y')) for r in v]; vz=[f(pick(r,'vibration_z')) for r in v]
    c0=max([f(pick(r,'clipping_0'),0) for r in v] or [0]); c1=max([f(pick(r,'clipping_1'),0) for r in v] or [0]); c2=max([f(pick(r,'clipping_2'),0) for r in v] or [0])
    print('\n=== VIBRATION ===')
    print(f'mean=[{mean(vx):.4f},{mean(vy):.4f},{mean(vz):.4f}] max=[{max(vx):.4f},{max(vy):.4f},{max(vz):.4f}] clipping_max=[{int(c0)},{int(c1)},{int(c2)}]')

# Camera sequence gap distribution.
if cam:
    seq=[int(float(r['sequence'])) for r in cam if r.get('sequence','')!='']
    gaps=[]
    for a,b in zip(seq,seq[1:]):
        if b>a+1: gaps.append((a,b,b-a-1))
    print('\n=== CAMERA DROPS ===')
    print(f'frames={len(seq)} gap_events={len(gaps)} missing_frames={sum(g[2] for g in gaps)} max_gap={max([g[2] for g in gaps] or [0])}')
    print('top gaps:', sorted(gaps,key=lambda x:x[2],reverse=True)[:10])

# Transport latency from replay-compatible HIGHRES main CSV.
lat=[f(r.get('transport_latency_ms')) for r in main]
lat=[x for x in lat if math.isfinite(x)]
if lat:
    sl=sorted(lat)
    def pct(q): return sl[int(round((len(sl)-1)*q))]
    print('\n=== HIGHRES TRANSPORT LATENCY ===')
    print(f'mean={mean(lat):.3f} ms p50={pct(.5):.3f} p95={pct(.95):.3f} p99={pct(.99):.3f} max={max(lat):.3f}')

# Simple verdict gates.
print('\n=== QUICK VERDICT ===')
for st in streams:
    if (st,'PRE') in stats and (st,'FINAL') in stats:
        p=stats[(st,'PRE')]; a=stats[(st,'FINAL')]; n=norm2(a[0]-p[0],a[1]-p[1])
        tag='V15_42_LIKE' if n>=0.0010 else ('MODERATE' if n>=0.0003 else 'SMALL')
        print(f'{st}: FINAL dGxy={n:.7f} -> {tag}')
print('REFERENCE v15.42 dGxy=0.0020182 rad/s; v13 dGxy≈0.0004921 rad/s')
