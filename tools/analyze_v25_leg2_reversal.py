#!/usr/bin/env python3
import csv, math
from pathlib import Path

HOME=Path("/home/vio")
BACKEND=HOME/"jtzero_500mm_v25_backend.csv"
FRONTEND=HOME/"jtzero_500mm_v25_frontend.csv"
CAMERA=HOME/"jtzero_500mm_v25_camera.csv"
EVENTS=HOME/"jtzero_500mm_v25_events.csv"

def read_csv(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))

b=read_csv(BACKEND)
f=read_csv(FRONTEND)
c=read_csv(CAMERA)
e=read_csv(EVENTS)

# Numeric helpers.
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])

# Map each backend state to nearest frontend callback by sensor timestamp.
front_ts=[I(r,"timestamp_ns") for r in f]

def nearest_front(ts):
    lo,hi=0,len(front_ts)
    while lo<hi:
        m=(lo+hi)//2
        if front_ts[m]<ts: lo=m+1
        else: hi=m
    cand=[]
    for j in (lo-1,lo,lo+1):
        if 0<=j<len(f): cand.append((abs(front_ts[j]-ts),f[j]))
    return min(cand,key=lambda x:x[0]) if cand else (10**30,None)

# Find leg2 start/end events.
ev2={r["event"]:r for r in e if r["leg"]=="2"}
if "START" not in ev2 or "END" not in ev2:
    raise SystemExit("LEG2 START/END events not found")
ks=I(ev2["START"],"keyframe")
ke=I(ev2["END"],"keyframe")

print("================ V25 LEG2 REVERSAL FORENSIC ================")
print(f"LEG2 event KF {ks} -> {ke}")
print()

# Locate largest change in horizontal direction / first point where X begins increasing
# after an initial return movement.
rows=[r for r in b if ks<=I(r,"keyframe")<=ke]
minx=min(rows,key=lambda r:F(r,"px_m"))
miny=min(rows,key=lambda r:F(r,"py_m"))
print(f"minimum X at KF={I(minx,'keyframe')} X={F(minx,'px_m')*1000:.1f}mm Y={F(minx,'py_m')*1000:.1f}mm")
print(f"minimum Y at KF={I(miny,'keyframe')} X={F(miny,'px_m')*1000:.1f}mm Y={F(miny,'py_m')*1000:.1f}mm")
print()

print(" KF   P[mm]                         V[mm/s]                    dP[mm]  FF_status       inl/put ratio  FFdt[ms] cbAge[ms]")
prev=None
for r in rows:
    k=I(r,"keyframe")
    if k<175 and k not in (ks,): continue
    if k>256: continue
    px,py,pz=(F(r,"px_m"),F(r,"py_m"),F(r,"pz_m"))
    vx,vy,vz=(F(r,"vx_m_s"),F(r,"vy_m_s"),F(r,"vz_m_s"))
    if prev is None: dp=0.0
    else:
        dp=1000*math.sqrt((px-prev[0])**2+(py-prev[1])**2+(pz-prev[2])**2)
    prev=(px,py,pz)
    dt,fr=nearest_front(I(r,"timestamp_ns"))
    if fr:
        st=fr["mono_status"]
        inl=I(fr,"mono_inliers"); put=I(fr,"mono_putatives")
        rat=F(fr,"mono_inlier_ratio")
        cbage=(I(r,"callback_wall_ns")-I(fr,"callback_wall_ns"))/1e6
        print(f"{k:3d} [{px*1000:+7.1f},{py*1000:+7.1f},{pz*1000:+7.1f}] "
              f"[{vx*1000:+7.1f},{vy*1000:+7.1f},{vz*1000:+7.1f}] "
              f"{dp:7.2f}  {st:14s} {inl:3d}/{put:3d} {rat:5.2f} {dt/1e6:+8.2f} {cbage:+8.2f}")

print()
print("================ FRONTEND STATUS COUNTS IN LEG2 ================")
counts={}
for fr in f:
    ts=I(fr,"timestamp_ns")
    if I(ev2["START"],"state_timestamp_ns") <= ts <= I(ev2["END"],"state_timestamp_ns"):
        counts[fr["mono_status"]]=counts.get(fr["mono_status"],0)+1
for k,v in sorted(counts.items(), key=lambda kv:(-kv[1],kv[0])):
    print(f"{k:16s} {v}")

print()
print("================ CAMERA GAPS AROUND LEG2 ================")
# camera csv: sequence,raw_ts_ns,corrected_ts_ns,mapping_valid,selected,...
cam=[r for r in c if I(ev2["START"],"state_timestamp_ns")-1_000_000_000 <= I(r,"corrected_ts_ns") <= I(ev2["END"],"state_timestamp_ns")+1_000_000_000]
maxgap=(0,None,None)
prev=None
for r in cam:
    ts=I(r,"corrected_ts_ns")
    if prev is not None and ts-prev[0]>maxgap[0]:
        maxgap=(ts-prev[0],prev[1],r)
    prev=(ts,r)
if maxgap[1]:
    print(f"max corrected camera gap={maxgap[0]/1e6:.3f}ms seq {maxgap[1]['sequence']}->{maxgap[2]['sequence']}")
else:
    print("no camera rows")

print()
print("================ EVENT AGES ================")
for r in e:
    print(f"{r['event']:5s} LEG{r['leg']} KF={r['keyframe']} phase={r['phase']} age={float(r['state_age_at_event_ms']):.1f}ms")
