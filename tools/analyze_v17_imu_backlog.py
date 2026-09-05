#!/usr/bin/env python3
import csv, statistics

H="/home/vio"
IMU=H+"/jtzero_500mm_v15.csv"
BACK=H+"/jtzero_500mm_v15_backend.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))

imu=[r for r in read(IMU) if r.get("type")=="IMU"]
be=read(BACK)
b={iv(r,"keyframe"):r for r in be}
t0=iv(b[153],"timestamp_ns"); t1=iv(b[154],"timestamp_ns")

# widen in mapped-time to see rows before/after the hole
rr=[r for r in imu if t0-1_000_000_000 <= iv(r,"mapped_ns") <= t1+1_000_000_000]

print("================ V17 IMU RECEIVE BACKLOG ================")
print(f"KF153->154 mapped interval {(t1-t0)/1e9:.3f}s")
prev=None
for r in rr:
    recv=iv(r,"recv_ns"); src=iv(r,"source_ns"); mapped=iv(r,"mapped_ns")
    if prev is None:
        dr=ds=dm=float('nan')
    else:
        dr=(recv-iv(prev,"recv_ns"))/1e6
        ds=(src-iv(prev,"source_ns"))/1e6
        dm=(mapped-iv(prev,"mapped_ns"))/1e6
    mark=""
    if prev is not None and (dr>100 or ds>100 or dm>100): mark="  <-- GAP/BURST"
    print(f"mapped_dt_from_KF153={(mapped-t0)/1e9:+.3f}s recv={recv} source={src} "
          f"dRecv={dr:.3f}ms dSource={ds:.3f}ms dMapped={dm:.3f}ms{mark}")
    prev=r

# Global max receive/source gaps.
for key in ("recv_ns","source_ns","mapped_ns"):
    ts=[iv(r,key) for r in imu]
    gaps=[(b-a)/1e6 for a,b in zip(ts,ts[1:]) if b>=a]
    print(f"{key}: max_gap_ms={max(gaps):.3f} median_gap_ms={statistics.median(gaps):.3f}")
