#!/usr/bin/env python3
import csv, os, math, statistics, sys

BACK=os.path.expanduser("~/jtzero_500mm_v25_backend.csv")
IMU=os.path.expanduser("~/jtzero_500mm_v25.csv")

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k,d=0.0):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def norm(v): return math.sqrt(sum(x*x for x in v))

for p in (BACK,IMU):
    if not os.path.exists(p):
        print("MISSING:",p); sys.exit(2)

back=read(BACK)
imu=read(IMU)

# Find largest backend jump.
best=None
for a,b in zip(back,back[1:]):
    pa=[F(a,"px_m"),F(a,"py_m"),F(a,"pz_m")]
    pb=[F(b,"px_m"),F(b,"py_m"),F(b,"pz_m")]
    dp=[(pb[j]-pa[j])*1000 for j in range(3)]
    mag=norm(dp)
    if best is None or mag>best[0]:
        best=(mag,I(b,"keyframe"),a,b)
mag,kf,a0,b0=best

# Use mapped_ns from the current V25 raw IMU CSV.
if not imu or "mapped_ns" not in imu[0]:
    print("IMU columns:", list(imu[0].keys()) if imu else [])
    raise SystemExit("mapped_ns missing")

print("================ V25 IMU AROUND LARGEST JUMP ================")
print(f"jump KF={kf} dP={mag:.2f}mm")

bykf={I(r,"keyframe"):r for r in back}
kstart=max(min(bykf),kf-12)
kend=min(max(bykf),kf+2)

for k in range(kstart,kend+1):
    br=bykf.get(k)
    prev=bykf.get(k-1)
    if not br or not prev: continue
    t0=I(prev,"timestamp_ns"); t1=I(br,"timestamp_ns")
    seg=[r for r in imu if t0 < I(r,"mapped_ns") <= t1]
    print(f"\nKF {k-1}->{k} dt={(t1-t0)*1e-6:.2f}ms imu_rows={len(seg)}")
    if not seg:
        continue

    ax=[F(r,"ax") for r in seg]; ay=[F(r,"ay") for r in seg]; az=[F(r,"az") for r in seg]
    gx=[F(r,"gx") for r in seg]; gy=[F(r,"gy") for r in seg]; gz=[F(r,"gz") for r in seg]
    amag=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(ax,ay,az)]
    gmag=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(gx,gy,gz)]

    mapped=[I(r,"mapped_ns") for r in seg]
    gaps=[(b-a)*1e-6 for a,b in zip(mapped,mapped[1:])]

    print("  acc mean=[%+.5f,%+.5f,%+.5f] m/s^2 |a| mean=%.5f min=%.5f max=%.5f" % (
        statistics.mean(ax),statistics.mean(ay),statistics.mean(az),
        statistics.mean(amag),min(amag),max(amag)))
    print("  gyro mean=[%+.6f,%+.6f,%+.6f] rad/s |g| mean=%.6f max=%.6f" % (
        statistics.mean(gx),statistics.mean(gy),statistics.mean(gz),
        statistics.mean(gmag),max(gmag)))
    if gaps:
        print("  imu gap mean=%.3fms max=%.3fms" % (statistics.mean(gaps),max(gaps)))

print("\n================ CHANGE BEFORE JUMP ================")
# Aggregate 3 windows: early, pre-jump, jump interval.
windows=[(kf-12,kf-7,"EARLY"),(kf-6,kf-1,"PREJUMP"),(kf-1,kf,"JUMP")]
for ka,kb,name in windows:
    t0=I(bykf[ka],"timestamp_ns"); t1=I(bykf[kb],"timestamp_ns")
    seg=[r for r in imu if t0 < I(r,"mapped_ns") <= t1]
    if not seg: continue
    acc=[[F(r,"ax"),F(r,"ay"),F(r,"az")] for r in seg]
    gyr=[[F(r,"gx"),F(r,"gy"),F(r,"gz")] for r in seg]
    am=[norm(v) for v in acc]; gm=[norm(v) for v in gyr]
    means=[statistics.mean(v[j] for v in acc) for j in range(3)]
    gmeans=[statistics.mean(v[j] for v in gyr) for j in range(3)]
    print(f"{name:7s} KF {ka}->{kb} n={len(seg)} accMean=[{means[0]:+.5f},{means[1]:+.5f},{means[2]:+.5f}] |a|={statistics.mean(am):.5f} gyroMean=[{gmeans[0]:+.6f},{gmeans[1]:+.6f},{gmeans[2]:+.6f}] |g|={statistics.mean(gm):.6f}")

print("\nInterpretation:")
print("- Large raw acceleration/gyro change or timing gap before the jump => source is upstream IMU/timing.")
print("- Raw IMU remains smooth while backend state accelerates => investigate bias/gravity compensation or preintegration/state-estimation coupling.")
