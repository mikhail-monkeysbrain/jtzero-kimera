#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
IMU=H+"/jtzero_500mm_v15.csv"
ATT=H+"/jtzero_500mm_v15_attitude.csv"
RNG=H+"/jtzero_500mm_v15_range.csv"
CAM=H+"/jtzero_500mm_v15_camera.csv"
BACK=H+"/jtzero_500mm_v15_backend.csv"
OUT=H+"/jtzero_v17_multistream_timeline.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):
    try:return int(float(r[k]))
    except:return 0

imu=[r for r in read(IMU) if r.get("type")=="IMU"]
att=read(ATT); rng=read(RNG); cam=read(CAM); be=read(BACK)
bk={iv(r,"keyframe"):r for r in be}

# Event A: KF153->154. Event B: known recv backlog ~0.87 s after +5.107 s.
t0=iv(bk[153],"timestamp_ns")
events=[
    ("A_KF153_154", t0, 6_000_000_000),
    ("B_POST_RECOVERY", t0+4_500_000_000, 2_500_000_000),
]

rows=[]

def gaps(rows,key):
    ts=[iv(r,key) for r in rows if iv(r,key)>0]
    ts.sort()
    return [(b-a)/1e6 for a,b in zip(ts,ts[1:])]

def count_window(rows,key,a,b):
    return [r for r in rows if a<=iv(r,key)<=b]

print("================ V17 MULTISTREAM TIMELINE ================")
for name,center,span in events:
    a=center-500_000_000
    b=center+span
    print(f"\nEVENT {name}")
    print(f"window={(b-a)/1e9:.3f}s")

    iw=count_window(imu,"recv_ns",a,b)
    aw=count_window(att,"recv_ns",a,b)
    rw=count_window(rng,"recv_ns",a,b)
    cw=count_window(cam,"corrected_timestamp_ns",a,b)

    print(f"IMU recv rows={len(iw)}")
    if iw:
        gr=gaps(iw,"recv_ns"); gs=gaps(iw,"source_ns"); gm=gaps(iw,"mapped_ns")
        print(f"  max recv gap={max(gr) if gr else float('nan'):.3f}ms")
        print(f"  max source gap={max(gs) if gs else float('nan'):.3f}ms")
        print(f"  max mapped gap={max(gm) if gm else float('nan'):.3f}ms")

    print(f"ATTITUDE rows={len(aw)}")
    if aw:
        ga=gaps(aw,"recv_ns")
        print(f"  max recv gap={max(ga) if ga else float('nan'):.3f}ms")

    print(f"RANGE rows={len(rw)}")
    if rw:
        grg=gaps(rw,"recv_ns")
        print(f"  max recv gap={max(grg) if grg else float('nan'):.3f}ms")

    print(f"CAMERA rows={len(cw)}")
    if cw:
        gc=gaps(cw,"corrected_timestamp_ns")
        seq=[iv(r,"sequence") for r in sorted(cw,key=lambda r:iv(r,"corrected_timestamp_ns"))]
        sj=[bb-aa for aa,bb in zip(seq,seq[1:])]
        print(f"  max timestamp gap={max(gc) if gc else float('nan'):.3f}ms")
        print(f"  seq jumps gt1={sum(x!=1 for x in sj)} max seq jump={max(sj) if sj else 0}")

# compact chronological anomalies across full run
print("\n================ LARGE GAPS >100ms ================")
series=[
    ("IMU_recv", imu, "recv_ns"),
    ("IMU_source", imu, "source_ns"),
    ("ATT_recv", att, "recv_ns"),
    ("RANGE_recv", rng, "recv_ns"),
    ("CAM_ts", cam, "corrected_timestamp_ns"),
]
for label,rr,key in series:
    s=sorted([(iv(r,key),r) for r in rr if iv(r,key)>0],key=lambda x:x[0])
    found=[]
    for (ta,ra),(tb,rb) in zip(s,s[1:]):
        dt=(tb-ta)/1e6
        if dt>100:
            found.append((ta,tb,dt))
    print(label)
    if not found:
        print("  none")
    else:
        for ta,tb,dt in found[:20]:
            print(f"  gap={dt:.3f}ms start={ta} end={tb}")

# merged event view around event A on recv clock where possible
print("\n================ EVENT A MERGED RECV VIEW ================")
a=t0-300_000_000; b=t0+5_000_000_000
merged=[]
for r in imu:
    ts=iv(r,"recv_ns")
    if a<=ts<=b: merged.append((ts,"IMU",iv(r,"source_ns")))
for r in att:
    ts=iv(r,"recv_ns")
    if a<=ts<=b: merged.append((ts,"ATT",iv(r,"time_boot_ms")))
for r in rng:
    ts=iv(r,"recv_ns")
    if a<=ts<=b: merged.append((ts,"RANGE",iv(r,"time_boot_ms")))
merged.sort()

prev={}
for ts,typ,src in merged:
    if typ not in prev:
        prev[typ]=ts
        continue
    dt=(ts-prev[typ])/1e6
    if dt>100:
        print(f"{typ} gap={dt:.3f}ms from={prev[typ]} to={ts} src={src}")
    prev[typ]=ts

with open(OUT,"w",newline="") as f:
    w=csv.writer(f)
    w.writerow(["type","recv_or_ts_ns","source_aux"])
    for r in imu:w.writerow(["IMU",iv(r,"recv_ns"),iv(r,"source_ns")])
    for r in att:w.writerow(["ATT",iv(r,"recv_ns"),iv(r,"time_boot_ms")])
    for r in rng:w.writerow(["RANGE",iv(r,"recv_ns"),iv(r,"time_boot_ms")])
    for r in cam:w.writerow(["CAM",iv(r,"corrected_timestamp_ns"),iv(r,"sequence")])

print("Saved:",OUT)
