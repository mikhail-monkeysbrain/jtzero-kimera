#!/usr/bin/env python3
import csv, os, math, statistics, sys

BACK=os.path.expanduser("~/jtzero_500mm_v25_backend.csv")

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k,d=0.0):
    try:return float(r.get(k,d))
    except:return d
def I(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def norm(v): return math.sqrt(sum(x*x for x in v))

if not os.path.exists(BACK):
    raise SystemExit("missing backend CSV")

rows=read(BACK)
bykf={I(r,"keyframe"):r for r in rows}

best=None
for a,b in zip(rows,rows[1:]):
    pa=[F(a,"px_m"),F(a,"py_m"),F(a,"pz_m")]
    pb=[F(b,"px_m"),F(b,"py_m"),F(b,"pz_m")]
    dp=norm([(pb[j]-pa[j])*1000 for j in range(3)])
    if best is None or dp>best[0]:
        best=(dp,I(b,"keyframe"))
jump,kf=best

print("================ V25 BIAS / ATTITUDE AROUND JUMP ================")
print(f"largest jump KF={kf} dP={jump:.2f}mm")
print(" KF  speed  roll pitch yaw    BA[x y z] m/s^2            |BA|    BG[x y z] rad/s             |BG|")

for k in range(max(min(bykf),kf-15),min(max(bykf),kf+10)+1):
    r=bykf.get(k)
    if not r: continue
    ba=[F(r,"bax"),F(r,"bay"),F(r,"baz")]
    bg=[F(r,"bgx"),F(r,"bgy"),F(r,"bgz")]
    mark=" <<<" if k==kf else ""
    print(f"{k:3d} {F(r,'speed_m_s')*1000:6.1f} "
          f"{F(r,'roll_deg'):+6.2f} {F(r,'pitch_deg'):+6.2f} {F(r,'yaw_deg'):+6.2f}  "
          f"[{ba[0]:+.4f},{ba[1]:+.4f},{ba[2]:+.4f}] {norm(ba):.4f}  "
          f"[{bg[0]:+.5f},{bg[1]:+.5f},{bg[2]:+.5f}] {norm(bg):.5f}{mark}")

# quantify step changes
print("\n================ LARGEST BIAS STEPS ================")
steps=[]
for a,b in zip(rows,rows[1:]):
    ka,kb=I(a,"keyframe"),I(b,"keyframe")
    ba0=[F(a,"bax"),F(a,"bay"),F(a,"baz")]; ba1=[F(b,"bax"),F(b,"bay"),F(b,"baz")]
    bg0=[F(a,"bgx"),F(a,"bgy"),F(a,"bgz")]; bg1=[F(b,"bgx"),F(b,"bgy"),F(b,"bgz")]
    dba=norm([ba1[j]-ba0[j] for j in range(3)])
    dbg=norm([bg1[j]-bg0[j] for j in range(3)])
    steps.append((dba,dbg,kb))
for dba,dbg,k in sorted(steps,reverse=True)[:12]:
    print(f"KF={k:3d} dBA={dba:.5f} m/s^2 dBG={dbg:.6f} rad/s")

print("\n================ LOCAL SUMMARY ================")
loc=[bykf[k] for k in range(max(min(bykf),kf-8),min(max(bykf),kf+8)+1) if k in bykf]
if loc:
    ba_norm=[norm([F(r,'bax'),F(r,'bay'),F(r,'baz')]) for r in loc]
    bg_norm=[norm([F(r,'bgx'),F(r,'bgy'),F(r,'bgz')]) for r in loc]
    print(f"BA norm min/mean/max={min(ba_norm):.5f}/{statistics.mean(ba_norm):.5f}/{max(ba_norm):.5f} m/s^2")
    print(f"BG norm min/mean/max={min(bg_norm):.6f}/{statistics.mean(bg_norm):.6f}/{max(bg_norm):.6f} rad/s")
