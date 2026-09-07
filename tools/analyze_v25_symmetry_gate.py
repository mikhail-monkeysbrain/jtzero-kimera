#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def std(xs):
    m=mean(xs)
    return math.sqrt(mean([(x-m)*(x-m) for x in xs])) if xs else float("nan")

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_symmetry_gate.py RUN_DIR")

root=Path(sys.argv[1])
imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
att=load(root/"jtzero_500mm_v25_attitude.csv")
events=load(root/"jtzero_500mm_v25_events.csv")
legs=load(root/"jtzero_500mm_v25_legs.csv")

ev={(I(r,"leg"),r["event"]):r for r in events}

print("================ V25 SYMMETRY GATE ================")
print("run:",root)
print("Goal: judge whether A->B and B->A were physically similar enough for causal comparison.")
print()

rows=[]
for L in legs:
    leg=I(L,"leg")
    s=ev.get((leg,"START")); e=ev.get((leg,"END"))
    if not s or not e: continue
    t0,t1=I(s,"event_wall_ns"),I(e,"event_wall_ns")
    seg=[r for r in imu if t0<=I(r,"recv_ns")<=t1]
    aseg=[r for r in att if t0<=I(r,"recv_ns")<=t1]
    dur=(t1-t0)/1e9
    ax=[F(r,"ax") for r in seg]; ay=[F(r,"ay") for r in seg]; az=[F(r,"az") for r in seg]
    h=[math.hypot(x,y) for x,y in zip(ax,ay)]
    yaws=[F(r,"yaw_deg") for r in aseg] if aseg else []
    row={
      "leg":leg,"direction":L["direction"],"dur":dur,
      "h_rms":math.sqrt(mean([x*x for x in h])) if h else float("nan"),
      "az_mean":mean(az),
      "az_std":std(az),
      "yaw_span":(max(yaws)-min(yaws)) if yaws else float("nan"),
      "scale":F(L,"scale_horizontal"),"dz_mm":F(L,"dz_m")*1000.0
    }
    rows.append(row)
    print(f"LEG {leg} {row['direction']}: dur={dur:.2f}s hAccRMS={row['h_rms']:.4f} yawSpan={row['yaw_span']:.3f}deg scale={row['scale']:.4f} dz={row['dz_mm']:+.1f}mm")

print()
print("================ PAIR SYMMETRY ================")
pairs=[(1,2),(3,4)]
for a,b in pairs:
    A=next((r for r in rows if r["leg"]==a),None)
    B=next((r for r in rows if r["leg"]==b),None)
    if not A or not B: continue
    dur_rel=abs(A["dur"]-B["dur"])/max(A["dur"],B["dur"])
    h_rel=abs(A["h_rms"]-B["h_rms"])/max(A["h_rms"],B["h_rms"])
    yaw_diff=abs(A["yaw_span"]-B["yaw_span"])
    # Conservative diagnostic gate, not a product requirement.
    passed = dur_rel<=0.20 and h_rel<=0.25 and yaw_diff<=1.0
    print(f"PAIR {a}<->{b}: duration rel diff={dur_rel*100:.1f}%  hAccRMS rel diff={h_rel*100:.1f}%  yawSpan diff={yaw_diff:.2f}deg  => {'PASS' if passed else 'FAIL'}")

print()
print("INTERPRETATION:")
print("- PASS means the pair is similar enough to use as evidence about direction.")
print("- FAIL means direction is still confounded by how the motion was performed.")
print("- This is a diagnostic gate only; it is not a P11 acceptance criterion.")
