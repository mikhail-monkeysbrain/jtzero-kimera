#!/usr/bin/env python3
import csv, math, sys, bisect, statistics
from pathlib import Path

WINDOW_SEC=1.0

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def median(xs): return statistics.median(xs) if xs else float("nan")

if len(sys.argv)<2:
    raise SystemExit("usage: analyze_v25_imu_before_motion_after.py RUN_DIR [RUN_DIR ...]")

for arg in sys.argv[1:]:
    root=Path(arg)
    imu=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
    events=load(root/"jtzero_500mm_v25_events.csv")
    legs=load(root/"jtzero_500mm_v25_legs.csv")

    imu=sorted(imu,key=lambda r:I(r,"recv_ns"))
    ts=[I(r,"recv_ns") for r in imu]

    def window(t0,t1):
        i=bisect.bisect_left(ts,t0)
        j=bisect.bisect_right(ts,t1)
        return imu[i:j]

    ev={(I(r,"leg"),r["event"]):r for r in events}

    print("\n================ RUN ================")
    print(root)
    print("Uses raw FC HIGHRES_IMU only. No backend bias, no attitude transform.")
    print(f"before/after windows: {WINDOW_SEC:.1f}s around operator leg boundaries.")
    print()

    rows=[]
    for L in legs:
        leg=I(L,"leg")
        s=ev.get((leg,"START")); e=ev.get((leg,"END"))
        if not s or not e:
            print(f"LEG {leg}: missing START/END")
            continue
        t0=I(s,"event_wall_ns"); t1=I(e,"event_wall_ns")

        before=window(t0-int(WINDOW_SEC*1e9),t0)
        motion=window(t0,t1)
        after=window(t1,t1+int(WINDOW_SEC*1e9))

        def stats(rr):
            if not rr: return None
            z=[-F(r,"az") for r in rr]  # raw FRD -> FLU body Z
            n=[math.sqrt(F(r,"ax")**2+F(r,"ay")**2+F(r,"az")**2) for r in rr]
            return {
              "n":len(rr),
              "z_mean":mean(z),"z_med":median(z),
              "norm_mean":mean(n),"norm_med":median(n)
            }

        B,M,A=stats(before),stats(motion),stats(after)
        if not B or not M or not A:
            print(f"LEG {leg}: insufficient samples")
            continue

        dz_ba=A["z_med"]-B["z_med"]
        dn_ba=A["norm_med"]-B["norm_med"]
        dz_motion=M["z_mean"]-B["z_med"]
        dn_motion=M["norm_mean"]-B["norm_med"]

        rows.append({
          "leg":leg,"direction":L["direction"],
          "before_z":B["z_med"],"after_z":A["z_med"],
          "before_n":B["norm_med"],"after_n":A["norm_med"],
          "shift_z":dz_ba,"shift_n":dn_ba,
          "motion_z":dz_motion,"motion_n":dn_motion,
          "dur":(t1-t0)/1e9
        })

        print(f"LEG {leg} {L['direction']}: dur={(t1-t0)/1e9:.2f}s backend dz={F(L,'dz_m')*1000:+.1f}mm")
        print(f"  body-Z  before={B['z_med']:+.5f} motion_delta={dz_motion:+.5f} after={A['z_med']:+.5f}  before->after={dz_ba:+.5f} m/s^2")
        print(f"  |accel| before={B['norm_med']:+.5f} motion_delta={dn_motion:+.5f} after={A['norm_med']:+.5f}  before->after={dn_ba:+.5f} m/s^2")
        print()

    print("================ DIRECTION SUMMARY ================")
    for d in ("A->B","B->A"):
        rr=[r for r in rows if r["direction"]==d]
        if rr:
            print(f"{d}: motion body-Z={mean([r['motion_z'] for r in rr]):+.5f}  motion |a|={mean([r['motion_n'] for r in rr]):+.5f}  "
                  f"before->after body-Z={mean([r['shift_z'] for r in rr]):+.5f}  before->after |a|={mean([r['shift_n'] for r in rr]):+.5f} m/s^2")

    print()
    print("INTERPRETATION:")
    print("- motion changes while before≈after => motion-correlated physical IMU effect.")
    print("- before->after shift comparable to motion delta => baseline drift/state change is a major confounder.")
    print("- repeated direction-dependent before->after shifts imply memory/hysteresis, not a pure instantaneous direction effect.")
