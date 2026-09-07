#!/usr/bin/env python3
import csv
import math
import statistics
import sys
from pathlib import Path


def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))


def I(r,k):
    return int(float(r[k]))


def F(r,k):
    return float(r[k])


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def sub(a,b):
    return tuple(x-y for x,y in zip(a,b))


def mean(xs):
    return statistics.mean(xs) if xs else float("nan")


def pearson(xs, ys):
    if len(xs) < 3:
        return float("nan")
    mx, my = mean(xs), mean(ys)
    vx = sum((x-mx)**2 for x in xs)
    vy = sum((y-my)**2 for y in ys)
    if vx <= 0 or vy <= 0:
        return float("nan")
    return sum((x-mx)*(y-my) for x,y in zip(xs,ys)) / math.sqrt(vx*vy)


if len(sys.argv) < 2:
    raise SystemExit("usage: analyze_v25_interleg_state_inheritance.py RUN_DIR [RUN_DIR ...]")

rows = []

print("================ V25 INTER-LEG STATE INHERITANCE ================")
print("Uses archived backend + legs only.")
print("For each transition, compares previous settled END state with next settled START state,")
print("then relates inherited START state to the NEXT leg scale.")
print("No backend speed is used as evidence of physical operator motion.")

for arg in sys.argv[1:]:
    root = Path(arg)
    back = load(root/"jtzero_500mm_v25_backend.csv")
    legs = load(root/"jtzero_500mm_v25_legs.csv")
    bykf = {I(r,"keyframe"): r for r in back}
    legs = sorted(legs, key=lambda r:I(r,"leg"))

    print("\nRUN:", root)

    # Per-leg starts/ends first.
    per = {}
    for L in legs:
        leg = I(L,"leg")
        ks = I(L,"start_settled_kf")
        ke = I(L,"end_settled_kf")
        s = bykf.get(ks)
        e = bykf.get(ke)
        if not s or not e:
            print(f"LEG {leg}: missing settled backend row")
            continue

        start_ba = (F(s,"bax"),F(s,"bay"),F(s,"baz"))
        end_ba = (F(e,"bax"),F(e,"bay"),F(e,"baz"))
        start_bg = (F(s,"bgx"),F(s,"bgy"),F(s,"bgz"))
        end_bg = (F(e,"bgx"),F(e,"bgy"),F(e,"bgz"))
        start_v = (F(s,"vx_m_s"),F(s,"vy_m_s"),F(s,"vz_m_s"))
        end_v = (F(e,"vx_m_s"),F(e,"vy_m_s"),F(e,"vz_m_s"))
        start_att = (F(s,"roll_deg"),F(s,"pitch_deg"))
        end_att = (F(e,"roll_deg"),F(e,"pitch_deg"))

        rec = dict(
            leg=leg,
            direction=L["direction"],
            scale=F(L,"scale_horizontal"),
            start_kf=ks,
            end_kf=ke,
            start_ba=start_ba,
            end_ba=end_ba,
            start_bg=start_bg,
            end_bg=end_bg,
            start_v=start_v,
            end_v=end_v,
            start_att=start_att,
            end_att=end_att,
        )
        per[leg]=rec

        print(
            f"LEG {leg} {rec['direction']}: scale={rec['scale']:.4f} "
            f"START BA=[{start_ba[0]:+.4f},{start_ba[1]:+.4f},{start_ba[2]:+.4f}] "
            f"|V|={norm(start_v)*1000:.1f}mm/s RP=[{start_att[0]:+.2f},{start_att[1]:+.2f}]"
        )
        print(
            f"                 END   BA=[{end_ba[0]:+.4f},{end_ba[1]:+.4f},{end_ba[2]:+.4f}] "
            f"|V|={norm(end_v)*1000:.1f}mm/s RP=[{end_att[0]:+.2f},{end_att[1]:+.2f}]"
        )

    print("\n---- transitions ----")
    for a,b in zip(legs,legs[1:]):
        la, lb = I(a,"leg"), I(b,"leg")
        if la not in per or lb not in per:
            continue
        A, B = per[la], per[lb]

        dba = sub(B["start_ba"], A["end_ba"])
        dbg = sub(B["start_bg"], A["end_bg"])
        dv = sub(B["start_v"], A["end_v"])
        drp = (
            B["start_att"][0]-A["end_att"][0],
            B["start_att"][1]-A["end_att"][1],
        )

        # What the next leg actually inherits at its settled start.
        ba_mag = norm(B["start_ba"])
        v_mag = norm(B["start_v"])
        rp_mag = math.hypot(B["start_att"][0], B["start_att"][1])

        rec = dict(
            run=root.name,
            prev_leg=la,
            next_leg=lb,
            next_direction=B["direction"],
            next_scale=B["scale"],
            carry_dba=norm(dba),
            carry_dbg=norm(dbg),
            carry_dv_mm_s=norm(dv)*1000.0,
            carry_drp_deg=math.hypot(*drp),
            next_bax=B["start_ba"][0],
            next_bay=B["start_ba"][1],
            next_baz=B["start_ba"][2],
            next_ba_mag=ba_mag,
            next_vx=B["start_v"][0],
            next_vy=B["start_v"][1],
            next_vz=B["start_v"][2],
            next_v_mag_mm_s=v_mag*1000.0,
            next_roll=B["start_att"][0],
            next_pitch=B["start_att"][1],
            next_rp_mag=rp_mag,
        )
        rows.append(rec)

        print(
            f"L{la}->L{lb} next={B['direction']} scale={B['scale']:.4f} "
            f"ENDprev->STARTnext: |dBA|={rec['carry_dba']:.5f}m/s2 "
            f"|dBG|={rec['carry_dbg']:.6f}rad/s "
            f"|dV|={rec['carry_dv_mm_s']:.1f}mm/s |dRP|={rec['carry_drp_deg']:.3f}deg"
        )
        print(
            f"  inherited START: BA=[{rec['next_bax']:+.4f},{rec['next_bay']:+.4f},{rec['next_baz']:+.4f}] "
            f"|BA|={rec['next_ba_mag']:.4f} "
            f"V=[{rec['next_vx']*1000:+.1f},{rec['next_vy']*1000:+.1f},{rec['next_vz']*1000:+.1f}]mm/s "
            f"RP=[{rec['next_roll']:+.2f},{rec['next_pitch']:+.2f}]"
        )

print("\n================ ASSOCIATION WITH NEXT-LEG SCALE ================")
metrics = [
    "carry_dba","carry_dbg","carry_dv_mm_s","carry_drp_deg",
    "next_bax","next_bay","next_baz","next_ba_mag",
    "next_vx","next_vy","next_vz","next_v_mag_mm_s",
    "next_roll","next_pitch","next_rp_mag",
]
for m in metrics:
    xs=[r[m] for r in rows]
    ys=[r["next_scale"] for r in rows]
    print(f"Pearson({m}, next_scale)={pearson(xs,ys):+.3f}")

print("\n================ DIRECTION-SPLIT TRANSITIONS ================")
for d in ("A->B","B->A"):
    rr=[r for r in rows if r["next_direction"]==d]
    if not rr:
        continue
    print(f"{d}: n={len(rr)} mean next_scale={mean([r['next_scale'] for r in rr]):.4f} "
          f"mean inherited |BA|={mean([r['next_ba_mag'] for r in rr]):.4f} "
          f"mean |V|={mean([r['next_v_mag_mm_s'] for r in rr]):.2f}mm/s "
          f"mean RPmag={mean([r['next_rp_mag'] for r in rr]):.3f}deg")

print("\nINTERPRETATION:")
print("- Small ENDprev->STARTnext deltas mean the next leg starts from nearly the state left by the previous leg: strong state inheritance.")
print("- Large deltas mean stationary settling substantially re-estimates/resets the state before the next leg.")
print("- Correlation here is diagnostic only; transitions from the same physical run are not independent.")
print("- If next-leg scale tracks inherited BA/attitude/state more strongly than frontend quality did, run/order state coupling strengthens.")
print("- If inherited state is nearly reset and does not track scale, look elsewhere before designing a reset-between-legs physical test.")
