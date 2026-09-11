#!/usr/bin/env python3
import csv, sys, math
from pathlib import Path

if len(sys.argv) != 2:
    print(f"Использование: {sys.argv[0]} <GROUND_MOTION_REPLAY_DATASET>")
    sys.exit(2)

dirp = Path(sys.argv[1])
frames_p = dirp / "frames.csv"
events_p = dirp / "events.csv"
detail_p = dirp / "replay_detail.csv"

if not frames_p.exists() or not events_p.exists():
    raise SystemExit("Нет frames.csv или events.csv")

with events_p.open(newline='') as f:
    ev = {r['event']: int(r['mono_ns']) for r in csv.DictReader(f) if r.get('event')}
if 'MOVE_START' not in ev or 'MOVE_END' not in ev:
    raise SystemExit("В events.csv нет MOVE_START/MOVE_END")

a, b = ev['MOVE_START'], ev['MOVE_END']
with frames_p.open(newline='') as f:
    rows = list(csv.DictReader(f))

def yes(r, k):
    try: return int(r.get(k, '0')) != 0
    except: return False

def ns(r): return int(r['mono_ns'])
def pct(n,d): return 100.0*n/d if d else 0.0

def section(name, rr):
    n=len(rr)
    dec=sum(yes(r,'decode_ok') for r in rr)
    lun=sum(yes(r,'luna_valid') for r in rr)
    att=sum(yes(r,'att_valid') for r in rr)
    sen=sum(yes(r,'sensors_valid') for r in rr)
    print(f"{name:12s} rows={n:5d} decode={dec:5d} ({pct(dec,n):5.1f}%)  luna={lun:5d} ({pct(lun,n):5.1f}%)  att={att:5d} ({pct(att,n):5.1f}%)  sensors={sen:5d} ({pct(sen,n):5.1f}%)")

move=[r for r in rows if a <= ns(r) <= b]
print("===== REPLAY CAPTURE INTEGRITY =====")
section("ALL", rows)
section("MOVE", move)

if rows:
    dur=(ns(rows[-1])-ns(rows[0]))*1e-9
    fps=(len(rows)-1)/dur if dur>0 else float('nan')
    print(f"duration={dur:.3f}s effective_capture_rate={fps:.2f} Hz")

# Длины серий sensors_valid / invalid в MOVE.
runs=[]
if move:
    cur=yes(move[0],'sensors_valid'); start=0
    for i in range(1,len(move)):
        v=yes(move[i],'sensors_valid')
        if v!=cur:
            runs.append((cur,start,i-1))
            cur=v; start=i
    runs.append((cur,start,len(move)-1))
invalid_runs=[x for x in runs if not x[0]]
if invalid_runs:
    lens=[j-i+1 for _,i,j in invalid_runs]
    maxrun=max(invalid_runs,key=lambda x:x[2]-x[1])
    i,j=maxrun[1],maxrun[2]
    t=(ns(move[j])-ns(move[i]))*1e-3/1000.0
    print(f"MOVE sensors-invalid runs={len(invalid_runs)} max_frames={max(lens)} approx_span_ms={t:.1f}")
else:
    print("MOVE sensors-invalid runs=0")

if 'sync_timeout' in (rows[0] if rows else {}):
    st=sum(yes(r,'sync_timeout') for r in rows)
    waits=[]
    for r in rows:
        try: waits.append(float(r.get('sync_wait_ms','nan')))
        except: pass
    waits=[x for x in waits if math.isfinite(x)]
    print(f"sync_timeout={st}/{len(rows)} ({pct(st,len(rows)):.1f}%)")
    if waits:
        waits.sort()
        def q(fr): return waits[min(len(waits)-1,int(fr*(len(waits)-1)))]
        print(f"sync_wait_ms p50={q(.50):.1f} p95={q(.95):.1f} max={max(waits):.1f}")

if detail_p.exists():
    with detail_p.open(newline='') as f:
        dr=list(csv.DictReader(f))
    base=[r for r in dr if r.get('scenario')=='BASELINE_CURRENT']
    base_move=[]
    # detail не содержит mono_ns; сопоставляем frame -> mono_ns.
    fmap={r['frame']: ns(r) for r in rows}
    for r in base:
        t=fmap.get(r.get('frame',''))
        if t is not None and a <= t <= b:
            base_move.append(r)
    from collections import Counter
    ca=Counter(r.get('reason','?') for r in base)
    cm=Counter(r.get('reason','?') for r in base_move)
    print("\nBASELINE_CURRENT reasons ALL:")
    print("  " + "  ".join(f"{k}={v}" for k,v in ca.most_common()))
    print("BASELINE_CURRENT reasons MOVE:")
    print("  " + "  ".join(f"{k}={v}" for k,v in cm.most_common()))

    masks=[r for r in dr if r.get('forced_reject')=='1']
    forced_valid=sum(r.get('reason')=='SYNTH_REJECT' for r in masks)
    print(f"\nforced-reject rows={len(masks)}, actually-hit-valid-step={forced_valid}")

move_sensor = sum(yes(r,'sensors_valid') for r in move)
if move and pct(move_sensor,len(move)) < 90.0:
    print("\nVERDICT: CAPTURE INVALID FOR REPLAY — sensors_valid в MOVE < 90%.")
    print("Причины visual dropout/anchor по этому dataset оценивать нельзя.")
else:
    print("\nVERDICT: sensor coverage пригодно для visual replay; дальше смотреть BASELINE displacement и причины visual invalid.")
