#!/usr/bin/env python3
"""V44.19 — archive-only gate causality check.

The first V44.17d run improved, but produced no runtime rejection marker.
This script compares the archived frontend/backend geometry and explicitly
separates "candidate absent" from "gate proven to have fired".
"""
import argparse, csv, math
from pathlib import Path

def f(x, default=float("nan")):
    try: return float(x)
    except: return default

def load_csv(run, stem):
    p=Path(run).expanduser()/stem
    if not p.exists(): raise SystemExit(f"ERROR: missing {p}")
    with p.open(newline="") as h: return list(csv.DictReader(h))

def pick(row,*names):
    for n in names:
        if n in row and row[n] not in ("",None): return row[n]
    return ""

ap=argparse.ArgumentParser()
ap.add_argument("--ungated",required=True)
ap.add_argument("--gated",required=True)
a=ap.parse_args()

def summarize(run):
    fr=load_csv(run,"jtzero_500mm_v25_frontend.csv")
    be=load_csv(run,"jtzero_500mm_v25_backend.csv")
    valid=low=0; max_tilt=0.0; first_low=None
    for i,r in enumerate(fr):
        st=pick(r,"status","tracking_status","mono_status")
        if st=="VALID": valid+=1
        if st=="LOW_DISPARITY":
            low+=1
            if first_low is None: first_low=i
        tx=f(pick(r,"mono_tx","tx","t_x")); ty=f(pick(r,"mono_ty","ty","t_y")); tz=f(pick(r,"mono_tz","tz","t_z"))
        if all(math.isfinite(v) for v in (tx,ty,tz)):
            h=math.hypot(tx,ty)
            if h>1e-12: max_tilt=max(max_tilt,math.degrees(math.atan2(abs(tz),h)))
    return fr,be,valid,low,max_tilt,first_low

u=summarize(a.ungated); g=summarize(a.gated)
print("="*104)
print("V44.19 — GATE RUNTIME EVIDENCE / ARCHIVE ONLY")
print("="*104)
print(f"UNGATED frontend rows={len(u[0])} VALID={u[2]} LOW_DISPARITY={u[3]} max_local_tilt={u[4]:.1f}deg")
print(f"GATED   frontend rows={len(g[0])} VALID={g[2]} LOW_DISPARITY={g[3]} max_local_tilt={g[4]:.1f}deg")
print()
print("KNOWN FROM V44.18:")
print("  ungated regression contained one VALID 63.2deg direction jump / 56.5deg tilt event")
print("  first gated archive contains no >=30deg VALID geometry event")
print("  first gated archive still develops a LOW_DISPARITY tail and ~12.4mm post-peak loss")
print()
print("CAUSALITY:")
print("  Absence of the anomaly is compatible with the gate, but is NOT proof that the gate fired.")
print("  No [JTZERO-MONO-POSE-GATE] runtime rejection was observed in the captured terminal log.")
print("  Therefore do not attribute the 457.6mm endpoint improvement to the gate yet.")
print()
print("NEXT IMPLEMENTATION REQUIREMENT:")
print("  instrument the gate with persistent counters: evaluated, rejected, max_jump_deg, max_tilt_deg,")
print("  and print a shutdown summary even when rejected=0. This removes dependence on a transient log line.")
print("="*104)
