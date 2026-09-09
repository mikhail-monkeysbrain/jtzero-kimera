#!/usr/bin/env python3
"""V44.18: compare the ungated regression, reference, and first gated run.

No new physical run is required.  This deliberately reuses the already archived
backend/frontend CSVs and the V44.15/V44.16 forensic scripts.
"""
import argparse
import subprocess
import sys
from pathlib import Path

def run(cmd):
    print("\n$ " + " ".join(map(str, cmd)), flush=True)
    p = subprocess.run(cmd)
    if p.returncode:
        raise SystemExit(p.returncode)

ap = argparse.ArgumentParser()
ap.add_argument("--reference", required=True)
ap.add_argument("--ungated", required=True)
ap.add_argument("--gated", required=True)
args = ap.parse_args()

root = Path(__file__).resolve().parent
for label, d in (("reference", args.reference), ("ungated", args.ungated), ("gated", args.gated)):
    p = Path(d).expanduser()
    if not p.is_dir():
        raise SystemExit(f"ERROR: {label} archive not found: {p}")

print("=" * 110)
print("V44.18 — FIRST GATED RUN EFFECT / NO NEW PHYSICAL RUN")
print("=" * 110)
print("Purpose:")
print("  1) test whether the kf89-style VALID geometry discontinuity disappeared;")
print("  2) separate that effect from the later LOW_DISPARITY tail;")
print("  3) avoid interpreting 457.6 mm alone as proof that the gate worked.")
print()
print("NOTE: the terminal log contains no [JTZERO-MONO-POSE-GATE] rejection line.")
print("Therefore gate activation must be verified from archived geometry, not inferred from endpoint improvement.")

py = sys.executable

print("\n===== A. REFERENCE vs FIRST GATED RUN: VALID POSE DISCONTINUITY =====")
run([py, str(root / "analyze_v44_15_pose_discontinuity.py"),
     "--reference", args.reference, "--current", args.gated])

print("\n===== B. UNGATED REGRESSION vs FIRST GATED RUN: VALID POSE DISCONTINUITY =====")
run([py, str(root / "analyze_v44_15_pose_discontinuity.py"),
     "--reference", args.ungated, "--current", args.gated])

print("\n===== C. REFERENCE vs FIRST GATED RUN: PIM/GATE SCREEN =====")
run([py, str(root / "analyze_v44_16_pose_pim_gate_screen.py"),
     "--reference", args.reference, "--current", args.gated])

print("\n===== D. REFERENCE vs FIRST GATED RUN: LATE REVERSAL =====")
run([py, str(root / "analyze_v44_13_backend_reversal.py"),
     "--reference", args.reference, "--current", args.gated, "--tail-start", "0.65"])

print("\n" + "=" * 110)
print("INTERPRETATION RULE")
print("=" * 110)
print("If V44.15 still reports a >=30deg jump in the gated archive, the runtime gate did NOT catch the")
print("same quantity used by the forensic CSV; do not tune thresholds and do not claim a gate success.")
print("If the large VALID jump is gone but no gate log exists, inspect coordinate/measurement logging before")
print("another run: disappearance may be ordinary run-to-run variation.")
print("If a gate event is demonstrated and late reversal shrinks, then the pre-fusion rejection branch is supported.")
