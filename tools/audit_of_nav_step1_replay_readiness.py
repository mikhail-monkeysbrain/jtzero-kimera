#!/usr/bin/env python3
"""Step 1/8: audit existing JT-Zero archives for offline optical-flow replay readiness.

No new physical run. This script only verifies whether an archive contains enough
camera/height/FC-motion information to build a deterministic offline estimator.
"""
import argparse,csv,json
from pathlib import Path

CANDIDATES = {
    "camera_forensic": [
        "jtzero_v43_camera_forensic.csv",
    ],
    "frontend": [
        "jtzero_500mm_v25_frontend.csv",
    ],
    "backend": [
        "jtzero_500mm_v25_backend.csv",
    ],
    "legs": [
        "jtzero_500mm_v25_legs.csv",
    ],
}

REQUIRED_COLUMN_GROUPS = {
    "camera_forensic": [
        ("affine translation", ["tx_px","ty_px"], ["tx","ty"]),
        ("height", ["height_m"], ["h_m"], ["height"]),
    ],
    "frontend": [
        ("timestamp", ["timestamp_ns"]),
        ("FC/PIM rotation evidence", ["pim_droll_deg","pim_dpitch_deg","pim_dyaw_deg"]),
    ],
    "backend": [
        ("timestamp", ["timestamp_ns"]),
        ("position", ["px_m","py_m"]),
    ],
    "legs": [
        ("leg bounds", ["start_settled_kf","end_press_kf"]),
    ],
}

def read_header(p):
    with p.open(newline="") as f:
        r=csv.reader(f)
        return next(r,[])

def find_file(run,names):
    for n in names:
        p=run/n
        if p.exists():
            return p
    return None

def has_variant(cols, variants):
    s=set(cols)
    for v in variants:
        if isinstance(v,list) and all(x in s for x in v):
            return True,v
    return False,None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--run",required=True)
    a=ap.parse_args()
    run=Path(a.run).expanduser()

    print("="*108)
    print("STEP 1/8 — OFFLINE OPTICAL-FLOW REPLAY READINESS AUDIT")
    print("="*108)
    print(f"run: {run}")

    if not run.is_dir():
        raise SystemExit("ERROR: run directory not found")

    ok=True
    found={}
    headers={}

    print("\nFILES")
    print("-"*108)
    for key,names in CANDIDATES.items():
        p=find_file(run,names)
        found[key]=p
        if p:
            headers[key]=read_header(p)
            print(f"{key:18s}: PASS  {p.name}  columns={len(headers[key])}")
        else:
            ok=False
            print(f"{key:18s}: MISS  expected one of {names}")

    print("\nCOLUMN GROUPS")
    print("-"*108)
    for key,groups in REQUIRED_COLUMN_GROUPS.items():
        cols=headers.get(key,[])
        if not cols:
            continue
        for label,*variants in groups:
            hit,which=has_variant(cols,variants)
            print(f"{key:18s} {label:28s}: {'PASS' if hit else 'MISS'}"
                  + (f"  {which}" if which else ""))
            ok &= hit

    print("\nIMPORTANT LIMITATION")
    print("-"*108)
    cf=headers.get("camera_forensic",[])
    explicit_frame_source=any(x in cf for x in ("frame_path","image_path","frame_file","image_file"))
    if explicit_frame_source:
        print("Recorded image/frame references: PASS")
    else:
        print("Recorded image/frame references: NOT FOUND in camera forensic CSV.")
        print("This does NOT block replay of already-computed affine/flow, but it DOES block")
        print("re-running a new pixel-level optical-flow algorithm from raw OV9281 images.")
        print("Step 2 will branch accordingly: reuse logged flow if raw frames are unavailable.")

    print("\nVERDICT")
    print("-"*108)
    if ok:
        print("PASS: archive is sufficient for an offline metric-motion prototype using existing logged visual motion + height + PIM/FC evidence.")
    else:
        print("PARTIAL: one or more required streams/columns are missing. Do not perform a new physical run yet; first inspect adjacent archived files.")
    print("="*108)

if __name__=="__main__":
    main()
