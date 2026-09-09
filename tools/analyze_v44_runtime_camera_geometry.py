#!/usr/bin/env python3
"""V44.3: no-motion runtime camera geometry audit."""
import argparse
import re
import subprocess

FX=568.53170752165227
FY=569.68005562865858
CAL_W=640
CAL_H=480

def run(cmd):
    p=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    return p.returncode,p.stdout

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--device",default="/dev/video0")
    ap.add_argument("--distance-mm",type=float,default=185.5)
    a=ap.parse_args()

    print("="*112)
    print("V44.3 — RUNTIME OV9281 GEOMETRY AUDIT (NO A->B)")
    print("="*112)
    print(f"calibration: {CAL_W}x{CAL_H} fx={FX:.3f} fy={FY:.3f}")
    print(f"physical sensor-plane height: {a.distance_mm:.2f} mm")

    rc,info=run(["v4l2-ctl","-d",a.device,"--all"])
    print("\nV4L2 --all")
    print("-"*112)
    print(info.rstrip())
    if rc:
        raise SystemExit("v4l2-ctl --all failed")

    rc_fmt,fmt=run(["v4l2-ctl","-d",a.device,"--get-fmt-video"])
    print("\nV4L2 ACTIVE FORMAT")
    print("-"*112)
    print(fmt.rstrip())
    if rc_fmt:
        raise SystemExit("v4l2 active-format query failed")

    rc_parm,parm=run(["v4l2-ctl","-d",a.device,"--get-parm"])
    print("\nV4L2 FRAME INTERVAL")
    print("-"*112)
    print(parm.rstrip())
    if rc_parm:
        print("NOTE: VIDIOC_G_PARM unsupported on this rp1-cfe raw capture node; this is non-fatal.")

    m=re.search(r"Width/Height\s*:\s*(\d+)\s*/\s*(\d+)",fmt)
    if m:
        w,h=map(int,m.groups())
        print(f"\nactive runtime resolution parsed: {w}x{h}")
        if (w,h)==(CAL_W,CAL_H):
            print("resolution identity: PASS")
        else:
            print(f"resolution identity: FAIL sx={w/CAL_W:.6f} sy={h/CAL_H:.6f}")
    else:
        print("\nWARNING: could not parse active Width/Height")

    rc,formats=run(["v4l2-ctl","-d",a.device,"--list-formats-ext"])
    print("\nSUPPORTED MODES")
    print("-"*112)
    print(formats.rstrip())

    print("\nNEXT DISCRIMINATOR")
    print("-"*112)
    print("If runtime is exactly 640x480 with no obvious mismatch, the next test is an independent static")
    print("effective-focal measurement using a planar target at the measured 185-186 mm sensor-plane distance.")
    print("No 500-mm A->B pass is required.")
    print("="*112)

if __name__=="__main__":
    main()
