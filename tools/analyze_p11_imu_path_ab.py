#!/usr/bin/env python3
import csv,sys,statistics
from pathlib import Path
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(Path("/home/vio/jtzero_p11_latest_imu_path_run.txt").read_text().strip())
rows=list(csv.DictReader((root/"p11_imu_path.csv").open()))
types=("RAW_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3","HIGHRES_IMU")
print("================ P11 IMU PATH A/B ================");print("run:",root)
for typ in types:
    rr=[r for r in rows if r["type"]==typ and r["recording"]=="1"]
    if not rr:
        print(f"\n[{typ}] НЕТ ДАННЫХ"); continue
    vals=[]
    print(f"\n[{typ}]")
    for lab in ("A1","B1","A2","B2","A3","B3","A4"):
        x=[r for r in rr if r["stage"]==lab]
        if not x: continue
        tm=max(int(r["recv_ns"]) for r in x);x=[r for r in x if int(r["recv_ns"])>=tm-5_000_000_000]
        n=[float(r["acc_norm_si"]) for r in x];z=[float(r["az_si"]) for r in x]
        nm=statistics.mean(n);zm=statistics.mean(z);vals.append((lab,lab[0],nm,zm))
        print(f"{lab}: n={len(x):4d} |a|={nm:.6f} Z={zm:+.6f} std|a|={statistics.pstdev(n):.5f}")
    a=[v for v in vals if v[1]=="A"];b=[v for v in vals if v[1]=="B"]
    if a and b:
        am=statistics.mean(v[2] for v in a);bm=statistics.mean(v[2] for v in b)
        az=statistics.mean(v[3] for v in a);bz=statistics.mean(v[3] for v in b)
        print(f"A mean |a|={am:.6f}  B mean |a|={bm:.6f}  B-A={bm-am:+.6f} m/s^2")
        print(f"A mean Z={az:+.6f}    B mean Z={bz:+.6f}    B-A={bz-az:+.6f} m/s^2")
print("\nINTERPRETATION:")
print("- Split in HIGHRES only, absent in SCALED/RAW => difference is introduced later in FC output path.")
print("- Similar split in SCALED and HIGHRES => effect exists before HIGHRES formatting/final processing.")
print("- Similar split across RAW/SCALED/HIGHRES => effect is already present in the FC IMU measurement path; this still does not prove direct sensor ADC causality.")
print("- Different SCALED_IMU vs SCALED_IMU2 behavior may identify an IMU-instance-specific effect.")
