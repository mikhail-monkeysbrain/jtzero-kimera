#!/usr/bin/env python3
import csv,sys,statistics
from pathlib import Path
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(Path("/home/vio/jtzero_p11_latest_run.txt").read_text().strip())
r=list(csv.DictReader((root/"p11_raw_imu.csv").open())); out=[]
print("================ P11 RAW IMU A/B POSITION ================");print("run:",root)
for lab in ("A1","B1","A2","B2","A3","B3","A4"):
 x=[q for q in r if q["stage"]==lab and q["recording"]=="1"]
 if not x: continue
 tm=max(int(q["recv_ns"]) for q in x);x=[q for q in x if int(q["recv_ns"])>=tm-5_000_000_000]
 z=[float(q["az_flu"]) for q in x];n=[float(q["acc_norm"]) for q in x];t=[float(q["temperature"]) for q in x]
 out.append((lab,lab[0],statistics.mean(z),statistics.mean(n),statistics.mean(t)))
 print(f"{lab}: n={len(x)} Z={statistics.mean(z):+.6f} std={statistics.pstdev(z):.5f} |a|={statistics.mean(n):.6f} std={statistics.pstdev(n):.5f} T={statistics.mean(t):.2f}")
print()
for p in ("A","B"):
 x=[q for q in out if q[1]==p]
 if x: print(f"{p} mean: Z={statistics.mean(q[2] for q in x):+.6f} |a|={statistics.mean(q[3] for q in x):.6f} T={statistics.mean(q[4] for q in x):.2f}")
a=[q for q in out if q[1]=="A"];b=[q for q in out if q[1]=="B"]
if a and b: print(f"B-A: Z={statistics.mean(q[2] for q in b)-statistics.mean(q[2] for q in a):+.6f} |a|={statistics.mean(q[3] for q in b)-statistics.mean(q[3] for q in a):+.6f} m/s^2")
print("\nInterpretation: stable A/B split => position/mechanical state; monotonic sequence => time/temperature drift; overlap => no stable position effect.")
