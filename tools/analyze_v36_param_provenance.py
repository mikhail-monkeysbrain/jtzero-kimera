#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, json

ROOT=Path("/home/vio/jtzero_runs")
REPO=Path.home()/"jtzero-kimera-sync"

fresh=sorted(ROOT.glob("*_v34_FRESHSTART_4X"))
cont=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5"))
if not fresh: raise SystemExit("Не найден *_v34_FRESHSTART_4X")
if not cont: raise SystemExit("Не найден *_v25_TBS_ROLD_7P5")
FRESH=fresh[-1]
CONT=cont[-1]
CURRENT=REPO/"params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003/ImuParams.yaml"

def sha256(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def extract_tbs(p):
    txt=p.read_text(errors="replace")
    m=re.search(r"(?ms)^T_BS:\s*\n(?:^[ \t]+.*\n?)+",txt)
    return m.group(0).strip() if m else "T_BS NOT FOUND"

def meta(p):
    if not p.exists(): return {}
    out={}
    for line in p.read_text(errors="replace").splitlines():
        if "=" in line:
            k,v=line.split("=",1); out[k.strip()]=v.strip()
    return out

items=[("CURRENT",CURRENT),("CONT_ARCHIVE",CONT/"params/ImuParams.yaml")]
for n in range(1,5):
    items.append((f"FRESH_{n}_ARCHIVE",FRESH/f"fresh_{n}/params/ImuParams.yaml"))

print("="*150)
print("V36 — PARAMETER PROVENANCE / T_BS CHECK")
print("="*150)
print("T_BS = матрица, переводящая измерения IMU из системы координат датчика в систему координат корпуса.")
print("Цель: проверить, действительно ли fresh-start и continuous использовали одну и ту же геометрию IMU и один и тот же файл параметров.\n")

records=[]
for label,p in items:
    print("-"*150)
    print(f"{label}")
    print(f"path: {p}")
    if not p.exists():
        print("MISSING")
        records.append((label,None,None))
        continue
    h=sha256(p)
    t=extract_tbs(p)
    print(f"sha256: {h}")
    print(t)
    records.append((label,h,t))

print("\n"+"="*150)
print("HASH COMPARISON")
print("="*150)
base_h=records[1][1]
for label,h,t in records:
    print(f"{label:<20} same_as_CONT={h==base_h if h else False}  same_as_CURRENT={h==records[0][1] if h else False}")

print("\n"+"="*150)
print("ARCHIVE METADATA")
print("="*150)
print("CONT:")
for k,v in meta(CONT/"METADATA.txt").items():
    if k in ("label","timestamp","jtzero_branch","jtzero_head","params_dir","mode"):
        print(f"  {k}={v}")
for n in range(1,5):
    print(f"FRESH_{n}:")
    md=meta(FRESH/f"fresh_{n}/METADATA.txt")
    v34=meta(FRESH/f"fresh_{n}/V34_METADATA.txt")
    for k,v in md.items():
        if k in ("label","timestamp","jtzero_branch","jtzero_head","params_dir","mode"):
            print(f"  {k}={v}")
    for k in ("binary","binary_rebuilt_inside_loop","process_restart_between_passes"):
        if k in v34: print(f"  {k}={v34[k]}")

print("\n"+"="*150)
print("AUTOMATIC VERDICT")
print("="*150)
all_hash=[h for _,h,_ in records[1:] if h]
if all_hash and len(set(all_hash))==1:
    print("PARAMETERS: IDENTICAL — archived ImuParams.yaml is byte-identical in CONT and all FRESH runs.")
    print("T_BS mismatch is excluded as the cause of the V35 Z difference.")
else:
    print("PARAMETERS: DIFFERENT — at least one archived ImuParams.yaml differs.")
    print("V35 is confounded; do not use its bias conclusion until the differing parameter set is identified.")

tbs_values=[t for _,h,t in records[1:] if h]
if tbs_values and len(set(tbs_values))==1:
    print("T_BS: IDENTICAL — IMU→body geometry is the same across archived runs.")
else:
    print("T_BS: DIFFERENT — geometry differs across compared runs.")

print("\nNext interpretation:")
print("1) If parameters/T_BS differ: fix provenance first; V35 is invalid as a bias A/B comparison.")
print("2) If parameters/T_BS are identical: the Z change comes from runtime initialization/state, not static ImuParams.yaml.")
print("3) In case (2), next step is to compare startup meanAcc/initRPY/initBA and first 2 s of backend orientation between CONT pass1 and FRESH 1..4.")
