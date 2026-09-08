#!/usr/bin/env python3
from pathlib import Path
import hashlib, re

ROOT=Path("/home/vio/jtzero_runs")
fresh=sorted(ROOT.glob("*_v34_FRESHSTART_4X"))[-1]
cont=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5"))[-1]
current=Path.home()/"jtzero-kimera-sync/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"

FILES=[
 "ImuParams.yaml","LeftCameraParams.yaml","RightCameraParams.yaml",
 "BackendParams.yaml","FrontendParams.yaml","PipelineParams.yaml"
]

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
def tbs(p):
    if not p.exists(): return None
    s=p.read_text(errors="replace")
    m=re.search(r"(?ms)^T_BS:\s*\n(?:^[ \t]+.*\n?)+",s)
    return m.group(0).strip() if m else None

sets=[("CURRENT",current),("CONT",cont/"params")]
for i in range(1,5): sets.append((f"FRESH_{i}",fresh/f"fresh_{i}/params"))

print("="*170)
print("V38 — FULL PARAMETER / SENSOR-EXTRINSIC PROVENANCE")
print("="*170)
print("T_BS = преобразование координат конкретного датчика в систему координат корпуса.")
print("Важно: отдельный T_BS может быть у IMU, левой камеры и правой камеры.")
print("Цель: проверить не только ImuParams.yaml, но ВСЮ геометрию и основные параметры VIO.\n")

for fn in FILES:
    print("-"*170)
    print(fn)
    hs=[]
    for label,d in sets:
        p=d/fn
        h=sha(p); hs.append(h)
        print(f"{label:<10} exists={p.exists()} sha256={h or '-'}")
        tb=tbs(p)
        if tb:
            one=" ".join(x.strip() for x in tb.splitlines())
            print(f"           {one}")
    uniq={h for h in hs if h}
    print(f"VERDICT {fn}: {'IDENTICAL' if len(uniq)==1 and len(uniq)>0 else 'DIFFERENT/MISSING'}")

print("\n"+"="*170)
print("GLOBAL DIFFERENCES CONT vs CURRENT / FRESH")
print("="*170)
for fn in FILES:
    ch=sha(cont/"params"/fn)
    cur=sha(current/fn)
    freshh=[sha(fresh/f"fresh_{i}/params"/fn) for i in range(1,5)]
    print(f"{fn:<24} CONT==CURRENT {ch==cur}   CONT==ALL_FRESH {all(h==ch for h in freshh if h) and all(h is not None for h in freshh)}")

print("\n"+"="*170)
print("INTERPRETATION")
print("="*170)
print("1) Если LeftCameraParams.yaml отличается, прежний V36 был неполным: camera T_BS мог объяснить различие Z.")
print("2) Если только camera T_BS отличается, V35 нельзя использовать как чистый тест fresh-start vs carry-over.")
print("3) Если все основные YAML идентичны, тогда различие действительно runtime-only, и V37 становится валидным основанием искать initialization/fusion.")
print("4) Если несколько файлов отличаются, сначала нужен чистый A/B с одним изменяемым фактором; причинный вывод по V35 запрещён.")
