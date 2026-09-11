#!/usr/bin/env python3
# JT-Zero — проверка оставшихся условий readyToUseOptFlow() по remote DataFlash BIN.
#
# Использует уже существующий parser из analyze_remote_xkv_gate.py и извлекает:
#   XKFS — реально активный source set;
#   XKF4 — solution status с высокой частотой, включая краткие выходы из constPos;
#   OF   — реальные обновления AP_OpticalFlow frontend (quality/flow/body rate).
#
# Параметры FC не изменяет.

from __future__ import annotations

import argparse
import glob
import os
import statistics
import sys
from collections import Counter, defaultdict

from analyze_remote_xkv_gate import read_formats, parse_records, key_time, pct


def last_remote_bin() -> str | None:
    files = glob.glob('/home/vio/jtzero_runs/*_REMOTE_LOG_XKV_DIAG/remote_ekf.bin')
    return max(files, key=os.path.getmtime) if files else None


def stat(v):
    if not v:
        return 'NO_DATA'
    return f'n={len(v)} min={min(v):.6f} p50={pct(v,0.5):.6f} p95={pct(v,0.95):.6f} max={max(v):.6f}'


def bits(f: int) -> str:
    names = [
        ('att',1),('velH',2),('velV',4),('posRel',8),('posAbs',16),
        ('posVAbs',32),('posVAGL',64),('constPos',128),('predRel',256),
        ('predAbs',512),('uninit',1024)
    ]
    return ' '.join(f'{n}={1 if f&m else 0}' for n,m in names)


def main() -> int:
    ap = argparse.ArgumentParser(description='Проверка remaining readyToUseOptFlow gates по remote BIN')
    ap.add_argument('bin', nargs='?', help='remote_ekf.bin; по умолчанию последний REMOTE_LOG_XKV_DIAG')
    args = ap.parse_args()
    path = args.bin or last_remote_bin()
    if not path:
        print('ОШИБКА: remote_ekf.bin не найден', file=sys.stderr)
        return 2

    with open(path,'rb') as f:
        data=f.read()
    fmts=read_formats(data)
    wanted={'XKFS','XKF4','OF','XKV1','XKV2','XKT'}
    rec=parse_records(data,fmts,wanted)
    for n in wanted:
        rec[n].sort(key=key_time)

    print('='*70)
    print('JT-ZERO — OPTICAL FLOW READY GATE ANALYSIS')
    print('='*70)
    print(f'BIN: {path}')
    print(f'bytes={len(data)} formats={len(fmts)}')
    for n in sorted(wanted):
        print(f'{n}: {len(rec[n])} records')

    print('\n===== ACTIVE SOURCE SET (XKFS) =====')
    if not rec['XKFS']:
        print('NO_DATA')
    else:
        by_core=defaultdict(list)
        for r in rec['XKFS']:
            by_core[int(r.get('C',0))].append(int(r.get('SS',-1)))
        for core, vals in sorted(by_core.items()):
            cnt=Counter(vals)
            print(f'core={core} source_set_counts={dict(sorted(cnt.items()))}')
        allsets=Counter(int(r.get('SS',-1)) for r in rec['XKFS'])
        if set(allsets)=={0}:
            print('SOURCE_SET_RUNTIME=PRIMARY_ONLY')
        else:
            print(f'SOURCE_SET_RUNTIME=NOT_PRIMARY_ONLY values={dict(sorted(allsets.items()))}')

    print('\n===== EKF SOLUTION STATUS FROM DATAFLASH (XKF4.SS) =====')
    if not rec['XKF4']:
        print('NO_DATA')
    else:
        by_core=defaultdict(list)
        for r in rec['XKF4']:
            by_core[int(r.get('C',0))].append(int(r.get('SS',0)))
        for core, vals in sorted(by_core.items()):
            cnt=Counter(vals)
            print(f'core={core} samples={len(vals)}')
            for val,n in cnt.most_common():
                print(f'  flags=0x{val:x} count={n} [{bits(val)}]')
            rel=sum(1 for v in vals if (v&8) and not (v&128))
            not_const=sum(1 for v in vals if not (v&128))
            print(f'  relative_samples={rel} non_constPos_samples={not_const}')

    print('\n===== AP_OPTICALFLOW FRONTEND LOG (OF) =====')
    if not rec['OF']:
        print('NO_DATA')
    else:
        q=[float(r.get('Qual',0)) for r in rec['OF']]
        fx=[float(r.get('flowX',0)) for r in rec['OF']]
        fy=[float(r.get('flowY',0)) for r in rec['OF']]
        bx=[float(r.get('bodyX',0)) for r in rec['OF']]
        by=[float(r.get('bodyY',0)) for r in rec['OF']]
        times=[key_time(r) for r in rec['OF'] if key_time(r)>0]
        dt_ms=[(b-a)/1000.0 for a,b in zip(times,times[1:]) if b>a]
        print(f'quality: {stat(q)}')
        print(f'flowX rad/s: {stat(fx)}')
        print(f'flowY rad/s: {stat(fy)}')
        print(f'bodyX rad/s: {stat(bx)}')
        print(f'bodyY rad/s: {stat(by)}')
        if dt_ms:
            print(f'frontend dt ms: {stat(dt_ms)}')
            gaps200=sum(1 for x in dt_ms if x>=200.0)
            print(f'gaps_ge_200ms={gaps200}/{len(dt_ms)}')
        print(f'quality_zero={sum(1 for x in q if x<=0)} quality_positive={sum(1 for x in q if x>0)}')

    print('\n===== VERDICT =====')
    primary_only=bool(rec['XKFS']) and all(int(r.get('SS',-1))==0 for r in rec['XKFS'])
    of_positive=bool(rec['OF']) and all(float(r.get('Qual',0))>0 for r in rec['OF'])
    times=[key_time(r) for r in rec['OF'] if key_time(r)>0]
    dts=[(b-a)/1000.0 for a,b in zip(times,times[1:]) if b>a]
    flow_frontend_fresh=bool(dts) and max(dts)<200.0
    any_nonconst=any((int(r.get('SS',0)) & 128)==0 for r in rec['XKF4'])
    any_rel=any((int(r.get('SS',0)) & 8) and not (int(r.get('SS',0)) & 128) for r in rec['XKF4'])

    print(f'PRIMARY_SOURCE_SET_LOGGED={"YES" if primary_only else "NO_OR_UNKNOWN"}')
    print(f'OF_FRONTEND_QUALITY_POSITIVE={"YES" if of_positive else "NO_OR_UNKNOWN"}')
    print(f'OF_FRONTEND_GAPS_LT_200MS={"YES" if flow_frontend_fresh else "NO_OR_UNKNOWN"}')
    print(f'XKF4_ANY_NON_CONSTPOS={"YES" if any_nonconst else "NO"}')
    print(f'XKF4_ANY_AID_RELATIVE_STATUS={"YES" if any_rel else "NO"}')
    print('NOTE: OF записи создаются AP_OpticalFlow::update_state после вызова AP::ahrs().writeOptFlowMeas().')
    print('При PRIMARY source set и EK3_SRC1_VELXY=5 source-gate должен быть открыт.')
    print('Если при этом XKF4 всегда constPos, остаётся искать различие между ожидаемым и фактическим flowMeaTime/вызовом setAidingMode, либо конкретный runtime source lookup по core.')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
