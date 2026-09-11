#!/usr/bin/env python3
# JT-Zero — анализ gyro-bias gate по удалённому DataFlash BIN ArduPilot.
#
# Разбирает FMT динамически, извлекает XKV1/XKV2/XKT/XKF1 и сравнивает
# covariance gyro-bias с порогом delAngBiasLearned из ArduCopter 4.7.0.

from __future__ import annotations

import argparse
import bisect
import glob
import math
import os
import struct
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple, Any, Optional

MAGIC = b"\xA3\x95"
FMT_MSG_ID = 128
FMT_LEN = 89

FMT_SPECS = {
    'a': ('64s', 64, None),
    'b': ('b', 1, None),
    'B': ('B', 1, None),
    'h': ('h', 2, None),
    'H': ('H', 2, None),
    'i': ('i', 4, None),
    'I': ('I', 4, None),
    'f': ('f', 4, None),
    'd': ('d', 8, None),
    'n': ('4s', 4, None),
    'N': ('16s', 16, None),
    'Z': ('64s', 64, None),
    'c': ('h', 2, 0.01),
    'C': ('H', 2, 0.01),
    'e': ('i', 4, 0.01),
    'E': ('I', 4, 0.01),
    'L': ('i', 4, 1.0e-7),
    'M': ('B', 1, None),
    'q': ('q', 8, None),
    'Q': ('Q', 8, None),
}

@dataclass
class MsgFormat:
    msgid: int
    length: int
    name: str
    fmt: str
    labels: List[str]


def cstr(b: bytes) -> str:
    return b.split(b'\0', 1)[0].decode('ascii', errors='ignore')


def read_formats(data: bytes) -> Dict[int, MsgFormat]:
    out: Dict[int, MsgFormat] = {}
    needle = MAGIC + bytes([FMT_MSG_ID])
    i = 0
    while True:
        i = data.find(needle, i)
        if i < 0:
            break
        if i + FMT_LEN > len(data):
            break
        try:
            msgid, length, name_b, fmt_b, labels_b = struct.unpack_from('<BB4s16s64s', data, i + 3)
        except struct.error:
            i += 1
            continue
        name = cstr(name_b)
        fmt = cstr(fmt_b)
        labels_s = cstr(labels_b)
        if 3 <= length <= 255 and name and all(32 <= ord(ch) < 127 for ch in name):
            out[msgid] = MsgFormat(msgid, length, name, fmt, labels_s.split(',') if labels_s else [])
            i += FMT_LEN
        else:
            i += 1
    return out


def decode_payload(mf: MsgFormat, payload: bytes) -> Optional[Dict[str, Any]]:
    values: List[Any] = []
    off = 0
    try:
        for ch in mf.fmt:
            if ch not in FMT_SPECS:
                return None
            sfmt, size, scale = FMT_SPECS[ch]
            if off + size > len(payload):
                return None
            v = struct.unpack_from('<' + sfmt, payload, off)[0]
            off += size
            if isinstance(v, (bytes, bytearray)):
                v = cstr(bytes(v))
            elif scale is not None:
                v = float(v) * scale
            values.append(v)
    except (struct.error, ValueError):
        return None

    labels = mf.labels
    if len(labels) != len(values):
        return {f'F{i}': v for i, v in enumerate(values)}
    return dict(zip(labels, values))


def parse_records(data: bytes, formats: Dict[int, MsgFormat], wanted: set[str]) -> Dict[str, List[Dict[str, Any]]]:
    by_name: Dict[str, List[Dict[str, Any]]] = {n: [] for n in wanted}
    i = 0
    n = len(data)
    while i + 3 <= n:
        if data[i:i+2] != MAGIC:
            i += 1
            continue
        msgid = data[i+2]
        if msgid == FMT_MSG_ID:
            length = FMT_LEN
        else:
            mf = formats.get(msgid)
            if mf is None:
                i += 1
                continue
            length = mf.length
        if length < 3 or i + length > n:
            i += 1
            continue
        if msgid != FMT_MSG_ID:
            mf = formats[msgid]
            if mf.name in wanted:
                rec = decode_payload(mf, data[i+3:i+length])
                if rec is not None:
                    by_name[mf.name].append(rec)
        i += length
    return by_name


def key_time(rec: Dict[str, Any]) -> int:
    return int(rec.get('TimeUS', 0))


def nearest(records: List[Dict[str, Any]], t_us: int, max_dt_us: int = 100_000) -> Optional[Dict[str, Any]]:
    if not records:
        return None
    times = [key_time(r) for r in records]
    k = bisect.bisect_left(times, t_us)
    cand = []
    if k < len(records):
        cand.append(records[k])
    if k > 0:
        cand.append(records[k-1])
    if not cand:
        return None
    r = min(cand, key=lambda x: abs(key_time(x)-t_us))
    return r if abs(key_time(r)-t_us) <= max_dt_us else None


def nav_to_body_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[List[float]]:
    # R_bn: body -> NED, стандартная 3-2-1; prevTnb в EKF = nav -> body = R_bn^T.
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    r_bn = [
        [cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
        [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy],
        [-sp,   sr*cp,            cr*cp],
    ]
    return [[r_bn[j][i] for j in range(3)] for i in range(3)]


def mat_vec(m: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return tuple(sum(m[i][j]*v[j] for j in range(3)) for i in range(3))  # type: ignore


def pct(v: List[float], q: float) -> float:
    if not v:
        return float('nan')
    s = sorted(v)
    if len(s) == 1:
        return s[0]
    x = (len(s)-1)*q
    lo = int(math.floor(x)); hi = int(math.ceil(x))
    if lo == hi:
        return s[lo]
    return s[lo]*(hi-x) + s[hi]*(x-lo)


def main() -> int:
    ap = argparse.ArgumentParser(description='Анализ XKV gyro-bias gate из удалённого DataFlash BIN')
    ap.add_argument('bin', nargs='?', help='remote_ekf.bin; если не указан — берётся последний REMOTE_LOG_XKV_DIAG')
    args = ap.parse_args()

    path = args.bin
    if not path:
        files = glob.glob('/home/vio/jtzero_runs/*_REMOTE_LOG_XKV_DIAG/remote_ekf.bin')
        if not files:
            print('ОШИБКА: remote_ekf.bin не найден', file=sys.stderr)
            return 2
        path = max(files, key=os.path.getmtime)

    with open(path, 'rb') as f:
        data = f.read()

    fmts = read_formats(data)
    wanted = {'XKV1', 'XKV2', 'XKT', 'XKF1'}
    rec = parse_records(data, fmts, wanted)

    print('======================================================================')
    print('JT-ZERO — XKV GYRO-BIAS GATE ANALYSIS')
    print('======================================================================')
    print(f'BIN: {path}')
    print(f'bytes={len(data)} formats={len(fmts)}')
    for n in sorted(wanted):
        print(f'{n}: {len(rec[n])} records')

    if not rec['XKV1'] or not rec['XKV2']:
        print('\nОШИБКА: XKV1/XKV2 не найдены. Проверь EK3_LOG_LEVEL=0 и целостность remote log.')
        return 3
    if not rec['XKT']:
        print('\nОШИБКА: XKT не найден — нельзя вычислить точный диапазон порога dtEkfAvg.')
        return 4

    for n in wanted:
        rec[n].sort(key=key_time)

    # XKT даёт минимум/максимум low-pass dtEkfAvg за окно логирования.
    dtmins = [float(r.get('EKFMin', 0.0)) for r in rec['XKT'] if float(r.get('EKFMin', 0.0)) > 0]
    dtmaxs = [float(r.get('EKFMax', 0.0)) for r in rec['XKT'] if float(r.get('EKFMax', 0.0)) > 0]
    if not dtmins or not dtmaxs:
        print('\nОШИБКА: в XKT нет валидных EKFMin/EKFMax.')
        return 5

    dt_min = min(dtmins)
    dt_max = max(dtmaxs)
    dt_mid = 0.5*(pct(dtmins, 0.5) + pct(dtmaxs, 0.5))
    rate_limit_deg_s = 0.15
    thr_min = math.radians(rate_limit_deg_s * dt_min) ** 2
    thr_max = math.radians(rate_limit_deg_s * dt_max) ** 2
    thr_mid = math.radians(rate_limit_deg_s * dt_mid) ** 2

    print('\n===== EKF TIMING / GATE =====')
    print(f'dtEkfAvg range from XKT: {dt_min:.9f} .. {dt_max:.9f} s; representative={dt_mid:.9f} s')
    print(f'delAngBiasVarMax range: {thr_min:.12e} .. {thr_max:.12e}; representative={thr_mid:.12e}')
    print(f'equivalent gyro-bias sigma limit: {rate_limit_deg_s:.3f} deg/s')

    xkv2_exact = {(key_time(r), int(r.get('C', 0))): r for r in rec['XKV2']}
    xkf_by_core: Dict[int, List[Dict[str, Any]]] = {}
    for r in rec['XKF1']:
        xkf_by_core.setdefault(int(r.get('C', 0)), []).append(r)

    p10s: List[float] = []; p11s: List[float] = []; p12s: List[float] = []
    txs: List[float] = []; tys: List[float] = []
    definite_fail = 0; definite_pass = 0; ambiguous = 0; paired = 0

    for a in rec['XKV1']:
        t = key_time(a); core = int(a.get('C', 0))
        b = xkv2_exact.get((t, core)) or nearest([r for r in rec['XKV2'] if int(r.get('C', 0)) == core], t, 20_000)
        if not b:
            continue
        try:
            p10 = float(a['V10']); p11 = float(a['V11']); p12 = float(b['V12'])
        except (KeyError, ValueError, TypeError):
            continue
        p10s.append(p10); p11s.append(p11); p12s.append(p12)
        attitude = nearest(xkf_by_core.get(core, []), t, 50_000)
        if attitude is None:
            continue
        try:
            rnb = nav_to_body_from_rpy_deg(float(attitude['Roll']), float(attitude['Pitch']), float(attitude['Yaw']))
        except (KeyError, ValueError, TypeError):
            continue
        tx, ty, _ = mat_vec(rnb, (p10, p11, p12))
        tx = abs(tx); ty = abs(ty)
        txs.append(tx); tys.append(ty); paired += 1
        # Реальный threshold в конкретный момент находится внутри [thr_min,thr_max].
        # >thr_max => гарантированный fail; <thr_min по обеим => гарантированный pass.
        if tx > thr_max or ty > thr_max:
            definite_fail += 1
        elif tx < thr_min and ty < thr_min:
            definite_pass += 1
        else:
            ambiguous += 1

    def stats_line(name: str, v: List[float]) -> None:
        print(f'{name}: n={len(v)} min={min(v):.12e} p50={pct(v,0.5):.12e} p95={pct(v,0.95):.12e} max={max(v):.12e}')

    print('\n===== RAW GYRO-BIAS COVARIANCE =====')
    stats_line('P10', p10s)
    stats_line('P11', p11s)
    stats_line('P12', p12s)
    print(f'representative sigma-rate: P10={math.degrees(math.sqrt(max(pct(p10s,0.5),0.0))/dt_mid):.6f} deg/s '
          f'P11={math.degrees(math.sqrt(max(pct(p11s,0.5),0.0))/dt_mid):.6f} deg/s '
          f'P12={math.degrees(math.sqrt(max(pct(p12s,0.5),0.0))/dt_mid):.6f} deg/s')

    print('\n===== YAW=0 BRANCH — REPRODUCED HORIZONTAL TEST =====')
    if paired:
        stats_line('|temp.x|', txs)
        stats_line('|temp.y|', tys)
        print(f'paired={paired} definite_fail={definite_fail} definite_pass={definite_pass} ambiguous={ambiguous}')
    else:
        print('Нет пар XKV + XKF1 для восстановления prevTnb; доступен только raw covariance.')

    print('\n===== VERDICT =====')
    if paired and definite_fail == paired:
        print('DEL_ANG_BIAS_GATE=PROVEN_FAIL')
        print('Все восстановленные samples гарантированно превышают даже максимальный допустимый threshold.')
        print('Следовательно delAngBiasLearned=false действительно блокирует readyToUseOptFlow().')
    elif paired and definite_pass == paired:
        print('DEL_ANG_BIAS_GATE=PROVEN_PASS')
        print('Все восстановленные samples ниже даже минимального threshold.')
        print('Следовательно gyro-bias gate не объясняет AID_NONE; нужно пересмотреть предыдущую локализацию readyToUseOptFlow().')
    elif paired:
        print('DEL_ANG_BIAS_GATE=MIXED_OR_AMBIGUOUS')
        print('По этому логу есть смесь/пограничные samples; смотри числа выше, параметрические догадки делать не нужно.')
    else:
        raw_fail = sum(1 for x,y in zip(p10s,p11s) if x > thr_max or y > thr_max)
        print(f'RAW_BODY_AXIS_FAIL_SAMPLES={raw_fail}/{min(len(p10s),len(p11s))}')
        print('Точный YAW=0 gate без attitude-пар не воспроизведён.')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
