#!/usr/bin/env python3
# JT-Zero — прямое доказательство доставки OpticalFlow до NavEKF3::writeOptFlowMeas.
#
# Использует remote DataFlash BIN и live PARAM_REQUEST_READ:
#   - ROFH: AP_DAL replay-запись writeOptFlowMeas;
#   - RFRH/RFRF: признаки того, что replay-поток вообще успел стартовать;
#   - EK2_ENABLE: если параметр отсутствует, EKF2 в этой прошивке недоступен.
#
# Важно: ROFH пишется только при LOG_REPLAY=1.
# Параметры FC не изменяет.

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from collections import Counter

from pymavlink import mavutil

from analyze_remote_xkv_gate import read_formats, parse_records, key_time


def last_remote_bin() -> str | None:
    files = glob.glob('/home/vio/jtzero_runs/*_REMOTE_LOG_XKV_DIAG/remote_ekf.bin')
    return max(files, key=os.path.getmtime) if files else None


def wait_fc(master, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if m is None:
            continue
        if int(getattr(m, 'autopilot', -1)) == int(mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA):
            return int(m.get_srcSystem()), int(m.get_srcComponent())
    return None


def request_param(master, sysid: int, compid: int, name: str, timeout: float = 2.0):
    master.mav.param_request_read_send(sysid, compid, name.encode('ascii'), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
        if m is None:
            continue
        if int(m.get_srcSystem()) != sysid:
            continue
        pid = m.param_id
        if isinstance(pid, bytes):
            pid = pid.decode('ascii', errors='ignore')
        pid = str(pid).rstrip('\x00')
        if pid == name:
            return float(m.param_value)
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bin', nargs='?', help='remote_ekf.bin; default: latest')
    ap.add_argument('--device', default='/dev/ttyAMA0')
    ap.add_argument('--baud', type=int, default=460800)
    args = ap.parse_args()

    path = args.bin or last_remote_bin()
    if not path:
        print('ОШИБКА: remote_ekf.bin не найден', file=sys.stderr)
        return 2

    data = open(path, 'rb').read()
    fmts = read_formats(data)
    wanted = {'ROFH', 'RFRH', 'RFRF', 'OF', 'XKFS', 'XKF4'}
    rec = parse_records(data, fmts, wanted)
    for name in wanted:
        rec[name].sort(key=key_time)

    print('=' * 70)
    print('JT-ZERO — EKF3 OPTICAL FLOW INGRESS PROOF')
    print('=' * 70)
    print(f'BIN: {path}')
    print(f'bytes={len(data)} formats={len(fmts)}')
    for name in sorted(wanted):
        print(f'{name}: {len(rec[name])} records')

    print('\n===== REPLAY STREAM HEALTH =====')
    replay_started = bool(rec['RFRH']) and bool(rec['RFRF'])
    print(f'REPLAY_FRAMES_PRESENT={"YES" if replay_started else "NO"}')
    print(f'RFRH={len(rec["RFRH"])} RFRF={len(rec["RFRF"])}')
    if not replay_started:
        print('NOTE: replay frame stream не виден; при коротком MAVLink remote log startup headers могли не успеть завершиться.')

    print('\n===== ROFH =====')
    if rec['ROFH']:
        t = [key_time(r) for r in rec['ROFH'] if key_time(r) > 0]
        q = [int(r.get('Qual', r.get('Q', 0))) for r in rec['ROFH']]
        print(f'ROFH_PRESENT=YES count={len(rec["ROFH"])}')
        if t:
            d = [(b-a)/1000.0 for a,b in zip(t,t[1:]) if b > a]
            if d:
                print(f'ROFH log dt ms min={min(d):.3f} max={max(d):.3f}')
        if q:
            print(f'ROFH quality values={dict(sorted(Counter(q).items()))}')
    else:
        print('ROFH_PRESENT=NO')

    print('\n===== LIVE FIRMWARE PARAM PROBE =====')
    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=197,
        autoreconnect=False,
    )
    fc = wait_fc(master)
    if fc is None:
        print('ОШИБКА: HEARTBEAT ArduPilot не получен')
        return 3
    sysid, compid = fc
    print(f'FC heartbeat sys={sysid} comp={compid}')

    ek2 = request_param(master, sysid, compid, 'EK2_ENABLE')
    ek3_flow = request_param(master, sysid, compid, 'EK3_FLOW_USE')
    ek3_velxy = request_param(master, sysid, compid, 'EK3_SRC1_VELXY')
    ahrs = request_param(master, sysid, compid, 'AHRS_EKF_TYPE')
    log_replay = request_param(master, sysid, compid, 'LOG_REPLAY')

    print(f'EK2_ENABLE={"ABSENT" if ek2 is None else ek2}')
    print(f'EK3_FLOW_USE={ek3_flow}')
    print(f'EK3_SRC1_VELXY={ek3_velxy}')
    print(f'AHRS_EKF_TYPE={ahrs}')
    print(f'LOG_REPLAY={log_replay}')

    print('\n===== VERDICT =====')
    rofh = bool(rec['ROFH'])
    ek2_absent = ek2 is None
    if rofh and ek2_absent:
        print('EKF3_WRITE_OPTFLOW_PATH=PROVEN')
        print('ROFH присутствует, а EKF2 parameter surface отсутствует.')
        print('Для этой конфигурации replay-запись ROFH прошла через NavEKF3::writeOptFlowMeas().')
        print('NavEKF3::writeOptFlowMeas() затем вызывает core[i].writeOptFlowMeas() для каждого EKF3 core.')
        return 0

    if rofh:
        print('EKF3_WRITE_OPTFLOW_PATH=AMBIGUOUS')
        print('ROFH есть, но EKF2 также может быть доступен, поэтому ROFH сам по себе не доказывает EKF3 path.')
        return 4

    if not replay_started:
        print('EKF3_WRITE_OPTFLOW_PATH=INCONCLUSIVE_STARTUP')
        print('ROFH нет, но и replay frame stream RFRH/RFRF не успел появиться.')
        print('Этот BIN слишком ранний для отрицательного вывода о EKF3 ingress.')
        return 6

    print('EKF3_WRITE_OPTFLOW_PATH=NOT_PROVEN')
    print('Replay stream активен, но ROFH отсутствует.')
    return 5


if __name__ == '__main__':
    raise SystemExit(main())
