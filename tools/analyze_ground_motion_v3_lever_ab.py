#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path


def q(v, p):
    if not v:
        return float('nan')
    a = sorted(v)
    x = (len(a)-1)*p
    i = int(math.floor(x)); j = int(math.ceil(x))
    if i == j:
        return a[i]
    return a[i]*(j-x)+a[j]*(x-i)


def main():
    if len(sys.argv) != 2:
        print(f"Использование: {sys.argv[0]} <GROUND_MOTION_V3_LEVER_AB.csv>", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    rows = []
    with path.open(newline='') as f:
        for r in csv.DictReader(f):
            try:
                r['_state'] = int(r['state'])
                for k in r:
                    if k not in ('state', '_state'):
                        try: r[k] = float(r[k])
                        except Exception: pass
                rows.append(r)
            except Exception:
                pass
    mov = [r for r in rows if r.get('_state') == 1]
    done = [r for r in rows if r.get('_state') == 2]
    if not mov:
        print('Нет строк state==1: физический прогон не был записан или не стартовал.')
        return 1

    last = done[-1] if done else mov[-1]
    cn = math.hypot(last['current_x_m'], last['current_y_m'])*1000
    ln = math.hypot(last['lever_x_m'], last['lever_y_m'])*1000
    cp = last['current_path_m']*1000
    lp = last['lever_path_m']*1000
    pitches = [r['pitch']*180/math.pi for r in mov]
    rolls = [r['roll']*180/math.pi for r in mov]
    hl = [r['h_luna_m']*1000 for r in mov]
    hc = [r['h_camera_m']*1000 for r in mov]
    dh = [r['lever_dh_m']*1000 for r in mov]
    cinl = [r['current_inliers'] for r in mov]
    linl = [r['lever_inliers'] for r in mov]
    csc = [r['current_scatter_m']*1000 for r in mov]
    lsc = [r['lever_scatter_m']*1000 for r in mov]
    camage = [r['camera_age_ms'] for r in mov]
    syncnear = [r['att_sync_nearest_ms'] for r in mov]
    lunaage = [r['luna_age_ms'] for r in mov]

    print('================ JT-ZERO V3 LEVER A/B ================')
    print(f'CSV: {path}')
    print(f'measurement rows        = {len(mov)}')
    print(f'pitch median/span       = {statistics.median(pitches):+.3f} deg / {min(pitches):+.3f} .. {max(pitches):+.3f}')
    print(f'roll  median/span       = {statistics.median(rolls):+.3f} deg / {min(rolls):+.3f} .. {max(rolls):+.3f}')
    print(f'H Luna median           = {statistics.median(hl):.3f} mm')
    print(f'lever dH median         = {statistics.median(dh):+.3f} mm')
    print(f'lever dH p05/p95        = {q(dh,0.05):+.3f} / {q(dh,0.95):+.3f} mm')
    print(f'H camera median         = {statistics.median(hc):.3f} mm')
    print('-------------------------------------------------------')
    print(f'CURRENT endpoint NET    = {cn:.3f} mm')
    print(f'LEVER   endpoint NET    = {ln:.3f} mm')
    print(f'LEVER - CURRENT NET     = {ln-cn:+.3f} mm')
    print(f'CURRENT endpoint PATH   = {cp:.3f} mm')
    print(f'LEVER   endpoint PATH   = {lp:.3f} mm')
    print(f'LEVER - CURRENT PATH    = {lp-cp:+.3f} mm')
    print(f'NET gain                = {(ln/cn-1)*100:+.3f} %' if cn else 'NET gain                = n/a')
    print('-------------------------------------------------------')
    print(f'inliers median C/L      = {statistics.median(cinl):.1f} / {statistics.median(linl):.1f}')
    print(f'scatter median C/L      = {statistics.median(csc):.3f} / {statistics.median(lsc):.3f} mm')
    print(f'camera age median       = {statistics.median(camage):.3f} ms')
    print(f'ATT sync nearest median = {statistics.median(syncnear):.3f} ms')
    print(f'Luna age median         = {statistics.median(lunaage):.3f} ms')
    print('=======================================================')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
