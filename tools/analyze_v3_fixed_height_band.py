#!/usr/bin/env python3
import csv, math, statistics, sys

if len(sys.argv) != 2:
    print(f"Использование: {sys.argv[0]} <GROUND_MOTION_V3_H20_H21_AB.csv>")
    raise SystemExit(2)

fn=sys.argv[1]
with open(fn,newline='') as f:
    rows=[r for r in csv.DictReader(f) if int(r['state'])==1]
if not rows:
    raise SystemExit('Нет строк state=1')

# This diagnostic uses the experimentally verified same-frame linear metric scaling:
# NET ratio matched camera-height ratio to ~1e-5 in H20/H21 A/B.
# Use H20 branch as the reference because its median camera height is recorded directly.
h20=[float(r['h20_m']) for r in rows]
h21=[float(r['h21_m']) for r in rows]
ref_h=statistics.median(h20)
last=rows[-1]
ref_net=math.hypot(float(last['h20_x_m']),float(last['h20_y_m']))
net21=math.hypot(float(last['h21_x_m']),float(last['h21_y_m']))
ratio_h=statistics.median([float(r['h21_m'])/float(r['h20_m']) for r in rows])
ratio_net=net21/ref_net

print('================ FIXED CAMERA HEIGHT BAND ================')
print('CSV:',fn)
print('rows:',len(rows))
print(f'reference H20 median : {ref_h*1000:.3f} mm')
print(f'reference H20 NET    : {ref_net*1000:.3f} mm')
print(f'check H21/H20 H      : {ratio_h:.6f}')
print(f'check H21/H20 NET    : {ratio_net:.6f}')
print()
for hmm in (195.0,200.0,205.0):
    net=ref_net*(hmm/1000.0)/ref_h
    err=(net-0.500)*1000.0
    print(f'H={hmm:6.1f} mm -> NET={net*1000:8.3f} mm   error500={err:+8.3f} mm ({err/5.0:+6.2f}%)')

h_for_500=ref_h*(0.500/ref_net)
print()
print(f'H required for NET=500 mm: {h_for_500*1000:.3f} mm')
print('Physical independent band : 195.000 .. 205.000 mm')
if 0.195 <= h_for_500 <= 0.205:
    print('RESULT: 500 mm IS compatible with the independent height band')
else:
    print('RESULT: 500 mm IS NOT compatible with the independent height band')
