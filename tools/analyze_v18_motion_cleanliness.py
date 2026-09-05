#!/usr/bin/env python3
import csv, math, statistics

H="/home/vio"
DEEP=H+"/jtzero_v18_leg_diagnostics_deep.csv"
EXC=H+"/jtzero_v18_imu_excitation.csv"
OUT=H+"/jtzero_v18_motion_cleanliness.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def fv(r,k):return float(r[k])
def iv(r,k):return int(float(r[k]))

deep={iv(r,"leg"):r for r in read(DEEP)}
exc={iv(r,"leg"):r for r in read(EXC)}

def corr(xs,ys):
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    n=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx=math.sqrt(sum((x-mx)**2 for x in xs))
    dy=math.sqrt(sum((y-my)**2 for y in ys))
    return n/(dx*dy) if dx and dy else float("nan")

rows=[]
print("================ V18 MOTION CLEANLINESS ================")
for leg in sorted(deep):
    d=deep[leg]; e=exc[leg]
    scale=fv(d,"scale")
    err=abs(scale-1.0)
    az=fv(e,"accel_std_z")
    gy=fv(e,"gyro_std_y")
    gz=fv(e,"gyro_std_z")
    rng=fv(d,"range_std_mm")
    step=fv(e,"accel_step_rms")
    # Dimensionless-ish diagnostic score normalized to the clean LEG5 values.
    cleanliness_penalty = (
        az/0.059258 +
        gy/0.003734 +
        gz/0.003612 +
        rng/2.35 +
        step/0.142174
    )/5.0
    row={
      "leg":leg,
      "direction":d["direction"],
      "scale":scale,
      "abs_scale_error":err,
      "accel_std_z":az,
      "gyro_std_y":gy,
      "gyro_std_z":gz,
      "range_std_mm":rng,
      "accel_step_rms":step,
      "motion_contamination_score":cleanliness_penalty,
    }
    rows.append(row)
    tag=" GOOD-REF" if leg==5 else (" BAD" if leg in (3,6) else "")
    print(f'LEG {leg} {d["direction"]}: scale={scale:.3f} absErr={err:.3f}{tag}')
    print(f'  azStd={az:.4f} gyroY/Z={gy:.5f}/{gz:.5f} rangeStd={rng:.2f}mm stepRMS={step:.4f}')
    print(f'  contaminationScore={cleanliness_penalty:.3f}')

print("\n================ CORRELATIONS ================")
ys=[r["abs_scale_error"] for r in rows]
for k in ["accel_std_z","gyro_std_y","gyro_std_z","range_std_mm","accel_step_rms","motion_contamination_score"]:
    print(f'Pearson({k}, absScaleError) = {corr([r[k] for r in rows],ys):+.4f}')

print("\n================ LEG5 vs LEG6 ================")
a=next(r for r in rows if r["leg"]==5)
b=next(r for r in rows if r["leg"]==6)
for k in ["scale","abs_scale_error","accel_std_z","gyro_std_y","gyro_std_z","range_std_mm","accel_step_rms","motion_contamination_score"]:
    print(f'{k}: LEG5={a[k]:.6f} LEG6={b[k]:.6f} delta={b[k]-a[k]:+.6f}')

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
print("Saved:",OUT)
