#!/usr/bin/env python3
# Проверка конкретного EKF3 ground-gate для OpticalFlow на основе условий стенда.
# Ничего не пишет в FC.
import argparse

ap=argparse.ArgumentParser()
ap.add_argument("--range", type=float, required=True)
ap.add_argument("--armed", choices=["yes","no"], required=True)
ap.add_argument("--takeoff-detected", choices=["yes","no"], default="no")
args=ap.parse_args()

rng=args.range
takeoff=args.takeoff_detected=="yes"
print("===== EKF3 OPTFLOW GROUND GATE CHECK =====")
print(f"range={rng:.3f} m")
print(f"armed={args.armed}")
print(f"takeOffDetected={takeoff}")
gate=(not takeoff) and (rng < 0.5)
print(f"ZERO_FLOW_GATE={'ACTIVE' if gate else 'OPEN'}")
if gate:
    print("INTERPRETATION: при HAGL < 0.5 м и takeOffDetected=false EKF3 обнуляет flowRadXY/flowRadXYcomp перед fusion.")
else:
    print("INTERPRETATION: это конкретное условие обнуления flow не выполняется.")
