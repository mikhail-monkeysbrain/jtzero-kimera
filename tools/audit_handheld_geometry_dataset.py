#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, math, statistics
from pathlib import Path

def f(r,k,d=float("nan")):
    try: return float(r[k])
    except Exception: return d

def span(v): return max(v)-min(v) if v else float("nan")
def pct(v,p):
    if not v: return float("nan")
    a=sorted(v); x=(len(a)-1)*p; i=int(x); j=min(i+1,len(a)-1); u=x-i
    return a[i]*(1-u)+a[j]*u

def unwrap(vals):
    if not vals:return []
    out=[vals[0]]
    for x in vals[1:]:
        y=x
        while y-out[-1]>math.pi:y-=2*math.pi
        while y-out[-1]<-math.pi:y+=2*math.pi
        out.append(y)
    return out

def clusters_1d(vals, gap=0.08):
    # Simple range-mode detector. A table->floor jump should appear as a distinct
    # cluster separated by many cm; no assumption which cluster is "correct".
    if not vals:return []
    a=sorted(vals)
    chunks=[[a[0]]]
    for x in a[1:]:
        if x-chunks[-1][-1] > gap:
            chunks.append([x])
        else:
            chunks[-1].append(x)
    chunks.sort(key=len, reverse=True)
    return chunks

def aruco_probe(rows, bin_path, sample_n=100):
    try:
        import cv2, numpy as np
    except Exception as e:
        return {"ok":False,"reason":f"OpenCV import: {e}"}
    if not hasattr(cv2,"aruco"):
        return {"ok":False,"reason":"cv2.aruco отсутствует"}

    names=[
      "DICT_4X4_50","DICT_4X4_100","DICT_4X4_250","DICT_4X4_1000",
      "DICT_5X5_50","DICT_5X5_100","DICT_5X5_250","DICT_5X5_1000",
      "DICT_6X6_50","DICT_6X6_100","DICT_6X6_250","DICT_6X6_1000",
      "DICT_7X7_50","DICT_7X7_100","DICT_7X7_250","DICT_7X7_1000",
      "DICT_ARUCO_ORIGINAL"
    ]
    ds=[]
    for n in names:
        if hasattr(cv2.aruco,n):
            ds.append((n,cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,n))))
    step=max(1,len(rows)//sample_n)
    picks=rows[::step][:sample_n]
    scores={n:[0,0,set()] for n,_ in ds}
    decoded=0
    with bin_path.open("rb") as fh:
        for r in picks:
            off=int(float(r["jpeg_offset"])); sz=int(float(r["jpeg_size"]))
            fh.seek(off); data=fh.read(sz)
            img=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            if img is None: continue
            decoded+=1
            for n,d in ds:
                try:
                    corners,ids,_=cv2.aruco.detectMarkers(img,d)
                except Exception:
                    det=cv2.aruco.ArucoDetector(d)
                    corners,ids,_=det.detectMarkers(img)
                if ids is not None and len(ids):
                    scores[n][0]+=1; scores[n][1]+=len(ids)
                    scores[n][2].update(int(x) for x in ids.flatten())
    ranked=[]
    for n,(fr,mk,ids) in scores.items():
        ranked.append((fr,mk,len(ids),n,sorted(ids)))
    ranked.sort(reverse=True)
    return {"ok":True,"decoded":decoded,"ranked":ranked[:5]}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--samples",type=int,default=100)
    args=ap.parse_args()
    cp=args.run_dir/"frames.csv"; bp=args.run_dir/"frames.mjpgbin"
    if not cp.exists() or not bp.exists():
        raise SystemExit("нет frames.csv или frames.mjpgbin")

    with cp.open(newline="") as fh: rows=list(csv.DictReader(fh))
    cam=[f(r,"camera_ts_ns") for r in rows]
    roll=[f(r,"roll") for r in rows]; pitch=[f(r,"pitch") for r in rows]
    yaw=unwrap([f(r,"yaw") for r in rows])
    gx=[f(r,"xgyro") for r in rows]; gy=[f(r,"ygyro") for r in rows]; gz=[f(r,"zgyro") for r in rows]
    gnorm=[math.sqrt(x*x+y*y+z*z) for x,y,z in zip(gx,gy,gz)]
    luna=[f(r,"luna_m") for r in rows if f(r,"luna_m")>0]
    lage=[f(r,"luna_age_ms") for r in rows if math.isfinite(f(r,"luna_age_ms"))]
    aage=[f(r,"att_age_ms") for r in rows if math.isfinite(f(r,"att_age_ms"))]
    iage=[f(r,"imu_age_ms") for r in rows if math.isfinite(f(r,"imu_age_ms"))]

    dt=[(cam[i]-cam[i-1])*1e-9 for i in range(1,len(cam)) if cam[i]>cam[i-1]]
    duration=(cam[-1]-cam[0])*1e-9
    modes=clusters_1d(luna,0.08)

    print("===== HANDHELD DATASET AUDIT =====")
    print(f"run={args.run_dir}")
    print(f"rows={len(rows)} duration={duration:.2f}s")
    print(f"saved-frame dt median/p95={statistics.median(dt):.4f}/{pct(dt,0.95):.4f}s")
    print(f"ATT age p50/p95={statistics.median(aage):.2f}/{pct(aage,0.95):.2f} ms")
    print(f"IMU age p50/p95={statistics.median(iage):.2f}/{pct(iage,0.95):.2f} ms")
    print(f"Luna age p50/p95={statistics.median(lage):.2f}/{pct(lage,0.95):.2f} ms")
    print()
    print("ATTITUDE excitation:")
    print(f"  roll span  = {math.degrees(span(roll)):.1f} deg")
    print(f"  pitch span = {math.degrees(span(pitch)):.1f} deg")
    print(f"  yaw span   = {math.degrees(span(yaw)):.1f} deg")
    print(f"  gyro norm p50/p95/max = {statistics.median(gnorm):.3f}/{pct(gnorm,0.95):.3f}/{max(gnorm):.3f} rad/s")
    print()
    print("TF-Luna:")
    print(f"  raw min/p50/p95/max = {min(luna):.3f}/{statistics.median(luna):.3f}/{pct(luna,0.95):.3f}/{max(luna):.3f} m")
    if len(modes)>1:
        print("  RANGE MULTIMODALITY DETECTED — не предполагаем одну плоскость.")
        for i,m in enumerate(modes[:4],1):
            print(f"    mode{i}: n={len(m)} median={statistics.median(m):.3f} m span={span(m):.3f} m")
        print("  Для геометрии Luna анализатор должен классифицировать table/floor отдельно")
        print("  и исключать переходы через край стола, а не усреднять их.")
    else:
        print("  один основной range-mode")
    print()

    ar=aruco_probe(rows,bp,args.samples)
    print("ArUco/ChArUco visibility probe:")
    if not ar["ok"]:
        print(f"  unavailable: {ar['reason']}")
    else:
        print(f"  decoded sample frames={ar['decoded']}")
        for fr,mk,uid,name,ids in ar["ranked"]:
            print(f"  {name}: detected_frames={fr}, markers={mk}, unique_ids={uid}, ids={ids[:20]}")
        if ar["ranked"] and ar["ranked"][0][0] > 0:
            print("  candidate dictionary found; exact ChArUco board geometry will be taken from the previous calibration setup.")
        else:
            print("  no ArUco markers detected in sampled frames")

    print()
    print("===== VERDICT =====")
    enough_att=math.degrees(span(roll))>=15 and math.degrees(span(pitch))>=15 and math.degrees(span(yaw))>=30
    enough_gyro=pct(gnorm,0.95)>=0.15
    print(f"attitude excitation : {'PASS' if enough_att else 'WEAK'}")
    print(f"angular excitation  : {'PASS' if enough_gyro else 'WEAK'}")
    print("range edge handling : REQUIRED (table/floor mixture is allowed)")
    print("No new physical run is requested by this audit.")
if __name__=="__main__":
    main()
