#!/usr/bin/env python3
import csv, math, statistics
from collections import Counter

H="/home/vio"
LEGS=H+"/jtzero_500mm_v15_legs.csv"
BACKEND=H+"/jtzero_500mm_v15_backend.csv"
FRONTEND=H+"/jtzero_500mm_v15_frontend.csv"
OUT=H+"/jtzero_v15_keyframe_frontend_diag.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k):return int(float(r[k]))
def fv(r,k):return float(r[k])

legs=read(LEGS)
be=sorted(read(BACKEND),key=lambda r:iv(r,"keyframe"))
fe=sorted(read(FRONTEND),key=lambda r:iv(r,"timestamp_ns"))
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

print("================ V15 KEYFRAME FRONTEND BY LEG ================")
rows=[]
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    te=kf_ts[iv(L,"end_settled_kf")]
    ep=kf_ts[iv(L,"end_press_kf")]
    kr=[r for r in fe if iv(r,"is_keyframe")!=0 and t0<=iv(r,"timestamp_ns")<=te]
    move=[r for r in kr if iv(r,"timestamp_ns")<=ep]
    settle=[r for r in kr if iv(r,"timestamp_ns")>ep]
    def stats(rr):
        if not rr:return "none"
        ratios=[fv(r,"mono_inlier_ratio") for r in rr]
        inl=[iv(r,"mono_inliers") for r in rr]
        puts=[iv(r,"mono_putatives") for r in rr]
        st=Counter(r["mono_status"] for r in rr)
        low=sum(1 for x in ratios if x<0.10)
        return (f'n={len(rr)} ratioMean={statistics.mean(ratios):.3f} '
                f'ratioMin={min(ratios):.3f} low<0.10={low}/{len(rr)} '
                f'inliersMean={statistics.mean(inl):.1f} putMean={statistics.mean(puts):.1f} status={dict(st)}')
    print(f'LEG {leg} {L["direction"]}:')
    print("  MOVE  ",stats(move))
    print("  SETTLE",stats(settle))
    for phase,rr in (("MOVE",move),("SETTLE",settle)):
        for r in rr:
            rows.append({"leg":leg,"direction":L["direction"],"phase":phase,**r})

with open(OUT,"w",newline="") as f:
    fields=["leg","direction","phase"]+list(fe[0].keys())
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)

print("\n================ LEG2 FRONTEND KEYFRAMES AROUND END ================")
L2=next(L for L in legs if iv(L,"leg")==2)
ep=kf_ts[iv(L2,"end_press_kf")]
for r in fe:
    if iv(r,"is_keyframe")==0: continue
    dt=(iv(r,"timestamp_ns")-ep)*1e-9
    if -3.0<=dt<=5.0:
        print(f'dt={dt:+.3f}s frame={iv(r,"frame_id")} tracked={iv(r,"tracked_features")} '
              f'inl={iv(r,"mono_inliers")}/{iv(r,"mono_putatives")} '
              f'ratio={fv(r,"mono_inlier_ratio"):.3f} status={r["mono_status"]}')

print("\n================ BACKEND KF145..165 RAW ================")
prev=None
for r in be:
    k=iv(r,"keyframe")
    if 145<=k<=165:
        ts=iv(r,"timestamp_ns")
        dp=0.0; dtms=float("nan")
        if prev is not None:
            dx=fv(r,"px_m")-fv(prev,"px_m");dy=fv(r,"py_m")-fv(prev,"py_m");dz=fv(r,"pz_m")-fv(prev,"pz_m")
            dp=1000*math.sqrt(dx*dx+dy*dy+dz*dz)
            dtms=(ts-iv(prev,"timestamp_ns"))/1e6
        print(f'KF={k} ts={ts} dt_prev={dtms:.3f}ms '
              f'P=[{fv(r,"px_m"):.4f},{fv(r,"py_m"):.4f},{fv(r,"pz_m"):.4f}] '
              f'V=[{fv(r,"vx_m_s"):.4f},{fv(r,"vy_m_s"):.4f},{fv(r,"vz_m_s"):.4f}] dPprev={dp:.1f}mm')
        prev=r

print("\n================ FRONTEND TIMESTAMP RANGE NEAR KF153..160 ================")
ts153=kf_ts.get(153)
ts160=kf_ts.get(160)
print("backend_ts_KF153=",ts153)
print("backend_ts_KF160=",ts160)
if ts153 is not None and ts160 is not None:
    print("delta_153_160_ms=",(ts160-ts153)/1e6)
    around=[r for r in fe if ts153-500_000_000<=iv(r,"timestamp_ns")<=ts160+500_000_000]
    print("frontend_rows_in_window=",len(around),"keyframes=",sum(iv(r,"is_keyframe")!=0 for r in around))
    for r in around:
        if iv(r,"is_keyframe")!=0:
            print(f'front ts={iv(r,"timestamp_ns")} dt_from_KF153={(iv(r,"timestamp_ns")-ts153)/1e9:+.3f}s '
                  f'frame={iv(r,"frame_id")} inl={iv(r,"mono_inliers")}/{iv(r,"mono_putatives")} '
                  f'ratio={fv(r,"mono_inlier_ratio"):.3f} status={r["mono_status"]}')
print("Saved:",OUT)
