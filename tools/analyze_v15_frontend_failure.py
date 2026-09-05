#!/usr/bin/env python3
import csv, math, statistics
from collections import Counter

H="/home/vio"
LEGS=H+"/jtzero_500mm_v15_legs.csv"
BACKEND=H+"/jtzero_500mm_v15_backend.csv"
FRONTEND=H+"/jtzero_500mm_v15_frontend.csv"
OUT=H+"/jtzero_v15_frontend_leg_diagnostics.csv"

def read(p):
    with open(p,newline="") as f:return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read(LEGS); be=read(BACKEND); fe=read(FRONTEND)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}
fe=sorted(fe,key=lambda r:iv(r,"timestamp_ns"))

def pct(xs,p):
    if not xs:return float("nan")
    s=sorted(xs); q=(len(s)-1)*p/100; a=int(math.floor(q)); b=int(math.ceil(q))
    if a==b:return s[a]
    return s[a]*(b-q)+s[b]*(q-a)

def summarize(rows):
    if not rows:return {}
    tracked=[fv(r,"tracked_features") for r in rows]
    inl=[fv(r,"mono_inliers") for r in rows]
    put=[fv(r,"mono_putatives") for r in rows]
    ratio=[fv(r,"mono_inlier_ratio") for r in rows if fv(r,"mono_putatives")>0]
    iters=[fv(r,"mono_ransac_iters") for r in rows]
    ft=[fv(r,"feature_tracking_time_s")*1000 for r in rows]
    st=Counter(r["mono_status"] for r in rows)
    return {
      "n":len(rows),
      "tracked_mean":statistics.mean(tracked),
      "tracked_p10":pct(tracked,10),
      "inliers_mean":statistics.mean(inl),
      "inliers_p10":pct(inl,10),
      "putatives_mean":statistics.mean(put),
      "ratio_mean":statistics.mean(ratio) if ratio else float("nan"),
      "ratio_p10":pct(ratio,10),
      "iters_mean":statistics.mean(iters),
      "track_ms_mean":statistics.mean(ft),
      "status":dict(st),
    }

detail=[]
print("================ V15 FRONTEND BY LEG ================")
for L in legs:
    leg=iv(L,"leg")
    s0=kf_ts[iv(L,"start_settled_kf")]
    ep=kf_ts[iv(L,"end_press_kf")]
    es=kf_ts[iv(L,"end_settled_kf")]

    move=[r for r in fe if s0<=iv(r,"timestamp_ns")<=ep]
    settle=[r for r in fe if ep<iv(r,"timestamp_ns")<=es]
    sm=summarize(move); ss=summarize(settle)
    print(f'LEG {leg} {L["direction"]}:')
    if sm:
      print(f'  MOVE   n={sm["n"]} tracked={sm["tracked_mean"]:.1f} p10={sm["tracked_p10"]:.1f} '
            f'inliers={sm["inliers_mean"]:.1f} p10={sm["inliers_p10"]:.1f} '
            f'ratio={sm["ratio_mean"]:.3f} p10={sm["ratio_p10"]:.3f} '
            f'ransac={sm["iters_mean"]:.1f} track={sm["track_ms_mean"]:.2f}ms status={sm["status"]}')
    if ss:
      print(f'  SETTLE n={ss["n"]} tracked={ss["tracked_mean"]:.1f} p10={ss["tracked_p10"]:.1f} '
            f'inliers={ss["inliers_mean"]:.1f} p10={ss["inliers_p10"]:.1f} '
            f'ratio={ss["ratio_mean"]:.3f} p10={ss["ratio_p10"]:.3f} '
            f'ransac={ss["iters_mean"]:.1f} track={ss["track_ms_mean"]:.2f}ms status={ss["status"]}')
    for phase,rr in (("MOVE",move),("SETTLE",settle)):
      for r in rr:
        detail.append({"leg":leg,"direction":L["direction"],"phase":phase,**r})

with open(OUT,"w",newline="") as f:
    fields=["leg","direction","phase"]+list(fe[0].keys())
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(detail)

# Focus around LEG2 end press +/- 2 s.
L2=next(L for L in legs if iv(L,"leg")==2)
ep2=kf_ts[iv(L2,"end_press_kf")]
focus=[r for r in fe if ep2-2_000_000_000<=iv(r,"timestamp_ns")<=ep2+3_000_000_000]
print("\n================ LEG2 END WINDOW ================")
print(f'end_press_kf={iv(L2,"end_press_kf")} t={ep2}')
for r in focus:
    dt=(iv(r,"timestamp_ns")-ep2)*1e-9
    print(f'dt={dt:+.3f}s frame={iv(r,"frame_id")} kf={iv(r,"is_keyframe")} '
          f'tracked={iv(r,"tracked_features")} inl={iv(r,"mono_inliers")}/{iv(r,"mono_putatives")} '
          f'ratio={fv(r,"mono_inlier_ratio"):.3f} iters={iv(r,"mono_ransac_iters")} '
          f'status={r["mono_status"]}')

# Backend focus around same event.
bf=[r for r in be if ep2-2_000_000_000<=iv(r,"timestamp_ns")<=ep2+3_000_000_000]
print("\n================ LEG2 BACKEND END WINDOW ================")
prev=None
for r in bf:
    dp=0.0
    if prev is not None:
        dx=fv(r,"px_m")-fv(prev,"px_m");dy=fv(r,"py_m")-fv(prev,"py_m");dz=fv(r,"pz_m")-fv(prev,"pz_m")
        dp=1000*math.sqrt(dx*dx+dy*dy+dz*dz)
    print(f'dt={(iv(r,"timestamp_ns")-ep2)*1e-9:+.3f}s KF={iv(r,"keyframe")} '
          f'P=[{fv(r,"px_m"):.4f},{fv(r,"py_m"):.4f},{fv(r,"pz_m"):.4f}] '
          f'V=[{fv(r,"vx_m_s"):.4f},{fv(r,"vy_m_s"):.4f},{fv(r,"vz_m_s"):.4f}] dPprev={dp:.1f}mm')
    prev=r

print("Saved:",OUT)
