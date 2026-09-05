#!/usr/bin/env python3
import csv, math, os, statistics, cv2, numpy as np

H="/home/vio"
CAM=H+"/jtzero_500mm_v13_camera.csv"
MJPG=H+"/jtzero_500mm_v13.mjpg"
LEGS=H+"/jtzero_500mm_v13_legs.csv"
BACKEND=H+"/jtzero_500mm_v13_backend.csv"
OUT=H+"/jtzero_v14_visual_leg_diagnostics.csv"

def read_csv(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def iv(r,k): return int(float(r[k]))
def fv(r,k): return float(r[k])

legs=read_csv(LEGS)
be=read_csv(BACKEND)
cam=read_csv(CAM)
kf_ts={iv(r,"keyframe"):iv(r,"timestamp_ns") for r in be}

selected=[r for r in cam if iv(r,"selected")!=0 and iv(r,"bytes")>0]
selected.sort(key=lambda r:iv(r,"corrected_timestamp_ns"))

orb=cv2.ORB_create(nfeatures=500, fastThreshold=15)
bf=cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

with open(MJPG,"rb") as f:
    blob=f.read()

def frame_from_row(r):
    off=iv(r,"offset"); n=iv(r,"bytes")
    arr=np.frombuffer(blob[off:off+n],dtype=np.uint8)
    return cv2.imdecode(arr,cv2.IMREAD_GRAYSCALE)

def pct(xs,p):
    if not xs:return float("nan")
    return float(np.percentile(np.asarray(xs,dtype=np.float64),p))

rows=[]
for L in legs:
    leg=iv(L,"leg")
    t0=kf_ts[iv(L,"start_settled_kf")]
    t1=kf_ts[iv(L,"end_press_kf")]
    cr=[r for r in selected if t0<=iv(r,"corrected_timestamp_ns")<=t1]

    sharp=[]; kp_counts=[]
    pair_flows=[]; pair_matches=[]; pair_inlier=[]; pair_dt=[]
    prev=None; prev_t=None

    for idx,r in enumerate(cr):
        g=frame_from_row(r)
        if g is None or g.size==0: continue
        sharp.append(float(cv2.Laplacian(g,cv2.CV_64F).var()))
        kp,des=orb.detectAndCompute(g,None)
        kp_counts.append(len(kp))

        if prev is not None:
            dt=(iv(r,"corrected_timestamp_ns")-prev_t)*1e-9
            p0=cv2.goodFeaturesToTrack(prev,maxCorners=300,qualityLevel=0.01,minDistance=7,blockSize=7)
            if p0 is not None and len(p0)>=8:
                p1,st,err=cv2.calcOpticalFlowPyrLK(prev,g,p0,None,winSize=(21,21),maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
                ok=(st.reshape(-1)==1)
                a=p0.reshape(-1,2)[ok]; b=p1.reshape(-1,2)[ok]
                if len(a)>=8:
                    flow=np.linalg.norm(b-a,axis=1)
                    pair_flows.append(float(np.median(flow)))

            kp0,d0=orb.detectAndCompute(prev,None)
            if d0 is not None and des is not None and len(d0)>=8 and len(des)>=8:
                ms=bf.match(d0,des)
                ms=sorted(ms,key=lambda m:m.distance)
                good=ms[:min(120,len(ms))]
                pair_matches.append(len(good))
                if len(good)>=8:
                    a=np.float32([kp0[m.queryIdx].pt for m in good])
                    b=np.float32([kp[m.trainIdx].pt for m in good])
                    Hm,mask=cv2.findHomography(a,b,cv2.RANSAC,3.0)
                    if mask is not None:
                        pair_inlier.append(float(mask.mean()))

            pair_dt.append(dt)
        prev=g; prev_t=iv(r,"corrected_timestamp_ns")

    rows.append({
      "leg":leg,
      "direction":L["direction"],
      "frames":len(cr),
      "sharp_mean":statistics.mean(sharp) if sharp else float("nan"),
      "sharp_p10":pct(sharp,10),
      "kp_mean":statistics.mean(kp_counts) if kp_counts else float("nan"),
      "flow_median_px":statistics.median(pair_flows) if pair_flows else float("nan"),
      "flow_p90_px":pct(pair_flows,90),
      "matches_mean":statistics.mean(pair_matches) if pair_matches else float("nan"),
      "homography_inlier_mean":statistics.mean(pair_inlier) if pair_inlier else float("nan"),
      "homography_inlier_p10":pct(pair_inlier,10),
      "pair_dt_median_ms":statistics.median(pair_dt)*1000 if pair_dt else float("nan"),
    })

with open(OUT,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

print("================ V14 VISUAL LEG DIAGNOSTICS ================")
for r in rows:
    mark="  <-- OUTLIER" if r["leg"]==2 else ""
    print(f'LEG {r["leg"]} {r["direction"]}: frames={r["frames"]} '
          f'sharp={r["sharp_mean"]:.1f} p10={r["sharp_p10"]:.1f} '
          f'kp={r["kp_mean"]:.1f} flowMed={r["flow_median_px"]:.2f}px flowP90={r["flow_p90_px"]:.2f}px '
          f'matches={r["matches_mean"]:.1f} H_inlier={r["homography_inlier_mean"]:.3f} '
          f'H_p10={r["homography_inlier_p10"]:.3f} dtMed={r["pair_dt_median_ms"]:.2f}ms{mark}')

print("\n================ B->A VISUAL COMPARISON ================")
for r in rows:
    if r["leg"] in (2,4,6):
        print(f'LEG {r["leg"]}: sharp={r["sharp_mean"]:.1f} kp={r["kp_mean"]:.1f} '
              f'flow={r["flow_median_px"]:.2f}px matches={r["matches_mean"]:.1f} '
              f'H_inlier={r["homography_inlier_mean"]:.3f} H_p10={r["homography_inlier_p10"]:.3f}')

print("Saved:",OUT)
