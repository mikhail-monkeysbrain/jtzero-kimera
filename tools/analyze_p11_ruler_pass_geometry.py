#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

run = Path(sys.argv[1])
video_path = run / "ov9281_ruler_pass.avi"
frames_csv = run / "p11_ruler_pass_frames.csv"
events_csv = run / "p11_ruler_pass_events.csv"

print("================ P11 RULER PASS GEOMETRY GATE ================")
print("run:", run)
print("Goal: test whether two repeatable line families of the ruler are observable")
print("through PRE_STILL_A -> MOVE_A_TO_B -> POST_STILL_B.")
print("This is a mechanical/visual geometry diagnostic, not an independent proof of P11 cause.")

if not video_path.exists() or not frames_csv.exists():
    print("FAIL: missing video or frame CSV")
    sys.exit(2)

rows=[]
with frames_csv.open(newline="") as f:
    rd=csv.DictReader(f)
    for r in rd:
        rows.append({
            "frame_id": int(r["frame_id"]),
            "steady_ns": int(r["steady_ns"]),
            "state": r["state"],
        })

if not rows:
    print("FAIL: frame CSV empty")
    sys.exit(3)

# Timing diagnostics from real capture timestamps.
dts=[(rows[i]["steady_ns"]-rows[i-1]["steady_ns"])*1e-9 for i in range(1,len(rows))
     if rows[i]["steady_ns"]>rows[i-1]["steady_ns"]]
print(f"frames in CSV: {len(rows)}")
if dts:
    print(f"capture dt median={statistics.median(dts)*1000:.2f} ms "
          f"p90={np.percentile(dts,90)*1000:.2f} ms "
          f"effective FPS~={1.0/statistics.median(dts):.1f}")

counts=defaultdict(int)
for r in rows: counts[r["state"]]+=1
for st in ["PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"]:
    print(f"{st}: frames={counts.get(st,0)}")

cap=cv2.VideoCapture(str(video_path))
if not cap.isOpened():
    print("FAIL: cannot open video")
    sys.exit(4)

video_n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
video_fps=float(cap.get(cv2.CAP_PROP_FPS))
print(f"video container: frames={video_n} fps={video_fps:.3f}")
if abs(video_n-len(rows))>2:
    print("WARNING: video frame count differs from frame CSV count; index alignment may be imperfect.")

def angle180(dx,dy):
    return math.degrees(math.atan2(dy,dx)) % 180.0

def cd180(a,b):
    d=abs(a-b)%180.0
    return min(d,180.0-d)

def unwrap(vals,ref):
    out=[]
    for a in vals:
        x=a
        while x-ref>90: x-=180
        while x-ref<-90: x+=180
        out.append(x)
    return out

clahe=cv2.createCLAHE(clipLimit=2.3,tileGridSize=(8,8))
lsd=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)

def detect_clusters(im):
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    g=clahe.apply(gray)
    g=cv2.GaussianBlur(g,(3,3),0)
    lines,_,_,_=lsd.detect(g)
    segs=[]
    h,w=gray.shape
    if lines is not None:
        for l in lines[:,0,:]:
            x1,y1,x2,y2=map(float,l)
            dx=x2-x1; dy=y2-y1
            ln=math.hypot(dx,dy)
            if ln<28:
                continue
            mx=(x1+x2)*0.5; my=(y1+y2)*0.5
            # Broad central/lower ROI: avoid top UI-like or border artifacts.
            if my<h*0.12:
                continue
            a=angle180(dx,dy)
            score=ln
            if mx<w*0.03 or mx>w*0.97:
                score*=0.6
            segs.append((score,ln,a,(int(x1),int(y1),int(x2),int(y2))))
    if not segs:
        return []

    cand=[]
    for _,_,a,_ in segs:
        cl=[s for s in segs if cd180(s[2],a)<=3.0]
        support=sum(s[0] for s in cl)
        med=statistics.median(unwrap([s[2] for s in cl],a))%180.0
        cand.append({"angle":med,"support":support,"n":len(cl),"segments":cl})
    cand.sort(key=lambda x:x["support"], reverse=True)
    uniq=[]
    for c in cand:
        if any(cd180(c["angle"],u["angle"])<5.0 for u in uniq):
            continue
        uniq.append(c)
        if len(uniq)>=5:
            break
    return uniq

# Sample at max 12 Hz by real timestamps to limit correlation.
sample_idx=[]
last_ns=None
for i,r in enumerate(rows):
    if last_ns is None or r["steady_ns"]-last_ns>=80_000_000:
        sample_idx.append(i)
        last_ns=r["steady_ns"]

frame_map={}
target=set(sample_idx)
idx=0
while True:
    ok,im=cap.read()
    if not ok: break
    if idx in target:
        frame_map[idx]=im
    idx+=1
cap.release()

obs=[]
for i in sample_idx:
    if i not in frame_map or i>=len(rows):
        continue
    cs=detect_clusters(frame_map[i])
    obs.append((i,rows[i],cs))

print(f"sampled frames analyzed: {len(obs)}")

# Build state-level orientation hypotheses.
def state_hypothesis(state):
    vals=[]
    frame_hits=0
    for i,r,cs in obs:
        if r["state"]!=state:
            continue
        for rank,c in enumerate(cs[:4]):
            vals.append((c["angle"], c["support"]/(rank+1)))
    if not vals:
        return []
    hyps=[]
    for a,_ in vals:
        near=[v for v in vals if cd180(v[0],a)<=3.0]
        score=sum(v[1] for v in near)
        med=statistics.median(unwrap([v[0] for v in near],a))%180.0
        per=[]
        for i,r,cs in obs:
            if r["state"]!=state:
                continue
            hit=[c for c in cs[:4] if cd180(c["angle"],med)<=3.0]
            if hit: per.append(hit[0]["angle"])
        uu=unwrap(per,med) if per else []
        sd=statistics.pstdev(uu) if len(uu)>1 else 0.0
        hyps.append({"angle":med,"score":score,"hits":len(per),"std":sd})
    hyps.sort(key=lambda h:(h["hits"],h["score"]),reverse=True)
    uniq=[]
    for h in hyps:
        if any(cd180(h["angle"],u["angle"])<5.0 for u in uniq):
            continue
        uniq.append(h)
        if len(uniq)>=4: break
    return uniq

state_h={}
print("\n================ STATE LINE-FAMILY CANDIDATES ================")
for st in ["PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"]:
    hs=state_hypothesis(st)
    state_h[st]=hs
    nstate=sum(1 for _,r,_ in obs if r["state"]==st)
    print(f"\n{st}: sampled={nstate}")
    if not hs:
        print("  no stable candidates")
    for j,h in enumerate(hs,1):
        print(f"  C{j}: angle={h['angle']:+.3f} deg hits={h['hits']}/{nstate} std={h['std']:.3f} deg")

# Search for an approximately orthogonal pair in A and B.
def best_pair(hs):
    best=None
    for i in range(len(hs)):
        for j in range(i+1,len(hs)):
            sep=cd180(hs[i]["angle"],hs[j]["angle"])
            ortho_err=abs(90.0-sep)
            support=hs[i]["hits"]+hs[j]["hits"]
            key=(ortho_err,-support)
            if best is None or key<best[0]:
                best=(key,hs[i],hs[j],sep)
    return best

print("\n================ TWO-FAMILY OBSERVABILITY ================")
pairs={}
for st in ["PRE_STILL_A","POST_STILL_B"]:
    p=best_pair(state_h.get(st,[]))
    pairs[st]=p
    if p is None:
        print(f"{st}: no 2-family pair")
    else:
        _,a,b,sep=p
        print(f"{st}: pair={a['angle']:+.3f}/{b['angle']:+.3f} deg "
              f"separation={sep:.3f} deg orthogonality_error={abs(90-sep):.3f} deg")

if pairs.get("PRE_STILL_A") and pairs.get("POST_STILL_B"):
    _,a1,a2,_=pairs["PRE_STILL_A"]
    _,b1,b2,_=pairs["POST_STILL_B"]
    # Match A families to B families by minimal total angular distance.
    d_same=cd180(a1["angle"],b1["angle"])+cd180(a2["angle"],b2["angle"])
    d_swap=cd180(a1["angle"],b2["angle"])+cd180(a2["angle"],b1["angle"])
    if d_swap<d_same:
        b1,b2=b2,b1
    print("\nA->B candidate-family deltas:")
    print(f"  family1: {a1['angle']:+.3f} -> {b1['angle']:+.3f}  delta={cd180(a1['angle'],b1['angle']):.3f} deg")
    print(f"  family2: {a2['angle']:+.3f} -> {b2['angle']:+.3f}  delta={cd180(a2['angle'],b2['angle']):.3f} deg")

# Annotated contact sheet of representative state frames.
diag=run/"ruler_pass_diag"
diag.mkdir(exist_ok=True)
panels=[]
for st in ["PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"]:
    candidates=[x for x in obs if x[1]["state"]==st]
    if not candidates: continue
    i,r,cs=candidates[len(candidates)//2]
    im=frame_map[i].copy()
    colors=[(0,255,255),(0,255,0),(255,0,255),(255,255,0)]
    for rank,c in enumerate(cs[:4]):
        col=colors[rank]
        for _,_,_,(x1,y1,x2,y2) in c["segments"]:
            cv2.line(im,(x1,y1),(x2,y2),col,2,cv2.LINE_AA)
        cv2.putText(im,f"C{rank+1} {c['angle']:.1f}deg",
                    (10,28+24*rank),cv2.FONT_HERSHEY_SIMPLEX,0.55,col,2,cv2.LINE_AA)
    cv2.putText(im,st,(360,30),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,0,255),2,cv2.LINE_AA)
    cv2.imwrite(str(diag/f"{st}.png"),im)
    panels.append(cv2.resize(im,(426,320)))

if panels:
    sheet=cv2.hconcat(panels)
    cv2.imwrite(str(diag/"contact_sheet.png"),sheet)
    print(f"\nAnnotated diagnostics: {diag/'contact_sheet.png'}")

print("\nINTERPRETATION:")
print("- First verify the overlay: the two selected families must actually correspond to ruler edges/ticks.")
print("- Stable two-family geometry in both A and B means the raw video is suitable for a stronger orientation estimator.")
print("- A/B line-family angle changes are image-space evidence only; do not equate them directly to roll/pitch.")
print("- Because the ruler is the mechanical guide, any change is a guide/stand/surface geometry effect candidate, not an independent sensor reference.")
