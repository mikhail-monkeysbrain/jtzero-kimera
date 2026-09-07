#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
import numpy as np

run=Path(sys.argv[1])
repo=Path(__file__).resolve().parents[1]
video=run/"ov9281_ruler_pass.avi"
frames=run/"p11_ruler_pass_frames.csv"
calib=repo/"calibration"/"ov9281_intrinsics.yaml"

print("================ P11 RULER TEMPORAL FAMILY TRACK ================")
print("run:",run)
print("Tracks the SAME two image-line families continuously A -> MOVE -> B.")
print("This fixes the previous estimator's frame-wise family switching.")
print("It still does NOT by itself prove physical roll/pitch or the P11 cause.")

fs=cv2.FileStorage(str(calib),cv2.FILE_STORAGE_READ)
K=fs.getNode("camera_matrix").mat(); D=fs.getNode("distortion_coefficients").mat(); fs.release()
if K is None or D is None: raise SystemExit("FAIL: calibration missing")

rows=[]
with frames.open(newline="") as f:
    for r in csv.DictReader(f):
        rows.append({"frame_id":int(r["frame_id"]),"steady_ns":int(r["steady_ns"]),"state":r["state"]})

clahe=cv2.createCLAHE(2.3,(8,8))
lsd=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
def a180(dx,dy): return math.degrees(math.atan2(dy,dx))%180
def dist(a,b):
    d=abs(a-b)%180
    return min(d,180-d)
def signed_delta(a,b):
    d=(b-a)%180
    if d>90:d-=180
    return d

def detect(im):
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    g=cv2.GaussianBlur(clahe.apply(gray),(3,3),0)
    lines,_,_,_=lsd.detect(g)
    if lines is None:return []
    h,w=gray.shape; seg=[]
    for z in lines[:,0,:]:
        x1,y1,x2,y2=map(float,z); dx=x2-x1;dy=y2-y1;ln=math.hypot(dx,dy)
        if ln<28:continue
        mx=(x1+x2)/2;my=(y1+y2)/2
        if my<h*.12:continue
        ang=a180(dx,dy); score=ln*(.6 if mx<w*.03 or mx>w*.97 else 1)
        seg.append((ang,score,ln,(x1,y1,x2,y2)))
    cand=[]
    for ang,_,_,_ in seg:
        near=[s for s in seg if dist(s[0],ang)<=3]
        if len(near)<2:continue
        support=sum(s[1] for s in near)
        if support<100:continue
        vals=[]
        for s in near:
            x=s[0]
            while x-ang>90:x-=180
            while x-ang<-90:x+=180
            vals.append(x)
        med=statistics.median(vals)%180
        cand.append({"angle":med,"support":support,"segments":near})
    cand.sort(key=lambda x:x["support"],reverse=True)
    out=[]
    for c in cand:
        if any(dist(c["angle"],u["angle"])<5 for u in out):continue
        out.append(c)
        if len(out)>=6:break
    return out

# Read every video frame; sample by REAL timestamps at ~12.5 Hz.
cap=cv2.VideoCapture(str(video))
if not cap.isOpened():raise SystemExit("FAIL: video open")
sample=[]; last=None; idx=0
while True:
    ok,im=cap.read()
    if not ok:break
    if idx<len(rows):
        ns=rows[idx]["steady_ns"]
        if last is None or ns-last>=80_000_000:
            sample.append((idx,rows[idx],detect(im),im.copy()))
            last=ns
    idx+=1
cap.release()
print("sampled frames:",len(sample))

# Seed from PRE_STILL_A state-level persistent pair near orthogonal in image.
pre=[x for x in sample if x[1]["state"]=="PRE_STILL_A"]
def state_modes(items):
    allc=[c for _,_,cs,_ in items for c in cs[:4]]
    modes=[]
    for c in allc:
        near=[d for d in allc if dist(d["angle"],c["angle"])<=3]
        hits=sum(any(dist(q["angle"],c["angle"])<=3 for q in cs[:4]) for _,_,cs,_ in items)
        vals=[]
        for d in near:
            x=d["angle"]
            while x-c["angle"]>90:x-=180
            while x-c["angle"]<-90:x+=180
            vals.append(x)
        med=statistics.median(vals)%180
        modes.append((hits,sum(d["support"] for d in near),med))
    modes.sort(reverse=True)
    uniq=[]
    for h,s,a in modes:
        if any(dist(a,u[2])<5 for u in uniq):continue
        uniq.append((h,s,a))
    return uniq[:5]
modes=state_modes(pre)
best=None
for i in range(len(modes)):
    for j in range(i+1,len(modes)):
        sep=dist(modes[i][2],modes[j][2])
        key=(abs(90-sep),-(modes[i][0]+modes[j][0]))
        if best is None or key<best[0]:best=(key,modes[i],modes[j])
if best is None:raise SystemExit("FAIL: cannot seed two families in A")
_,m1,m2=best
seed=[m1[2],m2[2]]
print(f"seed A families: {seed[0]:+.3f}, {seed[1]:+.3f} deg")

# Continuity tracker: prediction is previous angle; hard max jump prevents switching.
tracks=[[],[]]; prev=seed[:]; miss=[0,0]
MAX_STEP=7.0
for idx,r,cs,im in sample:
    used=set()
    for k in range(2):
        choices=[]
        for j,c in enumerate(cs):
            if j in used:continue
            d=dist(prev[k],c["angle"])
            if d<=MAX_STEP:
                choices.append((d,-c["support"],j,c))
        if choices:
            choices.sort(); _,_,j,c=choices[0];used.add(j)
            # unwrap relative to prev
            raw=c["angle"]; delta=signed_delta(prev[k],raw); val=prev[k]+delta
            tracks[k].append({"idx":idx,"state":r["state"],"angle":val,"raw":raw,
                              "support":c["support"],"segments":c["segments"],"im":im})
            prev[k]=val%180;miss[k]=0
        else:
            tracks[k].append({"idx":idx,"state":r["state"],"angle":None,"raw":None,
                              "support":0,"segments":[],"im":im})
            miss[k]+=1
            # Deliberately do not re-seed after misses.

print("\n================ TRACK CONTINUITY ================")
for k,tr in enumerate(tracks,1):
    print(f"family{k}:")
    for st in ("PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"):
        xs=[x["angle"] for x in tr if x["state"]==st and x["angle"] is not None]
        n=sum(x["state"]==st for x in tr)
        if xs:
            print(f"  {st}: hits={len(xs)}/{n} median={statistics.median(xs):+.3f} deg std={statistics.pstdev(xs):.3f}")
        else: print(f"  {st}: hits=0/{n}")

print("\n================ A -> B TRACKED IMAGE ANGLES ================")
ab=[]
for k,tr in enumerate(tracks,1):
    A=[x["angle"] for x in tr if x["state"]=="PRE_STILL_A" and x["angle"] is not None]
    B=[x["angle"] for x in tr if x["state"]=="POST_STILL_B" and x["angle"] is not None]
    if A and B:
        am=statistics.median(A);bm=statistics.median(B);dd=bm-am
        while dd>90:dd-=180
        while dd<-90:dd+=180
        ab.append((am,bm,dd))
        print(f"family{k}: A={am:+.3f} B={bm:+.3f} delta={dd:+.3f} deg")
    else: print(f"family{k}: unavailable")

# A strict gate: both families must survive most of all 3 states.
ok=True
for tr in tracks:
    for st,thr in (("PRE_STILL_A",.8),("MOVE_A_TO_B",.55),("POST_STILL_B",.7)):
        n=sum(x["state"]==st for x in tr); h=sum(x["state"]==st and x["angle"] is not None for x in tr)
        if n==0 or h/n<thr:ok=False

# Contact sheet: representative tracked segments at A/mid/B.
diag=run/"ruler_temporal_track";diag.mkdir(exist_ok=True)
panels=[]
for st in ("PRE_STILL_A","MOVE_A_TO_B","POST_STILL_B"):
    candidates=[]
    for pos in range(len(sample)):
        if sample[pos][1]["state"]!=st:continue
        if tracks[0][pos]["angle"] is not None and tracks[1][pos]["angle"] is not None:candidates.append(pos)
    if not candidates:continue
    pos=candidates[len(candidates)//2]; im=sample[pos][3].copy()
    for k,col in ((0,(0,255,255)),(1,(0,255,0))):
        x=tracks[k][pos]
        for _,_,_,(x1,y1,x2,y2) in x["segments"]:
            cv2.line(im,(int(x1),int(y1)),(int(x2),int(y2)),col,2,cv2.LINE_AA)
        cv2.putText(im,f"F{k+1} {x['angle']:.1f}deg",(10,28+26*k),cv2.FONT_HERSHEY_SIMPLEX,.6,col,2)
    cv2.putText(im,st,(330,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
    panels.append(cv2.resize(im,(426,320)))
if panels:
    sheet=cv2.hconcat(panels);out=diag/"contact_sheet.png";cv2.imwrite(str(out),sheet)
    print("\nAnnotated diagnostics:",out)

print("\n================ GATE ================")
print("PASS" if ok else "FAIL")
if ok:
    print("Both A-seeded line families survived continuously enough through MOVE to B.")
    print("Tracked image-angle changes are now stronger evidence than independent A/B candidate selection.")
else:
    print("At least one A-seeded family did not survive continuity thresholds; do NOT infer A/B orientation from this run.")

print("\nINTERPRETATION:")
print("- PASS means we followed the same angle families rather than choosing a fresh pair in B.")
print("- It still does not turn image angles directly into physical roll/pitch.")
print("- If PASS, the next calculation may use these tracked physical families for calibrated geometry.")
print("- If FAIL, do not write another estimator around this dataset; inspect overlay/target visibility first.")
