#!/usr/bin/env python3
import sys, cv2, math, statistics
from pathlib import Path
import numpy as np

root=Path(sys.argv[1])
stages=["A1","B1","A2","B2","A3","B3","A4"]
diag=root/"ruler_diag"
diag.mkdir(exist_ok=True)

def angle180(dx,dy):
    a=math.degrees(math.atan2(dy,dx))
    while a<0: a+=180
    while a>=180: a-=180
    return a

def circ_dist180(a,b):
    d=abs(a-b)%180
    return min(d,180-d)

def detect_ruler_like(im):
    h,w=im.shape[:2]
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    # The ruler is visible in the lower/central field in the recorded setup.
    # This is only a candidate detector, not a semantic proof.
    y0=int(h*0.28)
    roi=gray[y0:h,:]
    roi=cv2.GaussianBlur(roi,(5,5),0)
    edges=cv2.Canny(roi,45,120)
    lines=cv2.HoughLinesP(edges,1,np.pi/180,threshold=55,
                          minLineLength=max(90,int(w*0.18)),maxLineGap=18)
    cand=[]
    if lines is not None:
        for l in lines[:,0]:
            x1,y1,x2,y2=map(int,l)
            y1+=y0; y2+=y0
            dx=x2-x1; dy=y2-y1
            ln=math.hypot(dx,dy)
            if ln<90: continue
            a=angle180(dx,dy)
            # Reject near-vertical frame borders; otherwise stay permissive.
            if 70<a<110: continue
            cand.append((ln,a,(x1,y1,x2,y2)))
    if not cand:
        return None,[]

    # Find the orientation with the largest total line-length support.
    best=None
    for _,a,_ in cand:
        support=sum(ln for ln,b,_ in cand if circ_dist180(a,b)<=4.0)
        if best is None or support>best[0]:
            best=(support,a)
    center=best[1]
    cluster=[x for x in cand if circ_dist180(center,x[1])<=4.0]
    angles=[x[1] for x in cluster]
    lengths=[x[0] for x in cluster]

    # Resolve wrap around 0/180 around the chosen center.
    unfolded=[]
    for a in angles:
        x=a
        while x-center>90: x-=180
        while x-center<-90: x+=180
        unfolded.append(x)
    med=statistics.median(unfolded)
    while med<0: med+=180
    while med>=180: med-=180

    return {
        "angle":med,
        "support":sum(lengths),
        "n":len(cluster),
        "maxlen":max(lengths),
    },cluster

print("================ P11 RULER OBSERVABILITY GATE ================")
print("run:",root)
print("Purpose: test whether the visible ruler provides a repeatable 1-D line reference.")
print("This does NOT estimate full camera roll/pitch and is NOT a causal orientation proof.")

stage_angles={}
thumbs=[]

for st in stages:
    imgs=sorted(root.glob(f"{st}_*.png"))
    vals=[]
    rep=None
    rep_cluster=[]
    for idx,p in enumerate(imgs):
        im=cv2.imread(str(p))
        if im is None: continue
        res,cluster=detect_ruler_like(im)
        if res and res["support"]>=180 and res["n"]>=2:
            vals.append(res["angle"])
        if idx==len(imgs)//2:
            rep=im.copy(); rep_cluster=cluster

    print(f"\n{st}: usable={len(vals)}/{len(imgs)}")
    if vals:
        # unwrap around median-ish first value for stable std
        ref=vals[0]
        uu=[]
        for a in vals:
            x=a
            while x-ref>90: x-=180
            while x-ref<-90: x+=180
            uu.append(x)
        med=statistics.median(uu)
        sd=statistics.pstdev(uu) if len(uu)>1 else 0.0
        stage_angles[st]=med
        print(f"  candidate ruler-line angle median={med:+.3f} deg std={sd:.3f} deg")
        print(f"  NOTE: line angle is image-space only.")
    else:
        print("  FAIL: ruler-like line not observed reliably.")

    if rep is not None:
        for ln,a,(x1,y1,x2,y2) in rep_cluster:
            cv2.line(rep,(x1,y1),(x2,y2),(0,255,255),2)
        cv2.putText(rep,st,(12,32),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,0,255),2)
        out=diag/f"{st}_candidate.png"
        cv2.imwrite(str(out),rep)
        small=cv2.resize(rep,(320,240))
        thumbs.append((st,small))

print("\n================ REPEATABILITY ================")
for group in (["A1","A2","A3","A4"],["B1","B2","B3"]):
    arr=[stage_angles[x] for x in group if x in stage_angles]
    if len(arr)>=2:
        ref=arr[0]
        uu=[]
        for a in arr:
            x=a
            while x-ref>90: x-=180
            while x-ref<-90: x+=180
            uu.append(x)
        print(f"{group[0][0]} plateaus: median={statistics.median(uu):+.3f} deg span={max(uu)-min(uu):.3f} deg")

print("\n================ ADJACENT A/B IMAGE-LINE DELTA ================")
for a,b in [("A1","B1"),("A2","B2"),("A3","B3")]:
    if a in stage_angles and b in stage_angles:
        d=stage_angles[b]-stage_angles[a]
        while d>90: d-=180
        while d<-90: d+=180
        print(f"{a}->{b}: image-line angle delta={d:+.3f} deg")
    else:
        print(f"{a}->{b}: unavailable")

if thumbs:
    rows=[]
    for i in range(0,len(thumbs),4):
        ims=[x[1] for x in thumbs[i:i+4]]
        while len(ims)<4: ims.append(np.zeros_like(thumbs[0][1]))
        rows.append(cv2.hconcat(ims))
    sheet=cv2.vconcat(rows)
    cv2.imwrite(str(diag/"contact_sheet.png"),sheet)
    print(f"\nAnnotated diagnostics: {diag/'contact_sheet.png'}")

print("\nINTERPRETATION:")
print("- Low within-A/within-B span means the detected line reference is repeatable at each position.")
print("- A/B line-angle difference is only a 1-D image projection effect; it cannot by itself prove physical roll/pitch.")
print("- If annotated images show the detector locked onto a non-ruler edge, discard the result.")
print("- This gate is useful only to decide whether the ruler can serve as an auxiliary visual reference.")
