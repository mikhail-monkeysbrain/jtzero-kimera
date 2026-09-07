#!/usr/bin/env python3
import sys, csv, cv2, math, statistics
from pathlib import Path
import numpy as np

run = Path(sys.argv[1])
repo = Path(__file__).resolve().parents[1]
video_path = run / "ov9281_ruler_pass.avi"
frames_csv = run / "p11_ruler_pass_frames.csv"
calib_path = repo / "calibration" / "ov9281_intrinsics.yaml"

print("================ P11 RULER PLANE-NORMAL GATE ================")
print("run:", run)
print("Goal: estimate camera-relative plane normal from two line families")
print("on PRE_STILL_A and POST_STILL_B only.")
print("This is still a ruler/guide geometry diagnostic, not an independent P11 proof.")

fs = cv2.FileStorage(str(calib_path), cv2.FILE_STORAGE_READ)
K = fs.getNode("camera_matrix").mat()
D = fs.getNode("distortion_coefficients").mat()
fs.release()
if K is None or D is None:
    print("FAIL: camera calibration not loaded")
    sys.exit(2)

print(f"K fx/fy={K[0,0]:.3f}/{K[1,1]:.3f} cx/cy={K[0,2]:.3f}/{K[1,2]:.3f}")

rows=[]
with frames_csv.open(newline="") as f:
    for r in csv.DictReader(f):
        rows.append({
            "frame_id": int(r["frame_id"]),
            "steady_ns": int(r["steady_ns"]),
            "state": r["state"],
        })

cap=cv2.VideoCapture(str(video_path))
if not cap.isOpened():
    print("FAIL: cannot open video")
    sys.exit(3)

clahe=cv2.createCLAHE(clipLimit=2.3,tileGridSize=(8,8))
lsd=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)

def a180(dx,dy):
    return math.degrees(math.atan2(dy,dx)) % 180.0

def cd180(a,b):
    d=abs(a-b)%180.0
    return min(d,180.0-d)

def clusters(im):
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    g=clahe.apply(gray)
    g=cv2.GaussianBlur(g,(3,3),0)
    lines,_,_,_=lsd.detect(g)
    segs=[]
    h,w=gray.shape
    if lines is None:
        return []
    for l in lines[:,0,:]:
        x1,y1,x2,y2=map(float,l)
        dx=x2-x1; dy=y2-y1
        ln=math.hypot(dx,dy)
        if ln<30: continue
        mx=(x1+x2)*0.5; my=(y1+y2)*0.5
        if my<h*0.12: continue
        ang=a180(dx,dy)
        score=ln*(0.6 if (mx<w*0.03 or mx>w*0.97) else 1.0)
        segs.append((score,ln,ang,(x1,y1,x2,y2)))
    if not segs: return []

    cand=[]
    for _,_,ang,_ in segs:
        cl=[s for s in segs if cd180(s[2],ang)<=3.0]
        support=sum(s[0] for s in cl)
        if len(cl)<2 or support<100: continue
        aa=[]
        for s in cl:
            x=s[2]
            while x-ang>90: x-=180
            while x-ang<-90: x+=180
            aa.append(x)
        med=statistics.median(aa)%180.0
        cand.append({"angle":med,"support":support,"segments":cl})
    cand.sort(key=lambda c:c["support"], reverse=True)
    out=[]
    for c in cand:
        if any(cd180(c["angle"],u["angle"])<5.0 for u in out): continue
        out.append(c)
        if len(out)>=6: break
    return out

def vanishing_direction(seg_cluster):
    # Undistort endpoints to calibrated pixel coordinates, then fit a homogeneous vanishing point.
    A=[]
    weights=[]
    for score,ln,ang,(x1,y1,x2,y2) in seg_cluster:
        pts=np.array([[[x1,y1]],[[x2,y2]]],dtype=np.float64)
        und=cv2.undistortPoints(pts,K,D,P=K).reshape(2,2)
        p1=np.array([und[0,0],und[0,1],1.0])
        p2=np.array([und[1,0],und[1,1],1.0])
        l=np.cross(p1,p2)
        n=math.hypot(l[0],l[1])
        if n<1e-9: continue
        l=l/n
        A.append(l)
        weights.append(max(1.0,ln))
    if len(A)<2:
        return None
    A=np.asarray(A)
    W=np.diag(np.asarray(weights)/max(weights))
    M=A.T@W@A
    vals,vecs=np.linalg.eigh(M)
    v=vecs[:,np.argmin(vals)]
    if np.linalg.norm(v)<1e-12:
        return None
    d=np.linalg.inv(K)@v
    dn=np.linalg.norm(d)
    if dn<1e-12:
        return None
    d=d/dn
    return d

def choose_pair(cs):
    best=None
    for i in range(len(cs)):
        d1=vanishing_direction(cs[i]["segments"])
        if d1 is None: continue
        for j in range(i+1,len(cs)):
            d2=vanishing_direction(cs[j]["segments"])
            if d2 is None: continue
            image_sep=cd180(cs[i]["angle"],cs[j]["angle"])
            if not (65.0 <= image_sep <= 115.0):
                continue
            ortho_err=abs(math.degrees(math.acos(np.clip(abs(float(np.dot(d1,d2))),0,1)))-90.0)
            # Favor 3D orthogonality, then support.
            support=cs[i]["support"]+cs[j]["support"]
            key=(ortho_err,-support)
            if best is None or key<best[0]:
                n=np.cross(d1,d2)
                nn=np.linalg.norm(n)
                if nn<1e-9: continue
                n=n/nn
                if n[2]<0: n=-n
                best=(key,cs[i],cs[j],d1,d2,n,ortho_err,image_sep)
    return best

def normal_angles(n):
    tilt=math.degrees(math.acos(np.clip(n[2],-1,1)))
    az=math.degrees(math.atan2(n[1],n[0]))
    return tilt,az

# Analyze at ~8 Hz, still states only.
targets=[]
last_ns={}
for i,r in enumerate(rows):
    st=r["state"]
    if st not in ("PRE_STILL_A","POST_STILL_B"):
        continue
    if st not in last_ns or r["steady_ns"]-last_ns[st]>=120_000_000:
        targets.append(i)
        last_ns[st]=r["steady_ns"]

targetset=set(targets)
obs={"PRE_STILL_A":[],"POST_STILL_B":[]}
idx=0
while True:
    ok,im=cap.read()
    if not ok: break
    if idx in targetset and idx<len(rows):
        st=rows[idx]["state"]
        cs=clusters(im)
        p=choose_pair(cs)
        if p is not None:
            _,c1,c2,d1,d2,n,oe,isep=p
            tilt,az=normal_angles(n)
            obs[st].append({
                "frame":idx,"n":n,"tilt":tilt,"az":az,
                "ortho_err":oe,"image_sep":isep,
                "a1":c1["angle"],"a2":c2["angle"]
            })
    idx+=1
cap.release()

def align_normals(items):
    if not items: return []
    ref=items[0]["n"]
    out=[]
    for x in items:
        y=dict(x)
        n=x["n"].copy()
        if float(np.dot(n,ref))<0: n=-n
        if n[2]<0: n=-n
        y["n"]=n
        y["tilt"],y["az"]=normal_angles(n)
        out.append(y)
    return out

for st in obs:
    obs[st]=align_normals(obs[st])

print("\n================ STILL-STATE PLANE NORMALS ================")
summary={}
for st in ("PRE_STILL_A","POST_STILL_B"):
    items=obs[st]
    print(f"\n{st}: valid={len(items)}/{sum(1 for i in targets if rows[i]['state']==st)}")
    if len(items)<5:
        print("  FAIL: insufficient valid two-family frames")
        continue
    normals=np.stack([x["n"] for x in items])
    mean=normals.mean(axis=0)
    mean=mean/np.linalg.norm(mean)
    if mean[2]<0: mean=-mean
    angdev=[math.degrees(math.acos(np.clip(float(np.dot(x["n"],mean)),-1,1))) for x in items]
    tilt,az=normal_angles(mean)
    oe=[x["ortho_err"] for x in items]
    summary[st]={"n":mean,"tilt":tilt,"az":az,"spread":statistics.median(angdev),
                 "p90":float(np.percentile(angdev,90)),"ortho":statistics.median(oe)}
    print(f"  mean normal=[{mean[0]:+.5f},{mean[1]:+.5f},{mean[2]:+.5f}]")
    print(f"  tilt-from-optical-axis={tilt:.3f} deg  normal-azimuth={az:+.3f} deg")
    print(f"  normal spread median/p90={statistics.median(angdev):.3f}/{np.percentile(angdev,90):.3f} deg")
    print(f"  3D orthogonality error median={statistics.median(oe):.3f} deg")

if "PRE_STILL_A" in summary and "POST_STILL_B" in summary:
    a=summary["PRE_STILL_A"]["n"]
    b=summary["POST_STILL_B"]["n"]
    d=math.degrees(math.acos(np.clip(float(np.dot(a,b)),-1,1)))
    print("\n================ A -> B NORMAL CHANGE ================")
    print(f"angular change between mean plane normals = {d:.3f} deg")
    print(f"tilt change = {summary['POST_STILL_B']['tilt']-summary['PRE_STILL_A']['tilt']:+.3f} deg")
    azd=summary["POST_STILL_B"]["az"]-summary["PRE_STILL_A"]["az"]
    while azd>180: azd-=360
    while azd<-180: azd+=360
    print(f"normal azimuth change = {azd:+.3f} deg")

print("\nINTERPRETATION:")
print("- Trust this only if valid-frame count is high, normal spread is small, and 3D orthogonality error is small.")
print("- The result estimates camera-relative normal of the ruler/guide plane from line geometry; it is stronger than raw image-line angles.")
print("- Because the ruler is the mechanical guide, an A/B normal change is evidence of guide/stand/surface geometry change, not proof of IMU error or of the ruler alone being causal.")
print("- Do not compare directly with the approximate 45-deg camera mounting angle; that value was not independently calibrated.")
