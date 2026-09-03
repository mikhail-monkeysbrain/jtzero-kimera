#!/usr/bin/env python3
import csv, math, os, sys
import cv2
import numpy as np

CAM='/home/vio/jtzero_yaw_only_v15_42_camera.csv'
MJPG='/home/vio/jtzero_yaw_only_v15_42.mjpg'
ATT='/home/vio/jtzero_yaw_only_v15_42_attitude.csv'
OUT='/home/vio/jtzero_homography_decomp_v15_49.csv'
K=np.array([[568.53170752165227,0,315.98271077441063],[0,569.68005562865858,239.88148589100641],[0,0,1]],float)
D=np.array([0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483],float)
RBC=np.array([[0.009367371,0.999954838,-0.001604448],[0.999326771,-0.009418381,-0.035458408],[-0.035471918,-0.001271215,-0.999369865]],float)

def rpy(r,p,y):
    r,p,y=np.deg2rad([r,p,y]); cr,sr=np.cos(r),np.sin(r); cp,sp=np.cos(p),np.sin(p); cy,sy=np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def rang(R): return math.degrees(math.acos(np.clip((np.trace(R)-1)*.5,-1,1)))
def aang(a,b,signless=False):
    a=np.asarray(a,float);b=np.asarray(b,float); na=np.linalg.norm(a);nb=np.linalg.norm(b)
    if na<1e-12 or nb<1e-12:return float('nan')
    c=float(np.dot(a,b)/(na*nb)); c=abs(c) if signless else c
    return math.degrees(math.acos(np.clip(c,-1,1)))
def med(x):
    x=np.asarray([v for v in x if np.isfinite(v)],float); return float(np.median(x)) if len(x) else float('nan')
def p90(x):
    x=np.asarray([v for v in x if np.isfinite(v)],float); return float(np.percentile(x,90)) if len(x) else float('nan')
def concentration(vs, signless=False):
    q=[]
    for v in vs:
        v=np.asarray(v,float); n=np.linalg.norm(v)
        if n>1e-12:q.append(v/n)
    if not q:return float('nan')
    if signless:
        M=sum(np.outer(v,v) for v in q)/len(q); return float(np.linalg.eigvalsh(M)[-1])
    return float(np.linalg.norm(np.mean(q,axis=0)))

def load_att():
    with open(ATT,newline='') as f:
        a=[]
        for r in csv.DictReader(f): a.append((int(r['mapped_rpi_ns']),float(r['roll_deg']),float(r['pitch_deg']),float(r['yaw_deg']),float(r['yawspeed'])))
    return a
def nearest(a,t,h):
    while h+1<len(a) and abs(a[h+1][0]-t)<=abs(a[h][0]-t):h+=1
    return a[h],h
def load_cam():
    rows=[]; prev_seq=None;prev_ts=None;last=0
    with open(CAM,newline='') as f:
      for r in csv.DictReader(f):
        seq=int(r['sequence']);ts=int(r['camera_timestamp_corrected_ns']);off=int(r['mjpeg_offset']);n=int(r['bytes_used'])
        ok=prev_seq is None or (seq==prev_seq+1 and 0<ts-prev_ts<=20_000_000); prev_seq,prev_ts=seq,ts
        due=last==0 or ts-last>=30_000_000
        if ok and due and n>0:rows.append((seq,ts,off,n));last=ts
    return rows
def decode(f,r):
    f.seek(r[2]); b=f.read(r[3]); return cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_GRAYSCALE)

A=load_att(); C=load_cam(); ah0=ah1=0; results=[]
with open(MJPG,'rb') as jf:
    prev=decode(jf,C[0])
    for i in range(1,len(C)):
        cur=decode(jf,C[i]);
        if prev is None or cur is None: prev=cur; continue
        p0=cv2.goodFeaturesToTrack(prev,350,.01,7,blockSize=7)
        if p0 is None or len(p0)<20: prev=cur;continue
        p1,st,_=cv2.calcOpticalFlowPyrLK(prev,cur,p0,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        st=st.reshape(-1).astype(bool); q0=p0.reshape(-1,2)[st];q1=p1.reshape(-1,2)[st]
        good=(q1[:,0]>=0)&(q1[:,0]<cur.shape[1])&(q1[:,1]>=0)&(q1[:,1]<cur.shape[0]);q0=q0[good];q1=q1[good]
        if len(q0)<20:prev=cur;continue
        u0=cv2.undistortPoints(q0.reshape(-1,1,2),K,D,P=K).reshape(-1,2);u1=cv2.undistortPoints(q1.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
        H,mask=cv2.findHomography(u0,u1,cv2.RANSAC,1.5,maxIters=2000,confidence=.995)
        if H is None:prev=cur;continue
        a0,ah0=nearest(A,C[i-1][1],ah0);a1,ah1=nearest(A,C[i][1],ah1)
        # Correct camera relative rotation direction established by v15.45b.
        R0=rpy(a0[1],a0[2],a0[3]);R1=rpy(a1[1],a1[2],a1[3]); Rfc=RBC.T@R0.T@R1@RBC
        rotating=abs(.5*(a0[4]+a1[4]))*180/math.pi>2 or rang(Rfc)>.08
        try: nsol,Rs,Ts,Ns=cv2.decomposeHomographyMat(H,K)
        except cv2.error:prev=cur;continue
        cand=[]
        for j in range(nsol):
            er=rang(np.asarray(Rs[j])@Rfc.T); t=np.asarray(Ts[j]).reshape(3);n=np.asarray(Ns[j]).reshape(3)
            cand.append((er,t,n))
        er,t,n=min(cand,key=lambda x:x[0])
        results.append((C[i-1][1],C[i][1],rotating,er,*t,*n))
        prev=cur

rot=[r for r in results if r[2]]
ts=[np.array(r[4:7]) for r in rot]; ns=[np.array(r[7:10]) for r in rot]; ers=[r[3] for r in rot]
tstep=[aang(ts[i-1],ts[i]) for i in range(1,len(ts))]; nstep=[aang(ns[i-1],ns[i],True) for i in range(1,len(ns))]
flips=sum(1 for i in range(1,len(ts)) if np.dot(ts[i-1],ts[i])<0)
tconc=concentration(ts); tconc_axis=concentration(ts,True); nconc=concentration(ns,True)
with open(OUT,'w',newline='') as f:
    w=csv.writer(f);w.writerow(['t0_ns','t1_ns','rotating','rot_error_vs_fc_deg','tx','ty','tz','nx','ny','nz']);w.writerows(results)
print('='*60);print('JT-ZERO v15.49 HOMOGRAPHY PHYSICAL DECOMPOSITION');print('='*60)
print(f'ALL_PAIRS={len(results)} ROTATION_PAIRS={len(rot)}')
print(f'ROTATION_MEDIAN_R_ERROR_VS_FC_DEG={med(ers):.4f}')
print(f'ROTATION_P90_R_ERROR_VS_FC_DEG={p90(ers):.4f}')
print(f'T_DIR_STEP_MEDIAN_DEG={med(tstep):.4f}')
print(f'T_DIR_STEP_P90_DEG={p90(tstep):.4f}')
print(f'T_DIR_SIGN_FLIPS={flips}')
print(f'T_DIR_CONCENTRATION_VECTOR={tconc:.4f}')
print(f'T_DIR_CONCENTRATION_AXIS={tconc_axis:.4f}')
print(f'PLANE_NORMAL_STEP_MEDIAN_DEG={med(nstep):.4f}')
print(f'PLANE_NORMAL_STEP_P90_DEG={p90(nstep):.4f}')
print(f'PLANE_NORMAL_AXIS_CONCENTRATION={nconc:.4f}')
if len(rot)<30: verdict='INSUFFICIENT_ROTATION_DATA'
elif med(ers)<1.0 and (med(tstep)>30 or p90(tstep)>80 or tconc_axis<0.70): verdict='ROTATION_STABLE_TRANSLATION_DIRECTION_POORLY_OBSERVED'
elif med(ers)<1.0 and tconc_axis>=0.85: verdict='ROTATION_AND_TRANSLATION_DIRECTION_STRUCTURED'
else: verdict='HOMOGRAPHY_DECOMPOSITION_INCONCLUSIVE'
print('V15_49_VERDICT='+verdict);print('CSV='+OUT);print('RESULT: COMPLETE')
