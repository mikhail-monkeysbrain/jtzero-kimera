#!/usr/bin/env python3
import csv,math,os
from collections import defaultdict
H="/home/vio"; M=H+"/jtzero_tbs_fine_cross_v13_manifest.csv"
D=H+"/jtzero_tbs_fine_cross_v13_detail.csv"; S=H+"/jtzero_tbs_fine_cross_v13_summary.csv"
def kft(p):
 with open(p,newline="") as f:return {int(r["keyframe"]):int(r["timestamp_ns"]) for r in csv.DictReader(f)}
def legs(p,t):
 with open(p,newline="") as f:return [(int(r["leg"]),t[int(r["start_settled_kf"])],t[int(r["end_settled_kf"])]) for r in csv.DictReader(f)]
def states(p):
 with open(p,newline="") as f:return [(int(r["timestamp_ns"]),float(r["px_m"]),float(r["py_m"]),float(r["pz_m"])) for r in csv.DictReader(f)]
def near(s,t):return min(s,key=lambda q:abs(q[0]-t))
def met(s,L):
 v=[];z2=x2=xsum=0.
 for n,ta,tb in L:
  a,b=near(s,ta),near(s,tb);dx,dy,dz=b[1]-a[1],b[2]-a[2],b[3]-a[3];xy=math.hypot(dx,dy)
  v.append((n,dx,dy,dz,xy));z2+=dz*dz;x2+=(xy-.5)**2;xsum+=xy
 N=len(v);zr=math.sqrt(z2/N)*1000;xr=math.sqrt(x2/N)*1000;by={q[0]:q for q in v};p2=0.;np=0
 for x,y in ((1,2),(3,4),(5,6)):
  a,b=by[x],by[y];sx,sy,sz=a[1]+b[1],a[2]+b[2],a[3]+b[3];p2+=sx*sx+sy*sy+sz*sz;np+=1
 pr=math.sqrt(p2/np)*1000;p0,pn=s[0],s[-1];fd=math.sqrt(sum((pn[i]-p0[i])**2 for i in (1,2,3)))*1000
 return dict(score_mm=zr+xr+pr,z_rms_mm=zr,xy_rms_mm=xr,pair_mm=pr,xy_mean_mm=xsum/N*1000,final_dp_mm=fd)
with open(M,newline="") as f:man=list(csv.DictReader(f))
cache={};rows=[]
for m in man:
 key=(m["backend"],m["legs"])
 if key not in cache:cache[key]=legs(m["legs"],kft(m["backend"]))
 q=H+"/jtzero_extrinsics_replay_v10_"+m["tag"]+".csv"
 rows.append(dict(dataset=m["dataset"],candidate=m["candidate"],roll_deg=float(m["roll_deg"]),pitch_deg=float(m["pitch_deg"]),**met(states(q),cache[key])))
with open(D,"w",newline="") as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
g=defaultdict(list)
for r in rows:g[r["candidate"]].append(r)
a=[]
for c,rr in g.items():
 a.append(dict(candidate=c,roll_deg=rr[0]["roll_deg"],pitch_deg=rr[0]["pitch_deg"],
  worst_score_mm=max(x["score_mm"] for x in rr),mean_score_mm=sum(x["score_mm"] for x in rr)/3,
  worst_z_mm=max(x["z_rms_mm"] for x in rr),worst_xy_mm=max(x["xy_rms_mm"] for x in rr),
  worst_pair_mm=max(x["pair_mm"] for x in rr),worst_final_mm=max(x["final_dp_mm"] for x in rr)))
a.sort(key=lambda x:(x["worst_score_mm"],x["mean_score_mm"]))
with open(S,"w",newline="") as f:
 w=csv.DictWriter(f,fieldnames=["rank"]+list(a[0]));w.writeheader()
 for i,r in enumerate(a,1):w.writerow({"rank":i,**r})
print("================ TBS 0.5 DEG FINE CROSS-VALIDATION ================")
print("rank roll pitch worst_score mean_score worst_Z worst_XY worst_pair worst_final")
for i,r in enumerate(a,1):
 print(f'{i:>2} {r["roll_deg"]:>4.1f} {r["pitch_deg"]:>5.1f} {r["worst_score_mm"]:>11.2f} {r["mean_score_mm"]:>10.2f} {r["worst_z_mm"]:>7.2f} {r["worst_xy_mm"]:>8.2f} {r["worst_pair_mm"]:>10.2f} {r["worst_final_mm"]:>11.2f}')
print("\nPer-dataset for TOP 3:")
for best in a[:3]:
 for r in sorted(g[best["candidate"]],key=lambda x:x["dataset"]):
  print(f'{r["dataset"]} R={r["roll_deg"]:.1f} P={r["pitch_deg"]:.1f} score={r["score_mm"]:.2f} Z={r["z_rms_mm"]:.2f} XY={r["xy_rms_mm"]:.2f} pair={r["pair_mm"]:.2f} meanXY={r["xy_mean_mm"]:.2f} final={r["final_dp_mm"]:.2f}')
print("Saved:",S)
