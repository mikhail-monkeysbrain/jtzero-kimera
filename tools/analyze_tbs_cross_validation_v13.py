#!/usr/bin/env python3
import csv, math, os, sys
from collections import defaultdict

HOME="/home/vio"
MANIFEST=os.path.join(HOME,"jtzero_tbs_cross_validation_v13_manifest.csv")
DETAIL=os.path.join(HOME,"jtzero_tbs_cross_validation_v13_detail.csv")
SUMMARY=os.path.join(HOME,"jtzero_tbs_cross_validation_v13_summary.csv")

def kft(path):
    with open(path,newline="") as f:
        return {int(r["keyframe"]):int(r["timestamp_ns"]) for r in csv.DictReader(f)}

def legs(path,t):
    out=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            sk,ek=int(r["start_settled_kf"]),int(r["end_settled_kf"])
            out.append((int(r["leg"]),t[sk],t[ek]))
    return out

def states(path):
    out=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            out.append((int(r["timestamp_ns"]),float(r["px_m"]),float(r["py_m"]),float(r["pz_m"])))
    return out

def near(s,t):
    return min(s,key=lambda q:abs(q[0]-t))

def calc(s,L):
    vv=[]; z2=xy2=xys=0.0
    for leg,a0,b0 in L:
        a,b=near(s,a0),near(s,b0)
        dx,dy,dz=b[1]-a[1],b[2]-a[2],b[3]-a[3]
        xy=math.hypot(dx,dy)
        vv.append((leg,dx,dy,dz,xy))
        z2+=dz*dz; xy2+=(xy-.5)**2; xys+=xy
    n=len(vv)
    zr=math.sqrt(z2/n)*1000; xr=math.sqrt(xy2/n)*1000
    by={x[0]:x for x in vv}; p2=pxy2=0.0; np=0
    for la,lb in ((1,2),(3,4),(5,6)):
        if la in by and lb in by:
            a,b=by[la],by[lb]
            sx,sy,sz=a[1]+b[1],a[2]+b[2],a[3]+b[3]
            p2+=sx*sx+sy*sy+sz*sz; pxy2+=sx*sx+sy*sy; np+=1
    pr=math.sqrt(p2/np)*1000
    p0,pn=s[0],s[-1]
    fd=math.sqrt(sum((pn[i]-p0[i])**2 for i in (1,2,3)))*1000
    return dict(score_mm=zr+xr+pr,z_rms_mm=zr,xy_err_rms_mm=xr,
                pair_closure_rms_mm=pr,pair_xy_closure_rms_mm=math.sqrt(pxy2/np)*1000,
                xy_mean_mm=xys/n*1000,final_dp_mm=fd)

def main():
    with open(MANIFEST,newline="") as f: M=list(csv.DictReader(f))
    cache={}; rows=[]
    for m in M:
        key=(m["backend"],m["legs"])
        if key not in cache: cache[key]=legs(m["legs"],kft(m["backend"]))
        p=os.path.join(HOME,f'jtzero_extrinsics_replay_v10_{m["tag"]}.csv')
        if not os.path.exists(p):
            print("WARN missing",p,file=sys.stderr); continue
        rows.append(dict(dataset=m["dataset"],candidate=m["candidate"],
                         roll_deg=float(m["roll_deg"]),pitch_deg=float(m["pitch_deg"]),
                         **calc(states(p),cache[key])))
    if not rows: raise SystemExit("no results")
    fields=list(rows[0].keys())
    with open(DETAIL,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader()
        for r in rows:w.writerow(r)
    g=defaultdict(list)
    for r in rows:g[r["candidate"]].append(r)
    agg=[]
    for c,rr in g.items():
        agg.append(dict(candidate=c,roll_deg=rr[0]["roll_deg"],pitch_deg=rr[0]["pitch_deg"],datasets=len(rr),
                        worst_score_mm=max(x["score_mm"] for x in rr),
                        mean_score_mm=sum(x["score_mm"] for x in rr)/len(rr),
                        worst_z_rms_mm=max(x["z_rms_mm"] for x in rr),
                        mean_z_rms_mm=sum(x["z_rms_mm"] for x in rr)/len(rr),
                        worst_xy_rms_mm=max(x["xy_err_rms_mm"] for x in rr),
                        mean_xy_rms_mm=sum(x["xy_err_rms_mm"] for x in rr)/len(rr),
                        worst_pair_mm=max(x["pair_closure_rms_mm"] for x in rr),
                        mean_pair_mm=sum(x["pair_closure_rms_mm"] for x in rr)/len(rr),
                        worst_final_dp_mm=max(x["final_dp_mm"] for x in rr),
                        mean_final_dp_mm=sum(x["final_dp_mm"] for x in rr)/len(rr)))
    agg.sort(key=lambda r:(r["worst_score_mm"],r["mean_score_mm"]))
    sf=["rank"]+list(agg[0].keys())
    with open(SUMMARY,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=sf);w.writeheader()
        for i,r in enumerate(agg,1):w.writerow({"rank":i,**r})
    print("================ TBS V11/V12/V13 CROSS-VALIDATION ================")
    print("rank candidate  roll pitch worst_score mean_score worst_Z worst_XY worst_pair worst_final")
    for i,r in enumerate(agg,1):
        print(f'{i:>2} {r["candidate"]:<9} {r["roll_deg"]:>5.1f} {r["pitch_deg"]:>5.1f} '
              f'{r["worst_score_mm"]:>11.2f} {r["mean_score_mm"]:>10.2f} '
              f'{r["worst_z_rms_mm"]:>7.2f} {r["worst_xy_rms_mm"]:>8.2f} '
              f'{r["worst_pair_mm"]:>10.2f} {r["worst_final_dp_mm"]:>11.2f}')
    print("\nPer-dataset:")
    for r in sorted(rows,key=lambda x:(x["candidate"],x["dataset"])):
        print(f'{r["dataset"]:<4} {r["candidate"]:<9} score={r["score_mm"]:7.2f} '
              f'Z={r["z_rms_mm"]:6.2f} XY={r["xy_err_rms_mm"]:6.2f} '
              f'pair={r["pair_closure_rms_mm"]:6.2f} meanXY={r["xy_mean_mm"]:7.2f} final={r["final_dp_mm"]:7.2f}')

if __name__=="__main__":
    main()
