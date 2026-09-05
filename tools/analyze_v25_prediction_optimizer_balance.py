#!/usr/bin/env python3
import csv, os, math

CHAIN=os.path.expanduser("~/jtzero_kimera_chain.csv")
EVENTS=os.path.expanduser("~/jtzero_500mm_v25_events.csv")

def read(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(r,k): return float(r[k])
def I(r,k): return int(r[k])
def P(r,pfx): return (F(r,pfx+"_px"),F(r,pfx+"_py"),F(r,pfx+"_pz"))
def V(r,pfx): return (F(r,pfx+"_vx"),F(r,pfx+"_vy"),F(r,pfx+"_vz"))
def add(a,b): return tuple(x+y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mul(a,s): return tuple(x*s for x in a)
def norm(a): return math.sqrt(sum(x*x for x in a))
def fmt(a): return "["+",".join(f"{x*1000:+.1f}" for x in a)+"]mm"

chain=read(CHAIN); events=read(EVENTS)
rows=sorted(chain,key=lambda r:I(r,"keyframe"))
bykf={I(r,"keyframe"):r for r in rows}
maxkf=max(bykf)

print("================ V25 PREDICTION vs OPTIMIZER CORRECTION ================")
print("Identity checked per KF: STATE_k - STATE_{k-1} = (PRED_k - STATE_{k-1}) + (STATE_k - PRED_k)")
print("So: net state motion = IMU/preintegration prediction step + optimizer correction.")

for e in [x for x in events if x["event"]=="END"]:
    leg=int(e["leg"]); k0=I(e,"keyframe")
    kend=min(maxkf,k0+100)
    total_pred=(0.0,0.0,0.0)
    total_opt=(0.0,0.0,0.0)
    total_net=(0.0,0.0,0.0)
    total_velcorr=(0.0,0.0,0.0)
    print(f"\nLEG {leg} END KF={k0} window={k0+1}..{kend}")
    print(" KF  dt[ms]  predStep[mm]              optCorr[mm]               netState[mm]              |pred| |opt| |net|")

    samples=[]
    prev=bykf.get(k0)
    if not prev: continue
    for k in range(k0+1,kend+1):
        cur=bykf.get(k)
        if not cur: continue
        dt=(F(cur,"timestamp_ns")-F(prev,"timestamp_ns"))*1e-6
        prev_state=P(prev,"state")
        pred=P(cur,"pred")
        state=P(cur,"state")
        pred_step=sub(pred,prev_state)
        opt_corr=sub(state,pred)
        net_step=sub(state,prev_state)
        vel_corr=sub(V(cur,"state"),V(cur,"pred"))

        total_pred=add(total_pred,pred_step)
        total_opt=add(total_opt,opt_corr)
        total_net=add(total_net,net_step)
        total_velcorr=add(total_velcorr,vel_corr)
        samples.append((k,dt,pred_step,opt_corr,net_step,vel_corr))

        prev=cur

    # Print early, large correction, and tail samples.
    chosen=set()
    for s in samples[:8]: chosen.add(s[0])
    for s in sorted(samples,key=lambda x:norm(x[3]),reverse=True)[:8]: chosen.add(s[0])
    for s in samples[-5:]: chosen.add(s[0])
    for k,dt,ps,oc,ns,vc in samples:
        if k not in chosen: continue
        print(f"{k:3d} {dt:7.1f}  {fmt(ps):24s} {fmt(oc):24s} {fmt(ns):24s}"
              f" {norm(ps)*1000:6.1f} {norm(oc)*1000:5.1f} {norm(ns)*1000:5.1f}")

    print("  accumulated prediction =",fmt(total_pred),f"|.|={norm(total_pred)*1000:.1f}mm")
    print("  accumulated optimizer  =",fmt(total_opt),f"|.|={norm(total_opt)*1000:.1f}mm")
    print("  accumulated STATE net  =",fmt(total_net),f"|.|={norm(total_net)*1000:.1f}mm")
    closure=add(total_pred,total_opt)
    err=norm(sub(closure,total_net))*1000
    print("  algebra check pred+opt =",fmt(closure),f"error={err:.6f}mm")

    if samples:
        mean_pred=sum(norm(x[2]) for x in samples)/len(samples)*1000
        mean_opt=sum(norm(x[3]) for x in samples)/len(samples)*1000
        mean_net=sum(norm(x[4]) for x in samples)/len(samples)*1000
        mean_vcorr=sum(norm(x[5]) for x in samples)/len(samples)*1000
        print(f"  mean per-KF: |predStep|={mean_pred:.2f}mm |optCorr|={mean_opt:.2f}mm |netState|={mean_net:.2f}mm |stateV-predV|={mean_vcorr:.2f}mm/s")

print("\n================ INTERPRETATION ================")
print("If accumulated prediction is large but optimizer correction is nearly opposite, residual velocity exists in the prediction, while the smoother is actively canceling most of its positional effect.")
print("If optimizer correction is small and STATE net follows prediction, the residual velocity directly drives pose drift.")
