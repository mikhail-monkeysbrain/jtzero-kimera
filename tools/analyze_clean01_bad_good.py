#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

def load(p):
    with p.open(newline="") as f: return list(csv.DictReader(f))
def I(x): return int(x)
def F(x): return float(x)
def med(v): return statistics.median(v) if v else float("nan")
def norm3(x,y,z): return math.sqrt(x*x+y*y+z*z)

def event(rows,name):
    x=[r for r in rows if r["event"]==name]
    if len(x)!=1: raise RuntimeError(f"expected exactly one {name}")
    return x[0]

def summarize(root,bins):
    ev=load(root/"events.csv"); be=load(root/"backend.csv"); fr=load(root/"frontend.csv")
    a=event(ev,"MOVE_START"); b=event(ev,"MOVE_END")
    t0=I(a["wall_ns"]); t1=I(b["wall_ns"]); dur=t1-t0
    pre=[r for r in be if I(r["callback_wall_ns"])<=t0]
    move_be=[r for r in be if t0<=I(r["callback_wall_ns"])<=t1]
    move_fr=[r for r in fr if t0<=I(r["callback_wall_ns"])<=t1]
    if not pre or not move_be: raise RuntimeError(f"{root}: missing backend data")

    last=pre[-1]
    startp=(F(a["px"]),F(a["py"]),F(a["pz"]))
    endp=(F(b["px"]),F(b["py"]),F(b["pz"]))
    endpoint=math.hypot(endp[0]-startp[0],endp[1]-startp[1])*1000

    out={"name":root.name,"dur_s":dur/1e9,"endpoint":endpoint,
         "pre_n":len(pre),
         "warm_s":(I(pre[-1]["callback_wall_ns"])-I(pre[0]["callback_wall_ns"]))/1e9,
         "v0":norm3(F(last["vx"]),F(last["vy"]),F(last["vz"]))*1000,
         "bg":norm3(F(last["bgx"]),F(last["bgy"]),F(last["bgz"])),
         "ba":norm3(F(last["bax"]),F(last["bay"]),F(last["baz"])),
         "bins":[]}

    for j in range(bins):
        lo=t0+dur*j//bins; hi=t0+dur*(j+1)//bins
        br=[r for r in move_be if lo<=I(r["callback_wall_ns"])<=hi]
        rr=[r for r in move_fr if lo<=I(r["callback_wall_ns"])<=hi]
        # cumulative backend horizontal displacement from captured A to last state in bin
        if br:
            q=br[-1]
            cum=math.hypot(F(q["px"])-startp[0],F(q["py"])-startp[1])*1000
        else: cum=float("nan")
        kf=[r for r in rr if r.get("is_keyframe","0")=="1"]
        valid=[r for r in kf if r.get("mono_status","")=="VALID"]
        low=[r for r in kf if r.get("mono_status","")=="LOW_DISPARITY"]
        invalid=[r for r in kf if r.get("mono_status","")=="INVALID"]
        tracked=[I(r["tracked"]) for r in rr]
        inliers=[I(r["inliers"]) for r in rr]
        put=[I(r["putatives"]) for r in rr]
        ratios=[I(r["inliers"])/I(r["putatives"]) for r in rr if I(r["putatives"])>0]
        out["bins"].append({
            "cum":cum,"front":len(rr),"kf":len(kf),"valid":len(valid),"low":len(low),"invalid":len(invalid),
            "trk":med(tracked),"inl":med(inliers),"ratio":med(ratios)})
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bad")
    ap.add_argument("good",nargs="+")
    ap.add_argument("--bins",type=int,default=10)
    a=ap.parse_args()
    bad=summarize(Path(a.bad),a.bins)
    goods=[summarize(Path(x),a.bins) for x in a.good]

    print("="*150)
    print("CLEAN-01 — BAD vs GOOD SAME-PROTOCOL STATE COMPARISON")
    print("="*150)
    print("Only the CLEAN-01 archives passed on the command line are read.")
    print()
    print("PRE-MOVE FINAL STATE")
    print("-"*150)
    print(f"{'RUN':42s} {'warm s':>8s} {'|V| mm/s':>10s} {'|Bg| rad/s':>13s} {'|Ba| m/s2':>13s} {'endpoint mm':>12s}")
    for r in [bad]+goods:
        print(f"{r['name']:42s} {r['warm_s']:8.3f} {r['v0']:10.3f} {r['bg']:13.6e} {r['ba']:13.6e} {r['endpoint']:12.2f}")

    print()
    print("MOVE BINS — normalized wall-time; cumulative H is descriptive only because hand-motion speed is not controlled.")
    print("-"*150)
    print(f"{'bin':>4s} {'BAD H':>9s} {'GOOD H med':>11s} {'BAD KF V/L/I':>15s} {'GOOD KF V/L/I med':>21s} {'BAD ratio':>10s} {'GOOD ratio':>11s}")
    for j in range(a.bins):
        bb=bad["bins"][j]
        gc=[g["bins"][j]["cum"] for g in goods if math.isfinite(g["bins"][j]["cum"])]
        gh=med(gc)
        gv=med([g["bins"][j]["valid"] for g in goods])
        gl=med([g["bins"][j]["low"] for g in goods])
        gi=med([g["bins"][j]["invalid"] for g in goods])
        gr=med([g["bins"][j]["ratio"] for g in goods if math.isfinite(g["bins"][j]["ratio"])])
        print(f"{j+1:4d} {bb['cum']:9.2f} {gh:11.2f} "
              f"{bb['valid']:3d}/{bb['low']:3d}/{bb['invalid']:3d} "
              f"{gv:5.1f}/{gl:5.1f}/{gi:5.1f} "
              f"{bb['ratio']:10.3f} {gr:11.3f}")

    print()
    print("IMPORTANT")
    print("-"*150)
    print("Do not infer metric scale from normalized-time H differences: the physical hand-motion profile differs between runs.")
    print("Use this report to find an early discrete state difference: initialization/bias, keyframe status, or frontend quality.")
    print("If BAD and GOOD are already different before MOVE_START, investigate initialization first.")
    print("If pre-move state is comparable but keyframe/status structure separates immediately during MOVE, investigate frontend/backend mode selection.")
    print("="*150)

if __name__=="__main__": main()
