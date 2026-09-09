#!/usr/bin/env python3
import argparse,csv,math,statistics
from pathlib import Path

def load(p):
    with Path(p).open(newline="") as f:return list(csv.DictReader(f))
def I(x):return int(x)
def F(x):return float(x)
def med(v):return statistics.median(v) if v else float("nan")
def norm3(x,y,z):return math.sqrt(x*x+y*y+z*z)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run")
    a=ap.parse_args()
    root=Path(a.run)

    ev=load(root/"events.csv")
    be=load(root/"backend.csv")
    st=[r for r in ev if r["event"]=="MOVE_START"]
    if len(st)!=1:raise SystemExit("need exactly one MOVE_START")
    t0=I(st[0]["wall_ns"])

    pre=[r for r in be if I(r["callback_wall_ns"])<=t0]
    if not pre:raise SystemExit("no backend states before MOVE_START")

    first=pre[0]; last=pre[-1]
    warm=(I(last["callback_wall_ns"])-I(first["callback_wall_ns"]))/1e9

    print("="*118)
    print("CLEAN-01 — PRE-MOVE INITIALIZATION / BIAS CONVERGENCE")
    print("="*118)
    print(f"run={root}")
    print(f"backend states before MOVE_START={len(pre)}")
    print(f"backend warm-up from first callback to last pre-move callback={warm:.3f} s")
    print("historical harness reference requirement=12.000 s stationary + stable backend-state gate")
    print()

    def state(r):
        return {
            "px":F(r["px"]),"py":F(r["py"]),"pz":F(r["pz"]),
            "vx":F(r["vx"]),"vy":F(r["vy"]),"vz":F(r["vz"]),
            "bgx":F(r["bgx"]),"bgy":F(r["bgy"]),"bgz":F(r["bgz"]),
            "bax":F(r["bax"]),"bay":F(r["bay"]),"baz":F(r["baz"]),
        }

    A=state(first);B=state(last)
    dp=[(B[k]-A[k]) for k in ("px","py","pz")]
    dv=[(B[k]-A[k]) for k in ("vx","vy","vz")]
    dbg=[(B[k]-A[k]) for k in ("bgx","bgy","bgz")]
    dba=[(B[k]-A[k]) for k in ("bax","bay","baz")]

    print("FIRST → LAST PRE-MOVE CHANGE")
    print("-"*118)
    print(f"dP   = [{dp[0]*1000:+.3f},{dp[1]*1000:+.3f},{dp[2]*1000:+.3f}] mm  norm={norm3(*dp)*1000:.3f} mm")
    print(f"dV   = [{dv[0]*1000:+.3f},{dv[1]*1000:+.3f},{dv[2]*1000:+.3f}] mm/s norm={norm3(*dv)*1000:.3f} mm/s")
    print(f"dBg  = [{dbg[0]:+.6e},{dbg[1]:+.6e},{dbg[2]:+.6e}] rad/s norm={norm3(*dbg):.6e}")
    print(f"dBa  = [{dba[0]:+.6e},{dba[1]:+.6e},{dba[2]:+.6e}] m/s^2 norm={norm3(*dba):.6e}")
    print()

    print("1-SECOND PRE-MOVE WINDOWS")
    print("-"*118)
    start=I(first["callback_wall_ns"])
    sec=1_000_000_000
    nbin=max(1,int((t0-start)//sec)+1)
    for j in range(nbin):
        a0=start+j*sec;a1=min(t0,start+(j+1)*sec)
        rr=[r for r in pre if a0<=I(r["callback_wall_ns"])<=a1]
        if not rr:continue
        speeds=[norm3(F(r["vx"]),F(r["vy"]),F(r["vz"]))*1000 for r in rr]
        bg=[norm3(F(r["bgx"]),F(r["bgy"]),F(r["bgz"])) for r in rr]
        ba=[norm3(F(r["bax"]),F(r["bay"]),F(r["baz"])) for r in rr]
        p0=rr[0];p1=rr[-1]
        drift=norm3(F(p1["px"])-F(p0["px"]),F(p1["py"])-F(p0["py"]),F(p1["pz"])-F(p0["pz"]))*1000
        print(f"{j:2d}: t={j:4.1f}-{(a1-start)/1e9:4.1f}s n={len(rr):2d} "
              f"|V| med={med(speeds):7.3f}mm/s  pose_drift={drift:7.3f}mm  "
              f"|Bg| med={med(bg):.6e}  |Ba| med={med(ba):.6e}")

    print()
    # Compare last 2 seconds if available.
    last2=[r for r in pre if I(r["callback_wall_ns"])>=t0-2_000_000_000]
    if len(last2)>=2:
        X=state(last2[0]);Y=state(last2[-1])
        dbg2=[Y[k]-X[k] for k in ("bgx","bgy","bgz")]
        dba2=[Y[k]-X[k] for k in ("bax","bay","baz")]
        drift2=norm3(Y["px"]-X["px"],Y["py"]-X["py"],Y["pz"]-X["pz"])*1000
        print("LAST 2 s BEFORE MOVE_START")
        print("-"*118)
        print(f"pose drift={drift2:.3f} mm")
        print(f"gyro-bias change norm={norm3(*dbg2):.6e} rad/s")
        print(f"accel-bias change norm={norm3(*dba2):.6e} m/s^2")

    print()
    print("DECISION")
    print("-"*118)
    if warm < 12.0:
        print("CLEAN-01 did NOT satisfy the older 12 s stationary convergence duration before movement.")
    else:
        print("CLEAN-01 satisfied the 12 s duration; inspect stability rather than duration.")
    print("This report uses only the CLEAN-01 archive; no historical result values are read.")
    print("="*118)

if __name__=="__main__":main()
