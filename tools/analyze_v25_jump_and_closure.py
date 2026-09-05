#!/usr/bin/env python3
import csv, glob, math, os, sys, bisect, statistics

def load(p):
    with open(p,newline="") as f: return list(csv.DictReader(f))
def F(x): return float(x)
def I(x): return int(float(x))
def n3(x,y,z): return math.sqrt(x*x+y*y+z*z)

roots=sys.argv[1:] or sorted(glob.glob(os.path.expanduser("~/jtzero_runs/*_v25_*")))
if not roots:
    raise SystemExit("No archived V25 runs found")

for root in roots:
    bp=os.path.join(root,"jtzero_500mm_v25_backend.csv")
    fp=os.path.join(root,"jtzero_500mm_v25_frontend.csv")
    lp=os.path.join(root,"jtzero_500mm_v25_legs.csv")
    ap=os.path.join(root,"jtzero_500mm_v25_attitude.csv")
    cp=os.path.join(root,"jtzero_500mm_v25_camera.csv")
    ip=os.path.join(root,"jtzero_500mm_v25.csv")
    if not all(os.path.exists(p) for p in (bp,fp,lp,ap,cp,ip)):
        print("SKIP",root,"missing required files")
        continue

    backend=load(bp); frontend=load(fp); legs=load(lp); att=load(ap); cam=load(cp); imu=load(ip)
    bykf={I(r["keyframe"]):r for r in backend}
    f_by_ts={I(r["timestamp_ns"]):r for r in frontend}
    f_ts=sorted(f_by_ts)
    att.sort(key=lambda r:I(r["recv_ns"]))
    cam.sort(key=lambda r:I(r["timestamp_ns"]))
    imu.sort(key=lambda r:I(r["mapped_ns"]) if "mapped_ns" in r and r["mapped_ns"] else I(r["recv_ns"]))

    def nearest_front(ts):
        j=bisect.bisect_left(f_ts,ts)
        cand=[]
        if j<len(f_ts): cand.append(f_by_ts[f_ts[j]])
        if j>0: cand.append(f_by_ts[f_ts[j-1]])
        return min(cand,key=lambda r:abs(I(r["timestamp_ns"])-ts)) if cand else None

    print(f"================ {os.path.basename(root)} ================")

    # Detect largest backend jumps and show top 5.
    jumps=[]
    for a,b in zip(backend[:-1],backend[1:]):
        dp=n3(F(b["px_m"])-F(a["px_m"]),F(b["py_m"])-F(a["py_m"]),F(b["pz_m"])-F(a["pz_m"]))*1000
        dt=(I(b["timestamp_ns"])-I(a["timestamp_ns"]))/1e6
        jumps.append((dp,dt,I(b["keyframe"]),a,b))
    jumps.sort(reverse=True,key=lambda x:x[0])
    print("\nTOP BACKEND JUMPS")
    for dp,dt,kf,a,b in jumps[:5]:
        print(f" KF={kf} dP={dp:.2f}mm dt={dt:.2f}ms "
              f"Vprev={n3(F(a['vx_m_s']),F(a['vy_m_s']),F(a['vz_m_s']))*1000:.1f} "
              f"Vnow={n3(F(b['vx_m_s']),F(b['vy_m_s']),F(b['vz_m_s']))*1000:.1f}mm/s")

    targets=[315]
    # Also inspect A-return settled KFs from cycles.
    if len(legs)>=4:
        targets += [I(legs[1]["end_settled_kf"]), I(legs[3]["end_settled_kf"])]
    targets=sorted(set(targets))

    for tk in targets:
        if tk not in bykf:
            print(f"\nTARGET KF {tk}: not present")
            continue
        print(f"\n================ WINDOW KF {tk-6}..{tk+6} ================")
        seg=[r for r in backend if tk-6<=I(r["keyframe"])<=tk+6]
        for r in seg:
            kf=I(r["keyframe"]); ts=I(r["timestamp_ns"])
            prev=bykf.get(kf-1)
            dp=0.0
            if prev:
                dp=n3(F(r["px_m"])-F(prev["px_m"]),F(r["py_m"])-F(prev["py_m"]),F(r["pz_m"])-F(prev["pz_m"]))*1000
            fr=nearest_front(ts)
            if fr:
                ratio=F(fr["mono_inlier_ratio"])
                ftxt=(f"front frame={I(fr['frame_id'])} kf={I(fr['is_keyframe'])} "
                      f"trk={I(fr['tracked_features'])} inl={I(fr['mono_inliers'])}/{I(fr['mono_putatives'])} "
                      f"ratio={ratio:.3f} status={fr['mono_status']} "
                      f"dtF={(I(fr['timestamp_ns'])-ts)/1e6:+.1f}ms")
            else:
                ftxt="front=NONE"
            print(
                f"KF={kf} dP={dp:6.1f}mm "
                f"P=[{F(r['px_m']):+.4f},{F(r['py_m']):+.4f},{F(r['pz_m']):+.4f}] "
                f"V=[{F(r['vx_m_s']):+.4f},{F(r['vy_m_s']):+.4f},{F(r['vz_m_s']):+.4f}] "
                f"RPY=[{F(r['roll_deg']):+.3f},{F(r['pitch_deg']):+.3f},{F(r['yaw_deg']):+.3f}] "
                f"BA=[{F(r['bax']):+.4f},{F(r['bay']):+.4f},{F(r['baz']):+.4f}] "
                f"BG=[{F(r['bgx']):+.5f},{F(r['bgy']):+.5f},{F(r['bgz']):+.5f}] | {ftxt}"
            )

        # Time continuity around target.
        tr=bykf[tk]; t0=I(tr["timestamp_ns"])
        print("\nTIMING AROUND TARGET")
        cwin=[r for r in cam if abs(I(r["timestamp_ns"])-t0)<=1500000000]
        if cwin:
            gaps=[(I(b["timestamp_ns"])-I(a["timestamp_ns"]))/1e6 for a,b in zip(cwin[:-1],cwin[1:])]
            seqj=[]
            if "sequence" in cwin[0]:
                for a,b in zip(cwin[:-1],cwin[1:]):
                    try: seqj.append(I(b["sequence"])-I(a["sequence"]))
                    except: pass
            print(f" camera rows={len(cwin)} maxGap={max(gaps) if gaps else 0:.2f}ms "
                  f"maxSeqJump={max(seqj) if seqj else 0}")
        iwin=[]
        ikey="mapped_ns" if imu and "mapped_ns" in imu[0] else "recv_ns"
        for r in imu:
            try: ts=I(r[ikey])
            except: continue
            if abs(ts-t0)<=1500000000: iwin.append(r)
        if iwin:
            gaps=[(I(b[ikey])-I(a[ikey]))/1e6 for a,b in zip(iwin[:-1],iwin[1:])]
            print(f" imu rows={len(iwin)} maxGap={max(gaps) if gaps else 0:.2f}ms")
        fwin=[r for r in frontend if abs(I(r["timestamp_ns"])-t0)<=1500000000 and I(r["is_keyframe"])==1]
        if fwin:
            ratios=[F(r["mono_inlier_ratio"]) for r in fwin]
            print(f" frontend keyframes={len(fwin)} ratioMean={statistics.mean(ratios):.3f} "
                  f"ratioMin={min(ratios):.3f} statuses={{{', '.join(sorted(set(r['mono_status'] for r in fwin)))}}}")

    print()
