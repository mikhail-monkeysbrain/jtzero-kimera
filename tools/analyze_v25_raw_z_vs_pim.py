#!/usr/bin/env python3
import csv, math, sys, bisect
from pathlib import Path

G=9.81
ONSET_XY_M=0.005

def load(p):
    with p.open(newline="") as f:
        return list(csv.DictReader(f))
def I(r,k): return int(float(r[k]))
def F(r,k): return float(r[k])
def mean(xs): return sum(xs)/len(xs) if xs else float("nan")
def R22_from_rp(roll_deg,pitch_deg):
    r=math.radians(roll_deg); p=math.radians(pitch_deg)
    return math.cos(p)*math.cos(r)

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_v25_raw_z_vs_pim.py RUN_DIR")

root=Path(sys.argv[1])
raw=[r for r in load(root/"jtzero_500mm_v25.csv") if r.get("type")=="IMU"]
front=load(root/"jtzero_500mm_v25_frontend.csv")
back=sorted(load(root/"jtzero_500mm_v25_backend.csv"), key=lambda r:I(r,"timestamp_ns"))
legs=load(root/"jtzero_500mm_v25_legs.csv")

need_f={"pim_valid","pim_dt_s","pim_dvz"}
if not front or not need_f.issubset(front[0].keys()):
    raise SystemExit("frontend CSV lacks PIM diagnostics")
if not raw:
    raise SystemExit("no IMU rows")

# Raw CSV logger stores both receive time and Kimera-mapped IMU time.
# Detect the mapped timestamp field defensively because older diagnostic
# revisions used slightly different header names.
raw_keys=set(raw[0].keys())
mapped_candidates=[
    "mapped_imu_ns","mapped_ns","timestamp_ns","kimera_ns",
    "mapped_timestamp_ns","rpi_mapped_ns"
]
mapped_key=next((k for k in mapped_candidates if k in raw_keys),None)
if mapped_key is None:
    # Last-resort semantic detection.
    mapped_key=next((k for k in raw_keys if k and "mapped" in k.lower() and "ns" in k.lower()),None)
if mapped_key is None:
    raise SystemExit("cannot find mapped IMU timestamp column; columns="+",".join(sorted(k for k in raw_keys if k)))

raw=sorted(raw,key=lambda r:I(r,mapped_key))
rts=[I(r,mapped_key) for r in raw]
bts=[I(r,"timestamp_ns") for r in back]
bykf={I(r,"keyframe"):r for r in back}

def prev_backend(ts):
    j=bisect.bisect_left(bts,ts)-1
    return back[j] if j>=0 else None

def nearest_backend(ts):
    j=bisect.bisect_left(bts,ts)
    cand=[]
    for k in (j-1,j,j+1):
        if 0<=k<len(back): cand.append(back[k])
    return min(cand,key=lambda r:abs(I(r,"timestamp_ns")-ts)) if cand else None

def raw_interval(t0,t1):
    i=bisect.bisect_right(rts,t0)
    j=bisect.bisect_right(rts,t1)
    return raw[i:j]

print("================ V25 RAW BODY-Z vs PIM ================")
print("run:",root)
print("raw mapped timestamp column:",mapped_key)
print("raw body-Z residual = FLU az - backend baz - expected gravity on body Z.")
print("If its sign already matches PIM residual, the direction-dependent signal is present before preintegration.")
print()

all_pairs=[]
for L in legs:
    leg=I(L,"leg")
    ks,ke=I(L,"start_settled_kf"),I(L,"end_press_kf")
    s=bykf.get(ks)
    if not s: continue

    seg=[r for r in back if ks<=I(r,"keyframe")<=ke]
    onset_kf=ks
    for r in seg:
        dx=F(r,"px_m")-F(s,"px_m"); dy=F(r,"py_m")-F(s,"py_m")
        if math.hypot(dx,dy)>=ONSET_XY_M:
            onset_kf=I(r,"keyframe"); break

    t_on=I(bykf[onset_kf],"timestamp_ns")
    t_end=I(bykf[ke],"timestamp_ns")
    fs=[r for r in front if t_on<=I(r,"timestamp_ns")<=t_end
        and I(r,"is_keyframe")==1 and I(r,"pim_valid")==1]

    rows=[]
    for f in fs:
        t1=I(f,"timestamp_ns")
        bp=prev_backend(t1); bc=nearest_backend(t1)
        if bp is None or bc is None: continue
        if abs(I(bc,"timestamp_ns")-t1)/1e6>80.0: continue
        t0=I(bp,"timestamp_ns")
        dt=F(f,"pim_dt_s")
        if dt<=0 or dt>1.0: continue
        rr=raw_interval(t0,t1)
        if len(rr)<3: continue

        # Raw HIGHRES_IMU is FRD. The actual V25 feed converts it to FLU:
        # (x, -y, -z). Therefore the body-Z specific force fed to Kimera is -az.
        baz=F(bp,"baz")
        r22=R22_from_rp(F(bp,"roll_deg"),F(bp,"pitch_deg"))
        expected_body_z=G*r22
        raw_res=[(-F(q,"az") - baz) - expected_body_z for q in rr]
        raw_mean=mean(raw_res)

        # Same native-Z residual used in the prior PIM decomposition.
        pim_native_acc=(r22*F(f,"pim_dvz") - G*dt)/dt
        # NOTE: pim_dvz is specific-force preintegration; subtracting gravity
        # here expresses the small residual acceleration after cancellation.
        # Our previous world formula used gravity_z=-G, equivalently r22*dVz-G*dt.

        rows.append((I(bc,"keyframe"),len(rr),raw_mean,pim_native_acc,baz,r22))
        all_pairs.append((L["direction"],raw_mean,pim_native_acc))

    first=rows[:min(8,len(rows))]
    print(f"LEG {leg} {L['direction']}: endpoint dz={F(L,'dz_m')*1000:+.1f}mm onsetKF={onset_kf} intervals={len(rows)}")
    for kf,n,rawm,pimr,baz,r22 in first:
        same=(rawm==0 or pimr==0 or rawm*pimr>0)
        print(f"  KF={kf:3d} n={n:3d} rawZres={rawm:+.5f} m/s^2  PIMZres={pimr:+.5f}  baz={baz:+.5f} R22={r22:.5f} sameSign={'Y' if same else 'N'}")
    if first:
        print(f"  first-{len(first)} mean rawZres={mean([x[2] for x in first]):+.5f} m/s^2")
        print(f"  first-{len(first)} mean PIMZres={mean([x[3] for x in first]):+.5f} m/s^2")
        agree=sum(1 for x in first if x[2]==0 or x[3]==0 or x[2]*x[3]>0)
        print(f"  sign agreement={agree}/{len(first)}")
    print()

print("================ DIRECTION MEANS ================")
for d in ("A->B","B->A"):
    rr=[x for x in all_pairs if x[0]==d]
    if rr:
        print(f"{d}: intervals={len(rr)} rawZres={mean([x[1] for x in rr]):+.5f} PIMZres={mean([x[2] for x in rr]):+.5f} m/s^2")

print()
print("INTERPRETATION:")
print("- rawZres already negative on A->B and positive on B->A, matching PIM => the sign is already present in IMU samples/bias entering preintegration.")
print("- rawZres near zero or unrelated, while PIMZres flips cleanly => preintegration/rotation handling creates the directional residual.")
print("- This comparison uses the backend bias at the start of each PIM interval, matching the estimator state as closely as the archived logs allow.")
