#!/usr/bin/env python3
import argparse,csv,math,statistics,bisect
from pathlib import Path

def loadcsv(p):
    if not p.exists(): return []
    with p.open(newline="") as f: return list(csv.DictReader(f))
def F(r,k,default=float("nan")):
    try:return float(r.get(k,default))
    except:return default
def I(r,k,default=0):
    try:return int(float(r.get(k,default)))
    except:return default
def mean(v): return statistics.mean(v) if v else float("nan")
def rms(v): return math.sqrt(mean([x*x for x in v])) if v else float("nan")
def pct(v,p):
    if not v:return float("nan")
    s=sorted(v); x=(len(s)-1)*p; i=int(math.floor(x)); j=int(math.ceil(x))
    return s[i] if i==j else s[i]*(j-x)+s[j]*(x-i)
def span(v): return max(v)-min(v) if v else float("nan")

def get_leg(run):
    legs=loadcsv(run/"jtzero_500mm_v25_legs.csv")
    if not legs:return {}
    L=legs[0]
    return {
      "horizontal_mm":F(L,"horizontal_m")*1000,
      "scale":F(L,"scale_horizontal"),
      "dz_mm":F(L,"dz_m")*1000,
      "start_kf":I(L,"start_settled_kf"),
      "end_kf":I(L,"end_press_kf"),
    }

def event_window(run):
    ev=loadcsv(run/"jtzero_500mm_v25_events.csv")
    starts=[r for r in ev if r.get("event")=="START"]
    ends=[r for r in ev if r.get("event")=="END"]
    if not starts or not ends:return None
    s,e=starts[0],ends[0]
    return I(s,"event_wall_ns"),I(e,"event_wall_ns")

def frontend(run):
    leg=get_leg(run); back=loadcsv(run/"jtzero_500mm_v25_backend.csv"); fr=loadcsv(run/"jtzero_500mm_v25_frontend.csv")
    if not leg or not back or not fr:return {}
    bykf={I(r,"keyframe"):r for r in back}
    s=bykf.get(leg["start_kf"]); e=bykf.get(leg["end_kf"])
    if not s or not e:return {}
    t0,t1=I(s,"timestamp_ns"),I(e,"timestamp_ns")
    q=[r for r in fr if t0<=I(r,"timestamp_ns")<=t1 and I(r,"is_keyframe")==1]
    valid=[r for r in q if r.get("mono_status")=="VALID"]
    ratios=[F(r,"mono_inlier_ratio") for r in valid]
    tracks=[F(r,"tracked_features") for r in valid]
    mono_tx=[F(r,"mono_body_tx") for r in valid if I(r,"mono_pose_valid")==1]
    mono_ty=[F(r,"mono_body_ty") for r in valid if I(r,"mono_pose_valid")==1]
    mono_tz=[F(r,"mono_body_tz") for r in valid if I(r,"mono_pose_valid")==1]
    return {
      "kf":len(q),"valid_frac":len(valid)/len(q) if q else float("nan"),
      "inlier_mean":mean(ratios),"tracked_mean":mean(tracks),
      "weak_frac":sum(x<0.40 for x in ratios)/len(ratios) if ratios else float("nan"),
      "strong_frac":sum(x>=0.70 for x in ratios)/len(ratios) if ratios else float("nan"),
      "mono_dir_samples":len(mono_tx),
      "mono_tilt_mean_deg":mean([math.degrees(math.atan2(abs(z),math.hypot(x,y))) for x,y,z in zip(mono_tx,mono_ty,mono_tz)]) if mono_tx else float("nan")
    }

def forensic(run):
    rows=loadcsv(run/"jtzero_v43_camera_forensic.csv")
    if not rows:return {}
    tx=sum(F(r,"tx_px",0) for r in rows); ty=sum(F(r,"ty_px",0) for r in rows)
    mfx=sum(F(r,"med_flow_x_px",0) for r in rows); mfy=sum(F(r,"med_flow_y_px",0) for r in rows)
    inl=[F(r,"n_inliers")/max(1,F(r,"n_tracks")) for r in rows]
    hs=[F(r,"height_m") for r in rows]
    return {
      "rows":len(rows),"aff_px":math.hypot(tx,ty),"med_px":math.hypot(mfx,mfy),
      "med_aff_ratio":math.hypot(mfx,mfy)/math.hypot(tx,ty) if math.hypot(tx,ty)>0 else float("nan"),
      "track_inlier_frac":mean(inl),"height_mean_mm":mean(hs)*1000,
      "final_cam_net_mm":math.hypot(F(rows[-1],"net_x_m"),F(rows[-1],"net_y_m"))*1000
    }

def raw_motion(run):
    w=event_window(run); imu=loadcsv(run/"jtzero_500mm_v25.csv"); att=loadcsv(run/"jtzero_500mm_v25_attitude.csv")
    if not w or not imu or not att:return {}
    t0,t1=w
    seg=[r for r in imu if r.get("type")=="IMU" and t0<=I(r,"recv_ns")<=t1]
    attseg=[r for r in att if t0<=I(r,"recv_ns")<=t1]
    ax=[];ay=[];az=[];gn=[]; roll=[];pitch=[];yaw=[]
    for r in seg:
      x=F(r,"ax");y=F(r,"ay");z=F(r,"az")
      if all(math.isfinite(v) for v in (x,y,z)):
        ax.append(x);ay.append(y);az.append(z)
      gx=F(r,"gx",F(r,"xgyro")); gy=F(r,"gy",F(r,"ygyro")); gz=F(r,"gz",F(r,"zgyro"))
      if all(math.isfinite(v) for v in (gx,gy,gz)): gn.append(math.sqrt(gx*gx+gy*gy+gz*gz))
    for r in attseg:
      roll.append(F(r,"roll_deg"));pitch.append(F(r,"pitch_deg"));yaw.append(F(r,"yaw_deg"))
    # raw horizontal sensor-frame acceleration is enough for relative cross-run excitation screen.
    horiz=[math.hypot(x,y) for x,y in zip(ax,ay)]
    return {
      "duration_s":(t1-t0)/1e9,"imu_n":len(seg),
      "hacc_rms":rms(horiz),"hacc_p90":pct(horiz,.9),"gyro_rms":rms(gn),
      "roll_span":span(roll),"pitch_span":span(pitch),"yaw_span":span(yaw)
    }

def backend(run):
    leg=get_leg(run); back=loadcsv(run/"jtzero_500mm_v25_backend.csv")
    if not leg or not back:return {}
    bykf={I(r,"keyframe"):r for r in back}
    s=bykf.get(leg["start_kf"]); e=bykf.get(leg["end_kf"])
    if not s or not e:return {}
    return {
      "dx_mm":(F(e,"px_m")-F(s,"px_m"))*1000,
      "dy_mm":(F(e,"py_m")-F(s,"py_m"))*1000,
      "dz_mm":(F(e,"pz_m")-F(s,"pz_m"))*1000,
      "end_pitch":F(e,"pitch_deg"),"end_roll":F(e,"roll_deg"),"end_yaw":F(e,"yaw_deg")
    }

def summarize(run):
    return {"leg":get_leg(run),"front":frontend(run),"forensic":forensic(run),"raw":raw_motion(run),"back":backend(run)}

def fmt(v,kind=""):
    if not math.isfinite(v): return "nan"
    if kind=="pct": return f"{v*100:.1f}%"
    return f"{v:.3f}"

def delta(a,b):
    if not (math.isfinite(a) and math.isfinite(b)):return float("nan")
    return b-a

def main():
    ap=argparse.ArgumentParser(description="V44.11 same-data cross-run regression discriminator")
    ap.add_argument("--reference",required=True)
    ap.add_argument("--current",required=True)
    a=ap.parse_args()
    ref=Path(a.reference); cur=Path(a.current)
    A=summarize(ref); B=summarize(cur)

    print("="*124)
    print("V44.11 — KIMERA REGRESSION CROSS-RUN FORENSIC (NO NEW PHYSICAL RUN)")
    print("="*124)
    print(f"REFERENCE: {ref}")
    print(f"CURRENT:   {cur}")
    print()

    def row(name,sect,key,scale=1.0,suffix=""):
      av=A[sect].get(key,float("nan"))*scale; bv=B[sect].get(key,float("nan"))*scale
      print(f"{name:38s} ref={av:10.3f}{suffix} current={bv:10.3f}{suffix} delta={delta(av,bv):+10.3f}{suffix}")

    print("PRIMARY OUTPUT")
    print("-"*124)
    row("Kimera horizontal","leg","horizontal_mm",suffix="mm")
    row("Kimera scale","leg","scale")
    row("Kimera dz","leg","dz_mm",suffix="mm")
    print()

    print("CAMERA-ONLY / TRACKING")
    print("-"*124)
    row("forensic rows","forensic","rows")
    row("affine summed pixel norm","forensic","aff_px",suffix="px")
    row("median-flow summed pixel norm","forensic","med_px",suffix="px")
    row("median/affine pixel ratio","forensic","med_aff_ratio")
    row("track inlier fraction","forensic","track_inlier_frac")
    row("logged camera net","forensic","final_cam_net_mm",suffix="mm")
    row("frontend VALID fraction","front","valid_frac")
    row("frontend mean inlier","front","inlier_mean")
    row("frontend mean tracked","front","tracked_mean")
    row("frontend weak fraction","front","weak_frac")
    row("frontend strong fraction","front","strong_frac")
    row("mono translation tilt","front","mono_tilt_mean_deg",suffix="deg")
    print()

    print("RAW MOTION / ATTITUDE")
    print("-"*124)
    row("physical duration","raw","duration_s",suffix="s")
    row("raw horiz accel RMS","raw","hacc_rms",suffix="m/s2")
    row("raw horiz accel p90","raw","hacc_p90",suffix="m/s2")
    row("raw gyro norm RMS","raw","gyro_rms",suffix="rad/s")
    row("FC roll span","raw","roll_span",suffix="deg")
    row("FC pitch span","raw","pitch_span",suffix="deg")
    row("FC yaw span","raw","yaw_span",suffix="deg")
    print()

    print("BACKEND ENDPOINT")
    print("-"*124)
    row("backend dx","back","dx_mm",suffix="mm")
    row("backend dy","back","dy_mm",suffix="mm")
    row("backend dz","back","dz_mm",suffix="mm")
    row("backend end pitch","back","end_pitch",suffix="deg")
    print()

    # Decision heuristic intentionally diagnostic, not causal proof.
    scale_drop=B["leg"].get("scale",float("nan"))-A["leg"].get("scale",float("nan"))
    cam_ratio=(B["forensic"].get("med_px",float("nan"))/A["forensic"].get("med_px",float("nan"))) if A["forensic"].get("med_px",0)>0 else float("nan")
    valid_drop=B["front"].get("valid_frac",float("nan"))-A["front"].get("valid_frac",float("nan"))
    inlier_drop=B["front"].get("inlier_mean",float("nan"))-A["front"].get("inlier_mean",float("nan"))
    dyn_ratio=(B["raw"].get("hacc_rms",float("nan"))/A["raw"].get("hacc_rms",float("nan"))) if A["raw"].get("hacc_rms",0)>0 else float("nan")
    print("DISCRIMINATOR")
    print("-"*124)
    print(f"Kimera scale delta={scale_drop:+.4f}")
    print(f"camera pixel-motion current/reference={cam_ratio:.4f}")
    print(f"frontend VALID delta={valid_drop:+.4f}; inlier delta={inlier_drop:+.4f}")
    print(f"raw horizontal-accel RMS current/reference={dyn_ratio:.4f}")
    print()
    print("Interpretation priority:")
    print("1) Kimera worsens while camera pixel motion stays similar -> regression is downstream of basic image displacement scale.")
    print("2) Frontend VALID/inlier also collapses -> visual measurement quality is a plausible contributor.")
    print("3) Raw motion/attitude excitation changes strongly -> operator/physical motion profile is a confounder.")
    print("4) Camera and frontend stay comparable but Kimera scale changes strongly -> backend/IMU fusion becomes primary branch.")
    print("="*124)

if __name__=="__main__":
    main()
