#!/usr/bin/env python3
from pathlib import Path
import csv, math, statistics

ROOT=Path("/home/vio/jtzero_runs")
fresh=sorted(ROOT.glob("*_v34_FRESHSTART_4X"))[-1]
cont=sorted(ROOT.glob("*_v25_TBS_ROLD_7P5"))[-1]

def rows(p):
    with p.open() as f: return list(csv.DictReader(f))
def ff(x):
    try:return float(x)
    except:return float("nan")
def mean(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.mean(v) if v else float("nan")
def first_existing(d,names):
    for n in names:
        p=d/n
        if p.exists(): return p
    return None
def inspect_run(label,d,pass_no=1):
    ev=rows(d/"jtzero_500mm_v25_events.csv")
    be=rows(d/"jtzero_500mm_v25_backend.csv")
    imu=rows(d/"jtzero_500mm_v25.csv")
    att=rows(d/"jtzero_500mm_v25_attitude.csv")
    E=[e for e in ev if int(float(e.get("leg",0)))==pass_no and e.get("event")=="START"]
    if not E: raise RuntimeError(f"{label}: START not found")
    e=E[0]; ts=int(float(e["state_timestamp_ns"])); wall=int(float(e["event_wall_ns"]))
    # Backend: last state before START and first 2 s after START.
    b0=[r for r in be if int(float(r["timestamp_ns"]))<=ts]
    b2=[r for r in be if ts<=int(float(r["timestamp_ns"]))<=ts+2_000_000_000]
    # Raw IMU around first 2 s after START.
    im2=[r for r in imu if ts<=int(float(r["mapped_ns"]))<=ts+2_000_000_000]
    # FC attitude around first 2 s after START, wall-clock domain.
    at2=[r for r in att if wall<=int(float(r["recv_ns"]))<=wall+2_000_000_000]
    pre=b0[-1] if b0 else {}
    def m(rs,k): return mean([ff(r.get(k,"nan")) for r in rs])
    def span(rs,k):
        a=[ff(r.get(k,"nan")) for r in rs]; a=[x for x in a if math.isfinite(x)]
        return max(a)-min(a) if a else float("nan")
    return dict(label=label,
      pre_roll=ff(pre.get("roll_deg","nan")),pre_pitch=ff(pre.get("pitch_deg","nan")),pre_yaw=ff(pre.get("yaw_deg","nan")),
      pre_bax=ff(pre.get("bax","nan")),pre_bay=ff(pre.get("bay","nan")),pre_baz=ff(pre.get("baz","nan")),
      b_roll=m(b2,"roll_deg"),b_pitch=m(b2,"pitch_deg"),b_roll_span=span(b2,"roll_deg"),b_pitch_span=span(b2,"pitch_deg"),
      imu_ax=m(im2,"ax"),imu_ay=m(im2,"ay"),imu_az=m(im2,"az"),
      fc_roll=m(at2,"roll_deg"),fc_pitch=m(at2,"pitch_deg"),
      n_backend=len(b2),n_imu=len(im2),n_att=len(at2))

out=[inspect_run("CONT_PASS1",cont,1)]
for n in range(1,5): out.append(inspect_run(f"FRESH_{n}",fresh/f"fresh_{n}",1))

print("="*170)
print("V37 — STARTUP / FIRST-2-SECONDS INITIALIZATION SCREEN")
print("="*170)
print("STARTUP = состояние VIO непосредственно перед началом измеряемого движения.")
print("bias = оценка постоянной ошибки акселерометра, которую backend Kimera включает в состояние.")
print("RPY = roll/pitch/yaw, углы ориентации корпуса.")
print("Первые 2 s сравниваются для поиска различия runtime initialization, которое не объясняется ImuParams.yaml или T_BS.\n")
print("RUN          preRPY_deg                    preBA_mps2                    IMUmean_xyz                  backendRP_mean/span             FC_RP_mean")
for r in out:
    print(f"{r['label']:<12} [{r['pre_roll']:+7.3f},{r['pre_pitch']:+7.3f},{r['pre_yaw']:+7.3f}] "
          f"[{r['pre_bax']:+.5f},{r['pre_bay']:+.5f},{r['pre_baz']:+.5f}] "
          f"[{r['imu_ax']:+.4f},{r['imu_ay']:+.4f},{r['imu_az']:+.4f}] "
          f"R={r['b_roll']:+6.3f}/{r['b_roll_span']:.3f} P={r['b_pitch']:+6.3f}/{r['b_pitch_span']:.3f} "
          f"FC={r['fc_roll']:+6.3f},{r['fc_pitch']:+6.3f}")

c=out[0]; fs=out[1:]
print("\n"+"="*170)
print("FRESH MEAN MINUS CONT PASS1")
print("="*170)
for k,desc in [
 ("pre_roll","pre-start roll"),("pre_pitch","pre-start pitch"),("pre_bax","pre-start accel bias X"),
 ("pre_bay","pre-start accel bias Y"),("pre_baz","pre-start accel bias Z"),
 ("imu_ax","first-2s raw accel X"),("imu_ay","first-2s raw accel Y"),("imu_az","first-2s raw accel Z"),
 ("b_roll","first-2s backend roll"),("b_pitch","first-2s backend pitch"),
 ("fc_roll","first-2s FC roll"),("fc_pitch","first-2s FC pitch")]:
    fm=mean([r[k] for r in fs])
    print(f"{desc:<32} CONT={c[k]:+10.5f} FRESHmean={fm:+10.5f} delta={fm-c[k]:+10.5f}")

print("\n"+"="*170)
print("INTERPRETATION")
print("="*170)
print("1) Если raw IMU mean различается вместе с backend orientation, причина начинается до/во время initialization.")
print("2) Если FC attitude одинаков, а backend orientation различается, проблема локализуется в VIO initialization / gravity alignment.")
print("3) Если backend и FC orientation одинаковы, но bias быстро расходится, следующий кандидат — bias observability/fusion during first motion.")
print("4) Это анализ существующих архивов: новый физический прогон не требуется.")
