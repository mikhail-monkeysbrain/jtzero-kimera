#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
KIMERA="/home/vio/Kimera-VIO"
SRC="${KIMERA}/src/backend/VioBackend.cpp"
BAK="/tmp/VioBackend.cpp.v15_51.bak"
PARAMS="${ROOT}/params/JTZeroMonoFLU"
IMU="/home/vio/jtzero_yaw_only_v15_42.csv"
CAM="/home/vio/jtzero_yaw_only_v15_42_camera.csv"
MJPG="/home/vio/jtzero_yaw_only_v15_42.mjpg"
TMP="/tmp/jtzero_v15_51"
BIN="${TMP}/replay_v15_51"
OUT="/home/vio/jtzero_visual_lifecycle_v15_51"
THRESH="0.25"

cd "$ROOT"
for f in "$SRC" "$IMU" "$CAM" "$MJPG" tools/replay_mono_imu_zxy_ab_v11.cpp; do
  [[ -s "$f" ]] || { echo "[FATAL] missing: $f"; exit 1; }
done
mkdir -p "$TMP" "$OUT"
rm -f "$OUT"/*.log "$OUT"/report.tsv
cp "$SRC" "$BAK"

restore() {
  if [[ -s "$BAK" ]]; then
    cp "$BAK" "$SRC"
    echo "[RESTORE] Restored clean VioBackend.cpp"
    cmake --build "$KIMERA/build" -j"$(nproc)" >/tmp/jtzero_v15_51_restore_build.log 2>&1 || \
      echo "[WARN] clean rebuild after restore failed; see /tmp/jtzero_v15_51_restore_build.log"
  fi
}
trap restore EXIT

echo "============================================================"
echo "JT-ZERO v15.51 VISUAL LIFECYCLE CAUSAL MATRIX"
echo "Dataset: v15.42"
echo "One diagnostic Kimera rebuild, many runtime A/B hypotheses."
echo "Rotation trigger: IMU keyframe delta >= ${THRESH} deg."
echo "No LOW_DISPARITY status and no zero-velocity/no-motion priors are injected."
echo "Production source is restored automatically at exit."
echo "============================================================"

python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); s=p.read_text()
if '#include <cstdlib>' not in s:
    s=s.replace('#include <limits>', '#include <cstdlib>\n#include <limits>', 1)
if '#include <string>' not in s:
    raise SystemExit('[FATAL] expected <string> include missing')

anchor='namespace VIO {\n'
globals=r'''namespace VIO {

// JT-ZERO v15.51 diagnostic-only runtime state.
// Each replay is a separate process, so these are reset between matrix rows.
static bool jtzero_v1551_rot_active = false;
static bool jtzero_v1551_rot_seen = false;

static std::string jtzeroV1551Mode() {
  const char* p = std::getenv("JTZERO_V1551_MODE");
  return p ? std::string(p) : std::string("CONTROL");
}
static double jtzeroV1551ThresholdDeg() {
  const char* p = std::getenv("JTZERO_V1551_ROT_THRESH_DEG");
  return p ? std::atof(p) : 0.25;
}
static int jtzeroV1551CapObs() {
  const char* p = std::getenv("JTZERO_V1551_CAP_OBS");
  return p ? std::atoi(p) : 0;
}
'''
if anchor not in s: raise SystemExit('[FATAL] namespace anchor not found')
s=s.replace(anchor,globals,1)

# Set guard state from the same preintegrated IMU rotation used by backend.
needle='''  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;
'''
insert='''  last_kf_id_ = curr_kf_id_;
  ++curr_kf_id_;

  // JT-ZERO v15.51: classify this keyframe by IMU rotation only.
  const double jtzero_v1551_rot_deg =
      gtsam::Rot3::Logmap(pim.deltaRij()).norm() * 180.0 / M_PI;
  jtzero_v1551_rot_active =
      jtzero_v1551_rot_deg >= jtzeroV1551ThresholdDeg();
  if (jtzero_v1551_rot_active) jtzero_v1551_rot_seen = true;
'''
if needle not in s: raise SystemExit('[FATAL] state anchor not found')
s=s.replace(needle,insert,1)

# Replace immutable measurement reference with a diagnostic working copy and
# apply lifecycle hypotheses before feature_tracks_ are touched.
needle='''  const StereoMeasurements& smart_stereo_measurements_kf =
      status_smart_stereo_measurements_kf.second;
'''
insert=r'''  StereoMeasurements smart_stereo_measurements_kf =
      status_smart_stereo_measurements_kf.second;

  // JT-ZERO v15.51 broad causal matrix. This only changes which visual
  // observations reach feature_tracks_/SmartStereoFactor. It does not alter
  // tracking status and therefore does not inject no-motion priors.
  const std::string jtzero_mode = jtzeroV1551Mode();
  if (jtzero_mode == "NO_VIS_ALL") {
    smart_stereo_measurements_kf.clear();
  } else if (jtzero_mode == "NO_VIS_DURING_ROT" && jtzero_v1551_rot_active) {
    smart_stereo_measurements_kf.clear();
  } else if (jtzero_mode == "NO_VIS_AFTER_TRIGGER" && jtzero_v1551_rot_seen) {
    smart_stereo_measurements_kf.clear();
  } else if (jtzero_mode == "NO_VIS_BEFORE_TRIGGER" && !jtzero_v1551_rot_seen) {
    smart_stereo_measurements_kf.clear();
  } else if ((jtzero_mode == "ROT_NEW_ONLY" ||
              jtzero_mode == "ROT_EXISTING_ONLY") &&
             jtzero_v1551_rot_active) {
    StereoMeasurements filtered;
    filtered.reserve(smart_stereo_measurements_kf.size());
    for (const auto& m : smart_stereo_measurements_kf) {
      const bool exists = feature_tracks_.find(m.first) != feature_tracks_.end();
      if ((jtzero_mode == "ROT_NEW_ONLY" && !exists) ||
          (jtzero_mode == "ROT_EXISTING_ONLY" && exists)) {
        filtered.push_back(m);
      }
    }
    smart_stereo_measurements_kf.swap(filtered);
  }
'''
if needle not in s: raise SystemExit('[FATAL] measurement anchor not found')
s=s.replace(needle,insert,1)

# Rebuild an updated SmartStereoFactor from only the most recent N observations.
# This directly tests whether long factor memory crossing the weak-geometry
# interval is causal. Slot bookkeeping remains unchanged.
needle='''    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      LOG_IF(FATAL, obs_kf.first != static_cast<FrameId>(curr_kf_id_))
          << "addLandmarksToGraph: last obs is not from the current "
             "keyframe!\\n";

      updateLandmarkInGraph(lmk_id, obs_kf);
      ++n_updated_landmarks;
    }
'''
insert=r'''    } else {
      const std::pair<FrameId, StereoPoint2> obs_kf = ft.obs_.back();

      LOG_IF(FATAL, obs_kf.first != static_cast<FrameId>(curr_kf_id_))
          << "addLandmarksToGraph: last obs is not from the current "
             "keyframe!\n";

      const std::string jtzero_mode = jtzeroV1551Mode();
      const int jtzero_cap = jtzeroV1551CapObs();
      const bool jtzero_cap_here =
          jtzero_cap >= 2 &&
          (jtzero_mode == "CAP_GLOBAL" ||
           (jtzero_mode == "CAP_ROT" && jtzero_v1551_rot_active));
      if (jtzero_cap_here) {
        auto old_it = old_smart_factors_.find(lmk_id);
        CHECK(old_it != old_smart_factors_.end());
        CHECK_NE(old_it->second.second, -1);
        SmartStereoFactor::shared_ptr fresh(new SmartStereoFactor(
            smart_noise_, smart_factors_params_, B_Pose_leftCamRect_));
        const size_t n = ft.obs_.size();
        const size_t begin = n > static_cast<size_t>(jtzero_cap)
                                 ? n - static_cast<size_t>(jtzero_cap)
                                 : 0u;
        for (size_t j = begin; j < n; ++j) {
          const auto& obs = ft.obs_[j];
          fresh->add(obs.second,
                     gtsam::Symbol(kPoseSymbolChar, obs.first),
                     stereo_cal_);
        }
        new_smart_factors_.insert(std::make_pair(lmk_id, fresh));
        old_it->second.first = fresh;
      } else {
        updateLandmarkInGraph(lmk_id, obs_kf);
      }
      ++n_updated_landmarks;
    }
'''
if needle not in s: raise SystemExit('[FATAL] factor update anchor not found')
s=s.replace(needle,insert,1)
p.write_text(s)
PY

grep -q 'JTZERO_V1551_MODE' "$SRC" || { echo "[FATAL] diagnostic patch failed"; exit 1; }

echo "[BUILD] Kimera diagnostic build..."
cmake --build "$KIMERA/build" -j"$(nproc)"

echo "[BUILD] replay binary..."
g++ -std=c++17 -O2 \
  -I"$ROOT/tools" -I"$KIMERA/include" -I"$KIMERA/build" \
  -I"$KIMERA/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) \
  "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -L"$KIMERA/build" -L/usr/local/lib \
  -lkimera_vio -lgtsam -lgflags -lglog -lpthread \
  $(pkg-config --libs opencv4)

printf 'MODE\tCAP\tRC\tSTATES\tFINAL_DP_MM\tMAXEXC_MM\tDROLL\tDPITCH\tDYAW\n' > "$OUT/report.tsv"

run_one() {
  local mode="$1" cap="${2:-0}" tag="$3"
  local log="$OUT/${tag}.log"
  echo "[RUN] mode=$mode cap=$cap"
  set +e
  JTZERO_V1551_MODE="$mode" \
  JTZERO_V1551_CAP_OBS="$cap" \
  JTZERO_V1551_ROT_THRESH_DEG="$THRESH" \
  LD_LIBRARY_PATH="$KIMERA/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" \
    "$BIN" "$PARAMS" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
  local rc=$?
  set -e
  local states dp mx rpy vr vp vy
  states=$(awk '/backend states:/ {v=$3} END{print v+0}' "$log")
  dp=$(awk '/final \|dP\| mm:/ {v=$4} END{print v}' "$log")
  mx=$(awk '/max excursion mm:/ {v=$4} END{print v}' "$log")
  rpy=$(sed -n 's/.*final dRPY deg: \[\([^]]*\)\].*/\1/p' "$log" | tail -1)
  read -r vr vp vy <<<"${rpy:-NA NA NA}"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$mode" "$cap" "$rc" "$states" "${dp:-NA}" "${mx:-NA}" "$vr" "$vp" "$vy" >> "$OUT/report.tsv"
  echo "      rc=$rc states=$states dP=${dp:-NA} mm maxexc=${mx:-NA} mm dRPY=[$vr $vp $vy]"
}

# Lifecycle partition hypotheses.
run_one CONTROL 0 control
run_one NO_VIS_DURING_ROT 0 no_vis_during_rot
run_one NO_VIS_AFTER_TRIGGER 0 no_vis_after_trigger
run_one NO_VIS_BEFORE_TRIGGER 0 no_vis_before_trigger
run_one ROT_NEW_ONLY 0 rot_new_only
run_one ROT_EXISTING_ONLY 0 rot_existing_only
run_one NO_VIS_ALL 0 no_vis_all

# Direct long-SmartFactor-memory hypotheses, one build / runtime cap sweep.
for cap in 2 3 4 6 8 12 16 24; do
  run_one CAP_ROT "$cap" "cap_rot_${cap}"
done
for cap in 2 4 8 16; do
  run_one CAP_GLOBAL "$cap" "cap_global_${cap}"
done

echo
echo "================ v15.51 MATRIX ================"
column -t -s $'\t' "$OUT/report.tsv" || cat "$OUT/report.tsv"

echo
python3 - "$OUT/report.tsv" <<'PY'
import csv,sys,math
rows=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))
def val(r,k,typ=float):
    try:return typ(r[k])
    except:return None
valid=[r for r in rows if val(r,'RC',int)==0 and (val(r,'STATES',int) or 0)>=140 and val(r,'FINAL_DP_MM') is not None]
ctrl=next((r for r in valid if r['MODE']=='CONTROL'),None)
if not ctrl:
    print('V15_51_VERDICT=INVALID_CONTROL');print('RESULT: COMPLETE');raise SystemExit
c=float(ctrl['FINAL_DP_MM'])
print(f'CONTROL_DP_MM={c:.3f} CONTROL_STATES={ctrl["STATES"]}')
ranked=sorted((float(r['FINAL_DP_MM']),r) for r in valid if r['MODE']!='CONTROL')
for i,(d,r) in enumerate(ranked[:8],1):
    print(f'RANK{i}={r["MODE"]}:CAP={r["CAP"]}:DP_MM={d:.3f}:RATIO={d/c:.4f}:STATES={r["STATES"]}')

def get(mode,cap=None):
    q=[r for r in valid if r['MODE']==mode and (cap is None or r['CAP']==str(cap))]
    return float(q[0]['FINAL_DP_MM']) if q else None

during=get('NO_VIS_DURING_ROT'); after=get('NO_VIS_AFTER_TRIGGER'); before=get('NO_VIS_BEFORE_TRIGGER')
newonly=get('ROT_NEW_ONLY'); existing=get('ROT_EXISTING_ONLY'); novis=get('NO_VIS_ALL')
caprot=min((float(r['FINAL_DP_MM']),int(r['CAP'])) for r in valid if r['MODE']=='CAP_ROT') if any(r['MODE']=='CAP_ROT' for r in valid) else None
capglobal=min((float(r['FINAL_DP_MM']),int(r['CAP'])) for r in valid if r['MODE']=='CAP_GLOBAL') if any(r['MODE']=='CAP_GLOBAL' for r in valid) else None
if caprot: print(f'BEST_CAP_ROT_OBS={caprot[1]} BEST_CAP_ROT_DP_MM={caprot[0]:.3f}')
if capglobal: print(f'BEST_CAP_GLOBAL_OBS={capglobal[1]} BEST_CAP_GLOBAL_DP_MM={capglobal[0]:.3f}')
if during is not None: print(f'NO_VIS_DURING_ROT_DP_MM={during:.3f}')
if after is not None: print(f'NO_VIS_AFTER_TRIGGER_DP_MM={after:.3f}')
if before is not None: print(f'NO_VIS_BEFORE_TRIGGER_DP_MM={before:.3f}')
if newonly is not None: print(f'ROT_NEW_ONLY_DP_MM={newonly:.3f}')
if existing is not None: print(f'ROT_EXISTING_ONLY_DP_MM={existing:.3f}')
if novis is not None: print(f'NO_VIS_ALL_DP_MM={novis:.3f}')

# Broad structural verdict, deliberately conservative.
best=ranked[0][0] if ranked else c
if caprot and caprot[0] <= 0.35*c and caprot[0] <= 100:
    verdict='LONG_SMART_FACTOR_MEMORY_CAUSALLY_DOMINANT'
elif after is not None and during is not None and after <= 0.5*during:
    verdict='POST_TRIGGER_VISUAL_UPDATES_CAUSALLY_DOMINANT'
elif before is not None and before <= 0.5*c:
    verdict='PRE_ROTATION_TRACK_HISTORY_CAUSALLY_IMPORTANT'
elif newonly is not None and existing is not None and existing > 1.5*newonly:
    verdict='PREEXISTING_TRACKS_DOMINATE_ROTATION_ERROR'
elif newonly is not None and existing is not None and newonly > 1.5*existing:
    verdict='NEW_ROTATION_TRACKS_DOMINATE_ERROR'
elif best <= 0.6*c:
    verdict='VISUAL_LIFECYCLE_SIGNIFICANT_BUT_MIXED'
else:
    verdict='VISUAL_LIFECYCLE_FILTERS_NOT_SUFFICIENT'
print('V15_51_VERDICT='+verdict)
print('NOTE=Diagnostic-only matrix. No row is a production mitigation by itself.')
print('RESULT: COMPLETE')
PY

echo "Logs: $OUT"
echo "Source will now be restored and Kimera rebuilt clean by trap."
