#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.22 — INSTALL CALLBACK/STATUS TELEMETRY"
echo "======================================================================"
echo "Kimera source: $ROOT"
echo

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: source not found: $SRC"
  exit 1
fi

if ! grep -q 'JTZERO_MONO_POSE_GATE_STATS_FILE' "$SRC"; then
  echo "ERROR: V44.21 crash-safe telemetry is not installed."
  exit 1
fi

if grep -q 'keyframes_seen=' "$SRC"; then
  echo "V44.22 callback/status telemetry already installed."
else
  cp "$SRC" "$SRC.v44_22_pre_status_telemetry.bak"

  /usr/bin/python3 - "$SRC" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text()

fields_old='''    uint64_t evaluated = 0;
    uint64_t rejected = 0;
    double max_jump_deg = 0.0;
    double max_tilt_deg = 0.0;
    std::string stats_file;
'''
fields_new='''    uint64_t keyframes_seen = 0;
    uint64_t valid_seen = 0;
    uint64_t low_disparity_seen = 0;
    uint64_t invalid_seen = 0;
    uint64_t evaluated = 0;
    uint64_t rejected = 0;
    double max_jump_deg = 0.0;
    double max_tilt_deg = 0.0;
    std::string stats_file;
'''
if fields_old not in s:
    raise SystemExit("ERROR: stats field anchor not found")
s=s.replace(fields_old,fields_new,1)

persist_old='''      out << "phase=" << phase
          << " enabled=" << (enabled ? 1 : 0)
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg
          << "\\n";
'''
persist_new='''      out << "phase=" << phase
          << " enabled=" << (enabled ? 1 : 0)
          << " keyframes_seen=" << keyframes_seen
          << " valid_seen=" << valid_seen
          << " low_disparity_seen=" << low_disparity_seen
          << " invalid_seen=" << invalid_seen
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg
          << "\\n";
'''
if persist_old not in s:
    raise SystemExit("ERROR: persist anchor not found")
s=s.replace(persist_old,persist_new,1)

summary_old='''          << " enabled=" << (enabled ? 1 : 0)
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg;
'''
summary_new='''          << " enabled=" << (enabled ? 1 : 0)
          << " keyframes_seen=" << keyframes_seen
          << " valid_seen=" << valid_seen
          << " low_disparity_seen=" << low_disparity_seen
          << " invalid_seen=" << invalid_seen
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg;
'''
if summary_old not in s:
    raise SystemExit("ERROR: shutdown summary anchor not found")
s=s.replace(summary_old,summary_new,1)

callback_anchor='''        CHECK(converted_output);
        if (converted_output->is_keyframe_) {
          // JT-ZERO diagnostic: when JTZERO_DIAG_IMU_ONLY is set, keep the
'''
callback_repl='''        CHECK(converted_output);
        if (converted_output->is_keyframe_) {
          ++jtzero_gate_stats->keyframes_seen;
          if (converted_output->status_mono_measurements_) {
            const auto jtzero_status =
                converted_output->status_mono_measurements_
                    ->first.kfTrackingStatus_mono_;
            if (jtzero_status == TrackingStatus::VALID) {
              ++jtzero_gate_stats->valid_seen;
            } else if (jtzero_status == TrackingStatus::LOW_DISPARITY) {
              ++jtzero_gate_stats->low_disparity_seen;
            } else {
              ++jtzero_gate_stats->invalid_seen;
            }
          } else {
            ++jtzero_gate_stats->invalid_seen;
          }
          jtzero_gate_stats->persist("keyframe");

          // JT-ZERO diagnostic: when JTZERO_DIAG_IMU_ONLY is set, keep the
'''
if callback_anchor not in s:
    raise SystemExit("ERROR: callback keyframe anchor not found")
s=s.replace(callback_anchor,callback_repl,1)

p.write_text(s)
print("SOURCE PATCH PASS")
PY
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'keyframes_seen|valid_seen|low_disparity_seen|invalid_seen|persist\("keyframe"\)' "$SRC"

echo
echo "===== BUILD ====="
cmake --build "$ROOT/build" -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with V44.22 callback/status telemetry."
echo "A stationary smoke test should now prove whether the frontend callback is actually executing."
