#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.20 — INSTALL PERSISTENT MONO POSE GATE COUNTERS"
echo "======================================================================"
echo "Kimera source: $ROOT"
echo

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: source not found: $SRC"
  exit 1
fi

if ! grep -q 'JTZERO-MONO-POSE-GATE' "$SRC"; then
  echo "ERROR: V44.17d gate source is not installed."
  echo "Run tools/install_v44_17_mono_pose_gate.sh first."
  exit 1
fi

if grep -q 'JTZERO-MONO-POSE-GATE-SUMMARY' "$SRC"; then
  echo "Persistent counters already installed."
else
  cp "$SRC" "$SRC.v44_20_pre_counters.bak"

  /usr/bin/python3 - "$SRC" <<'PY'
from pathlib import Path
import sys

p = Path(sys.argv[1])
s = p.read_text()

state_anchor = '''  const double jtzero_gate_tilt_deg =
      jtzero_env_double("JTZERO_MONO_POSE_GATE_TILT_DEG", 30.0);
  auto jtzero_prev_good_body_t = std::make_shared<gtsam::Vector3>();
  auto jtzero_have_prev_good_body_t = std::make_shared<bool>(false);
'''
state_repl = '''  const double jtzero_gate_tilt_deg =
      jtzero_env_double("JTZERO_MONO_POSE_GATE_TILT_DEG", 30.0);

  struct JtzeroMonoPoseGateStats {
    bool enabled = false;
    uint64_t evaluated = 0;
    uint64_t rejected = 0;
    double max_jump_deg = 0.0;
    double max_tilt_deg = 0.0;
    ~JtzeroMonoPoseGateStats() {
      LOG(WARNING)
          << "[JTZERO-MONO-POSE-GATE-SUMMARY]"
          << " enabled=" << (enabled ? 1 : 0)
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg;
    }
  };
  auto jtzero_gate_stats = std::make_shared<JtzeroMonoPoseGateStats>();
  jtzero_gate_stats->enabled = jtzero_gate_enabled;

  auto jtzero_prev_good_body_t = std::make_shared<gtsam::Vector3>();
  auto jtzero_have_prev_good_body_t = std::make_shared<bool>(false);
'''
if state_anchor not in s:
    raise SystemExit("ERROR: V44.17d gate-state anchor not found")
s = s.replace(state_anchor, state_repl, 1)

capture_anchor = '''       jtzero_gate_tilt_deg,
       jtzero_prev_good_body_t,
       jtzero_have_prev_good_body_t](
          const FrontendOutputPacketBase::Ptr& output) {
'''
capture_repl = '''       jtzero_gate_tilt_deg,
       jtzero_gate_stats,
       jtzero_prev_good_body_t,
       jtzero_have_prev_good_body_t](
          const FrontendOutputPacketBase::Ptr& output) {
'''
if capture_anchor not in s:
    raise SystemExit("ERROR: V44.17d lambda-capture anchor not found")
s = s.replace(capture_anchor, capture_repl, 1)

eval_anchor = '''            const bool reject =
                have_jump &&
                jump_deg >= jtzero_gate_jump_deg &&
                tilt_deg >= jtzero_gate_tilt_deg;

            if (reject) {
'''
eval_repl = '''            ++jtzero_gate_stats->evaluated;
            jtzero_gate_stats->max_tilt_deg =
                std::max(jtzero_gate_stats->max_tilt_deg, tilt_deg);
            if (have_jump) {
              jtzero_gate_stats->max_jump_deg =
                  std::max(jtzero_gate_stats->max_jump_deg, jump_deg);
            }

            const bool reject =
                have_jump &&
                jump_deg >= jtzero_gate_jump_deg &&
                tilt_deg >= jtzero_gate_tilt_deg;

            if (reject) {
              ++jtzero_gate_stats->rejected;
'''
if eval_anchor not in s:
    raise SystemExit("ERROR: V44.17d reject-decision anchor not found")
s = s.replace(eval_anchor, eval_repl, 1)

p.write_text(s)
print("SOURCE PATCH PASS")
PY
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'JTZERO-MONO-POSE-GATE-SUMMARY|evaluated|rejected|max_jump_deg|max_tilt_deg' "$SRC"

echo
echo "===== BUILD ====="
cmake --build "$ROOT/build" -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with persistent V44.20 gate counters."
echo "The gate remains OFF by default."
echo
echo "On shutdown, expect exactly one line:"
echo "  [JTZERO-MONO-POSE-GATE-SUMMARY] enabled=... evaluated=... rejected=... max_jump_deg=... max_tilt_deg=..."
echo
echo "Rollback if needed:"
echo "  cp '$SRC.v44_20_pre_counters.bak' '$SRC'"
echo "  cmake --build '$ROOT/build' -j2"
