#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.21 — INSTALL CRASH-SAFE MONO POSE GATE TELEMETRY"
echo "======================================================================"
echo "Kimera source: $ROOT"
echo

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: source not found: $SRC"
  exit 1
fi

if ! grep -q 'JTZERO-MONO-POSE-GATE-SUMMARY' "$SRC"; then
  echo "ERROR: V44.20 counters are not installed."
  exit 1
fi

if grep -q 'JTZERO_MONO_POSE_GATE_STATS_FILE' "$SRC"; then
  echo "Crash-safe telemetry already installed."
else
  cp "$SRC" "$SRC.v44_21_pre_telemetry.bak"

  /usr/bin/python3 - "$SRC" <<'PY'
from pathlib import Path
import sys

p = Path(sys.argv[1])
s = p.read_text()

inc_old = """#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
"""
inc_new = """#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
"""
if inc_old not in s:
    raise SystemExit("ERROR: include anchor not found")
s = s.replace(inc_old, inc_new, 1)

state_old = """  struct JtzeroMonoPoseGateStats {
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
"""
state_new = """  struct JtzeroMonoPoseGateStats {
    bool enabled = false;
    uint64_t evaluated = 0;
    uint64_t rejected = 0;
    double max_jump_deg = 0.0;
    double max_tilt_deg = 0.0;
    std::string stats_file;

    void persist(const char* phase) const {
      if (stats_file.empty()) return;
      std::ofstream out(stats_file, std::ios::out | std::ios::trunc);
      if (!out.is_open()) return;
      out << "phase=" << phase
          << " enabled=" << (enabled ? 1 : 0)
          << " evaluated=" << evaluated
          << " rejected=" << rejected
          << " max_jump_deg=" << max_jump_deg
          << " max_tilt_deg=" << max_tilt_deg
          << "\\n";
      out.flush();
    }

    ~JtzeroMonoPoseGateStats() {
      persist("shutdown");
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
  if (const char* e = std::getenv("JTZERO_MONO_POSE_GATE_STATS_FILE")) {
    jtzero_gate_stats->stats_file = e;
  }
  jtzero_gate_stats->persist("init");
"""
if state_old not in s:
    raise SystemExit("ERROR: V44.20 stats anchor not found")
s = s.replace(state_old, state_new, 1)

eval_old = """            ++jtzero_gate_stats->evaluated;
            jtzero_gate_stats->max_tilt_deg =
                std::max(jtzero_gate_stats->max_tilt_deg, tilt_deg);
            if (have_jump) {
              jtzero_gate_stats->max_jump_deg =
                  std::max(jtzero_gate_stats->max_jump_deg, jump_deg);
            }
"""
eval_new = """            ++jtzero_gate_stats->evaluated;
            jtzero_gate_stats->max_tilt_deg =
                std::max(jtzero_gate_stats->max_tilt_deg, tilt_deg);
            if (have_jump) {
              jtzero_gate_stats->max_jump_deg =
                  std::max(jtzero_gate_stats->max_jump_deg, jump_deg);
            }
            jtzero_gate_stats->persist("evaluated");
"""
if eval_old not in s:
    raise SystemExit("ERROR: V44.20 evaluation anchor not found")
s = s.replace(eval_old, eval_new, 1)

rej_old = """            if (reject) {
              ++jtzero_gate_stats->rejected;
"""
rej_new = """            if (reject) {
              ++jtzero_gate_stats->rejected;
              jtzero_gate_stats->persist("rejected");
"""
if rej_old not in s:
    raise SystemExit("ERROR: V44.20 reject anchor not found")
s = s.replace(rej_old, rej_new, 1)

p.write_text(s)
print("SOURCE PATCH PASS")
PY
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'JTZERO_MONO_POSE_GATE_STATS_FILE|persist\(|phase=|fstream' "$SRC"

echo
echo "===== BUILD ====="
cmake --build "$ROOT/build" -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with crash-safe gate telemetry."
echo "The telemetry file is updated at init, every evaluation, and every rejection."
