#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.17d — INSTALL OPT-IN MONO POSE PRE-FUSION GATE"
echo "======================================================================"
echo "Kimera source: $ROOT"
echo

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: source not found: $SRC"
  exit 1
fi

if grep -q 'JTZERO-MONO-POSE-GATE' "$SRC"; then
  echo "Gate source already installed."
else
  cp "$SRC" "$SRC.v44_17_pre_gate.bak"

  /usr/bin/python3 - "$SRC" <<'PY'
from pathlib import Path
import sys

p = Path(sys.argv[1])
s = p.read_text()

# 1) Includes: local JT-Zero Kimera already has <cstdlib>.
inc_old = """#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cstdlib>
#include <string>
"""
inc_new = """#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
"""
if inc_old not in s:
    raise SystemExit("ERROR: local include anchor not found")
s = s.replace(inc_old, inc_new, 1)

# 2) Persistent state must live outside the callback.
state_anchor = """  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue](const FrontendOutputPacketBase::Ptr& output) {
"""
state_repl = """  // JT-Zero V44.17 diagnostic gate state (OFF unless env enables it).
  const auto jtzero_gate_enabled = []() {
    const char* e = std::getenv("JTZERO_MONO_POSE_GATE");
    if (!e) return false;
    const std::string v(e);
    return v == "1" || v == "true" || v == "TRUE" ||
           v == "on" || v == "ON";
  }();
  const auto jtzero_env_double = [](const char* name, double fallback) {
    const char* e = std::getenv(name);
    if (!e) return fallback;
    char* end = nullptr;
    const double v = std::strtod(e, &end);
    return end != e && std::isfinite(v) ? v : fallback;
  };
  const double jtzero_gate_jump_deg =
      jtzero_env_double("JTZERO_MONO_POSE_GATE_JUMP_DEG", 30.0);
  const double jtzero_gate_tilt_deg =
      jtzero_env_double("JTZERO_MONO_POSE_GATE_TILT_DEG", 30.0);
  auto jtzero_prev_good_body_t = std::make_shared<gtsam::Vector3>();
  auto jtzero_have_prev_good_body_t = std::make_shared<bool>(false);

  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue,
       jtzero_gate_enabled,
       jtzero_gate_jump_deg,
       jtzero_gate_tilt_deg,
       jtzero_prev_good_body_t,
       jtzero_have_prev_good_body_t](
          const FrontendOutputPacketBase::Ptr& output) {
"""
if state_anchor not in s:
    raise SystemExit("ERROR: local callback state anchor not found")
s = s.replace(state_anchor, state_repl, 1)

# 3) Insert the gate after the already-existing JTZERO_DIAG_IMU_ONLY logic
# and before BackendInput construction.
gate_anchor = """          if (std::getenv("JTZERO_DIAG_IMU_ONLY") &&
              converted_output->status_mono_measurements_) {
            jtzero_diag_measurements =
                std::make_shared<StatusMonoMeasurements>(
                    *converted_output->status_mono_measurements_);
            jtzero_diag_measurements->second.clear();
            // JT-ZERO TRUE IMU-ONLY: disable LOW_DISPARITY backend constraints
            // Clearing measurements alone is not enough: LOW_DISPARITY in
            // TrackerStatusSummary makes VioBackend add ZeroVelocityPrior and
            // NoMotionFactor. Force INVALID only in diagnostic mode so the
            // backend receives IMU factors without visual/no-motion constraints.
            jtzero_diag_measurements->first.kfTrackingStatus_mono_ =
                TrackingStatus::INVALID;
          }

          //! Only push to Backend input queue if it is a keyframe!
"""
gate_repl = """          if (std::getenv("JTZERO_DIAG_IMU_ONLY") &&
              converted_output->status_mono_measurements_) {
            jtzero_diag_measurements =
                std::make_shared<StatusMonoMeasurements>(
                    *converted_output->status_mono_measurements_);
            jtzero_diag_measurements->second.clear();
            // JT-ZERO TRUE IMU-ONLY: disable LOW_DISPARITY backend constraints
            // Clearing measurements alone is not enough: LOW_DISPARITY in
            // TrackerStatusSummary makes VioBackend add ZeroVelocityPrior and
            // NoMotionFactor. Force INVALID only in diagnostic mode so the
            // backend receives IMU factors without visual/no-motion constraints.
            jtzero_diag_measurements->first.kfTrackingStatus_mono_ =
                TrackingStatus::INVALID;
          }

          // JT-ZERO V44.17: reject a current-only accepted monocular
          // translation-direction discontinuity before backend fusion.
          //
          // IMPORTANT: use INVALID, NOT LOW_DISPARITY. This local Kimera tree
          // explicitly documents that LOW_DISPARITY creates ZeroVelocityPrior
          // and NoMotionFactor in the backend. For an anomalous visual pose we
          // want one IMU-propagated interval with no visual/no-motion factor.
          if (jtzero_gate_enabled &&
              !std::getenv("JTZERO_DIAG_IMU_ONLY") &&
              converted_output->status_mono_measurements_ &&
              converted_output->status_mono_measurements_
                      ->first.kfTrackingStatus_mono_ ==
                  TrackingStatus::VALID) {
            const gtsam::Vector3 cam_t =
                converted_output->status_mono_measurements_
                    ->first.lkf_T_k_mono_.translation();
            const gtsam::Vector3 body_t =
                converted_output->b_Pose_cam_rect_.rotation().rotate(cam_t);

            const double horizontal =
                std::hypot(body_t.x(), body_t.y());
            const double tilt_deg =
                std::atan2(std::abs(body_t.z()), horizontal) *
                180.0 / 3.14159265358979323846;

            double jump_deg = 0.0;
            bool have_jump = false;
            if (*jtzero_have_prev_good_body_t) {
              const double denom =
                  jtzero_prev_good_body_t->norm() * body_t.norm();
              if (denom > 1e-12) {
                const double c =
                    std::max(-1.0,
                             std::min(
                                 1.0,
                                 jtzero_prev_good_body_t->dot(body_t) /
                                     denom));
                jump_deg =
                    std::acos(c) * 180.0 / 3.14159265358979323846;
                have_jump = true;
              }
            }

            const bool reject =
                have_jump &&
                jump_deg >= jtzero_gate_jump_deg &&
                tilt_deg >= jtzero_gate_tilt_deg;

            if (reject) {
              jtzero_diag_measurements =
                  std::make_shared<StatusMonoMeasurements>(
                      *converted_output->status_mono_measurements_);
              jtzero_diag_measurements->second.clear();
              jtzero_diag_measurements->first.kfTrackingStatus_mono_ =
                  TrackingStatus::INVALID;
              jtzero_diag_measurements->first.lkf_T_k_mono_ =
                  gtsam::Pose3();

              LOG(WARNING)
                  << "[JTZERO-MONO-POSE-GATE] reject VALID mono pose"
                  << " ts=" << converted_output->frame_lkf_.timestamp_
                  << " jump_deg=" << jump_deg
                  << " tilt_deg=" << tilt_deg
                  << " body_t=[" << body_t.transpose() << "]";
              // Do not update previous-good direction with the rejected pose.
            } else {
              *jtzero_prev_good_body_t = body_t;
              *jtzero_have_prev_good_body_t = true;
            }
          }

          //! Only push to Backend input queue if it is a keyframe!
"""
if gate_anchor not in s:
    raise SystemExit("ERROR: local JTZERO_DIAG_IMU_ONLY anchor not found")
s = s.replace(gate_anchor, gate_repl, 1)

p.write_text(s)
print("SOURCE PATCH PASS")
PY
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'JTZERO_MONO_POSE_GATE|JTZERO-MONO-POSE-GATE|jtzero_gate_enabled|TrackingStatus::INVALID' "$SRC"

echo
echo "===== BUILD ====="
cmake --build "$ROOT/build" -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with V44.17d gate support."
echo "Default remains OFF."
echo
echo "For the gated diagnostic run:"
echo "  export JTZERO_MONO_POSE_GATE=1"
echo "  export JTZERO_MONO_POSE_GATE_JUMP_DEG=30"
echo "  export JTZERO_MONO_POSE_GATE_TILT_DEG=30"
echo
echo "Rollback if needed:"
echo "  cp '$SRC.v44_17_pre_gate.bak' '$SRC'"
echo "  cmake --build '$ROOT/build' -j2"
