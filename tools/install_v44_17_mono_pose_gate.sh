#!/usr/bin/env bash
set -euo pipefail

ROOT="${JTZERO_KIMERA_SRC:-/home/vio/Kimera-VIO}"
SRC="$ROOT/src/pipeline/MonoImuPipeline.cpp"

echo "======================================================================"
echo "V44.17b — INSTALL OPT-IN MONO POSE PRE-FUSION GATE"
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

inc_old = """#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>
"""
inc_new = """#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
"""

old = r'''  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue](const FrontendOutputPacketBase::Ptr& output) {
        auto converted_output =
            std::dynamic_pointer_cast<MonoFrontendOutput>(output);
        CHECK(converted_output);
        if (converted_output->is_keyframe_) {
          //! Only push to Backend input queue if it is a keyframe!
          backend_input_queue.push(std::make_unique<BackendInput>(
              converted_output->frame_lkf_.timestamp_,
              converted_output->status_mono_measurements_,
              converted_output->pim_,
              converted_output->imu_acc_gyrs_,
              converted_output->body_lkf_OdomPose_body_kf_,
              converted_output->body_kf_world_OdomVel_body_kf_));
        } else {
          VLOG(5)
              << "Frontend did not output a keyframe, skipping Backend input.";
        }
      });
'''
new = r'''  // JT-Zero diagnostic gate (OFF by default).
  //
  // V44.15/V44.16 isolated one current-only monocular pose discontinuity:
  // the accepted body-frame translation direction jumped by ~63 deg and
  // became ~56 deg out of plane while PIM rotation was only ~0.007 deg.
  // This gate converts ONLY such an anomalous VALID mono update to
  // LOW_DISPARITY before BackendInput is constructed. PIM/IMU propagation
  // continues exactly as it does for a naturally occurring LOW_DISPARITY row.
  //
  // Enable explicitly:
  //   JTZERO_MONO_POSE_GATE=1
  // Optional thresholds:
  //   JTZERO_MONO_POSE_GATE_JUMP_DEG=30
  //   JTZERO_MONO_POSE_GATE_TILT_DEG=30
  const auto env_flag = [](const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    return std::string(value) == "1" || std::string(value) == "true" ||
           std::string(value) == "TRUE" || std::string(value) == "on" ||
           std::string(value) == "ON";
  };
  const auto env_double = [](const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end != value && std::isfinite(parsed) ? parsed : fallback;
  };
  const bool jtzero_pose_gate = env_flag("JTZERO_MONO_POSE_GATE", false);
  const double jtzero_jump_deg =
      env_double("JTZERO_MONO_POSE_GATE_JUMP_DEG", 30.0);
  const double jtzero_tilt_deg =
      env_double("JTZERO_MONO_POSE_GATE_TILT_DEG", 30.0);
  auto jtzero_prev_good_body_t =
      std::make_shared<std::optional<gtsam::Vector3>>();

  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue,
       jtzero_pose_gate,
       jtzero_jump_deg,
       jtzero_tilt_deg,
       jtzero_prev_good_body_t](const FrontendOutputPacketBase::Ptr& output) {
        auto converted_output =
            std::dynamic_pointer_cast<MonoFrontendOutput>(output);
        CHECK(converted_output);
        if (converted_output->is_keyframe_) {
          StatusMonoMeasurementsPtr status_for_backend =
              converted_output->status_mono_measurements_;
          bool rejected_by_jtzero_gate = false;

          if (jtzero_pose_gate && status_for_backend &&
              status_for_backend->first.kfTrackingStatus_mono_ ==
                  TrackingStatus::VALID) {
            const gtsam::Vector3 cam_t =
                status_for_backend->first.lkf_T_k_mono_.translation();
            const gtsam::Vector3 body_t =
                converted_output->b_Pose_cam_rect_.rotation().rotate(cam_t);
            const double horizontal = std::hypot(body_t.x(), body_t.y());
            const double tilt_deg =
                std::atan2(std::abs(body_t.z()), horizontal) *
                180.0 / 3.14159265358979323846;

            double jump_deg = 0.0;
            bool have_jump = false;
            if (jtzero_prev_good_body_t->has_value()) {
              const auto& prev = jtzero_prev_good_body_t->value();
              const double denom = prev.norm() * body_t.norm();
              if (denom > 1e-12) {
                const double dot =
                    std::max(-1.0, std::min(1.0, prev.dot(body_t) / denom));
                jump_deg =
                    std::acos(dot) * 180.0 / 3.14159265358979323846;
                have_jump = true;
              }
            }

            if (have_jump && jump_deg >= jtzero_jump_deg &&
                tilt_deg >= jtzero_tilt_deg) {
              auto gated =
                  std::make_shared<StatusMonoMeasurements>(*status_for_backend);
              gated->first.kfTrackingStatus_mono_ =
                  TrackingStatus::LOW_DISPARITY;
              gated->first.lkf_T_k_mono_ = gtsam::Pose3();
              status_for_backend = gated;
              rejected_by_jtzero_gate = true;
              LOG(WARNING) << "[JTZERO-MONO-POSE-GATE] reject VALID mono pose"
                           << " ts=" << converted_output->frame_lkf_.timestamp_
                           << " jump_deg=" << jump_deg
                           << " tilt_deg=" << tilt_deg
                           << " body_t=[" << body_t.transpose() << "]";
            }

            if (!rejected_by_jtzero_gate) {
              *jtzero_prev_good_body_t = body_t;
            }
          }

          //! Only push to Backend input queue if it is a keyframe!
          backend_input_queue.push(std::make_unique<BackendInput>(
              converted_output->frame_lkf_.timestamp_,
              status_for_backend,
              converted_output->pim_,
              converted_output->imu_acc_gyrs_,
              converted_output->body_lkf_OdomPose_body_kf_,
              converted_output->body_kf_world_OdomVel_body_kf_));
        } else {
          VLOG(5)
              << "Frontend did not output a keyframe, skipping Backend input.";
        }
      });
'''

if inc_old not in s:
    raise SystemExit("ERROR: include anchor not found; source layout differs")
if old not in s:
    raise SystemExit("ERROR: backend callback anchor not found; source layout differs")

s = s.replace(inc_old, inc_new, 1)
s = s.replace(old, new, 1)
p.write_text(s)
print("SOURCE PATCH PASS")
PY
fi

echo
echo "===== SOURCE MARKERS ====="
grep -nE 'JTZERO_MONO_POSE_GATE|JTZERO-MONO-POSE-GATE|status_for_backend' "$SRC"

echo
echo "===== BUILD ====="
cmake --build "$ROOT/build" -j2

echo
echo "===== RESULT ====="
echo "Kimera rebuilt with gate support."
echo "Default remains OFF."
echo
echo "Next diagnostic run:"
echo "  export JTZERO_MONO_POSE_GATE=1"
echo "  export JTZERO_MONO_POSE_GATE_JUMP_DEG=30"
echo "  export JTZERO_MONO_POSE_GATE_TILT_DEG=30"
echo
echo "Rollback source if needed:"
echo "  cp '$SRC.v44_17_pre_gate.bak' '$SRC'"
echo "  cmake --build '$ROOT/build' -j2"
