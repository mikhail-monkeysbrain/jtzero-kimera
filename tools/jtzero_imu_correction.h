#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace jtzero {

// IMU correction used before feeding Kimera.
// Input convention is FLU (x forward, y left, z up).
//
// v7 change: accelerometer gravity feedback is no longer allowed to act during
// translational motion. The caller may explicitly disable it while a measured
// leg is in progress, and an internal static detector must still agree before
// any tilt correction is applied.
class ImuCorrection {
 public:
  static constexpr double kGyroCx = 0.014570;
  static constexpr double kGyroCy = 0.082383;
  static constexpr double kGravityMps2 = 9.81;

  // Conservative static-only gravity feedback.
  static constexpr double kGravityKp = 0.35;
  static constexpr double kGravityAccTolMps2 = 0.25;
  static constexpr double kStaticGyroMaxRadS = 0.035;
  static constexpr double kStaticAccelResidualMaxMps2 = 0.12;
  static constexpr double kStaticHoldSec = 0.25;
  static constexpr double kMaxGravityCorrectionRadS = 0.012;
  static constexpr double kMaxGravityErrorRad = 5.0 * M_PI / 180.0;
  static constexpr double kAccelLpTauSec = 0.20;

  void reset() {
    initialized_ = false;
    accel_lp_initialized_ = false;
    last_imu_us_ = 0;
    static_time_sec_ = 0.0;
    gravity_body_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    accel_lp_ = Eigen::Vector3d::Zero();
  }

  static Eigen::Vector3d accelFrdToFlu(double xacc, double yacc, double zacc) {
    return Eigen::Vector3d(xacc, -yacc, -zacc);
  }

  static Eigen::Vector3d gyroFrdToFlu(double xgyro, double ygyro, double zgyro) {
    return Eigen::Vector3d(xgyro, -ygyro, -zgyro);
  }

  // Fixed Z->XY gyro correction established by v41/v42.
  static Eigen::Vector3d applyZxy(const Eigen::Vector3d& gyro_flu) {
    return Eigen::Vector3d(
        gyro_flu.x() + kGyroCx * gyro_flu.z(),
        gyro_flu.y() + kGyroCy * gyro_flu.z(),
        gyro_flu.z());
  }

  // allow_gravity_feedback=false means gyro-only propagation (after ZXY).
  // This is used during deliberate translation in v7 so linear acceleration
  // cannot be mistaken for a change of gravity direction.
  Eigen::Vector3d correctGyro(uint64_t imu_us,
                              const Eigen::Vector3d& accel_flu,
                              const Eigen::Vector3d& gyro_flu,
                              bool allow_gravity_feedback = true,
                              bool apply_zxy = true) {
    const Eigen::Vector3d gyro_zxy = apply_zxy ? applyZxy(gyro_flu) : gyro_flu;

    double dt = 0.0;
    if (last_imu_us_ != 0 && imu_us > last_imu_us_) {
      dt = static_cast<double>(imu_us - last_imu_us_) * 1e-6;
    }
    last_imu_us_ = imu_us;
    if (dt <= 0.0 || dt > 0.03) return gyro_zxy;

    if (!accel_lp_initialized_) {
      accel_lp_ = accel_flu;
      accel_lp_initialized_ = true;
    } else {
      const double alpha = std::exp(-dt / kAccelLpTauSec);
      accel_lp_ = alpha * accel_lp_ + (1.0 - alpha) * accel_flu;
    }

    const double acc_norm = accel_flu.norm();
    const bool gravity_magnitude_ok =
        acc_norm > 1e-6 &&
        std::abs(acc_norm - kGravityMps2) <= kGravityAccTolMps2;
    const bool gyro_quiet = gyro_zxy.norm() <= kStaticGyroMaxRadS;
    const bool accel_quiet =
        (accel_flu - accel_lp_).norm() <= kStaticAccelResidualMaxMps2;
    const bool static_sample =
        allow_gravity_feedback && gravity_magnitude_ok && gyro_quiet && accel_quiet;

    if (static_sample) static_time_sec_ += dt;
    else static_time_sec_ = 0.0;

    const bool static_confirmed = static_time_sec_ >= kStaticHoldSec;

    if (!initialized_) {
      if (static_confirmed) {
        gravity_body_ = accel_lp_.normalized();
        initialized_ = true;
      }
      return gyro_zxy;
    }

    Eigen::Vector3d corrected = gyro_zxy;
    if (static_confirmed) {
      const Eigen::Vector3d measured_gravity = accel_lp_.normalized();
      Eigen::Vector3d gravity_error = gravity_body_.cross(measured_gravity);
      const double error_norm = gravity_error.norm();

      // Do not snap to a radically different acceleration direction. A valid
      // static correction is expected to be only a small roll/pitch trim.
      if (error_norm <= std::sin(kMaxGravityErrorRad)) {
        Eigen::Vector3d correction = kGravityKp * gravity_error;
        const double correction_norm = correction.norm();
        if (correction_norm > kMaxGravityCorrectionRadS) {
          correction *= kMaxGravityCorrectionRadS / correction_norm;
        }
        corrected -= correction;
      }
    }

    // Always propagate predicted gravity with the exact angular rate returned
    // to Kimera. During motion this is pure corrected-gyro propagation.
    const Eigen::Vector3d theta = -corrected * dt;
    const double angle = theta.norm();
    if (angle > 1e-12) {
      gravity_body_ = Eigen::AngleAxisd(angle, theta / angle) * gravity_body_;
      gravity_body_.normalize();
    }

    return corrected;
  }

 private:
  bool initialized_ = false;
  bool accel_lp_initialized_ = false;
  uint64_t last_imu_us_ = 0;
  double static_time_sec_ = 0.0;
  Eigen::Vector3d gravity_body_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d accel_lp_ = Eigen::Vector3d::Zero();
};

}  // namespace jtzero
