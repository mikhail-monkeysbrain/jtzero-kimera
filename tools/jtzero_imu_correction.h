#pragma once

#include <cmath>
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace jtzero {

// Validated on the independent v42 dataset and the 300 mm v4 run.
// Input convention is FLU (x forward, y left, z up).
class ImuCorrection {
 public:
  static constexpr double kGyroCx = 0.014570;
  static constexpr double kGyroCy = 0.082383;
  static constexpr double kGravityKp = 1.0;
  static constexpr double kGravityMps2 = 9.81;
  static constexpr double kGravityAccTolMps2 = 0.35;

  void reset() {
    initialized_ = false;
    last_imu_us_ = 0;
    gravity_body_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  }

  // ArduPilot HIGHRES_IMU is FRD. Convert both accelerometer and gyro to FLU.
  static Eigen::Vector3d accelFrdToFlu(double xacc, double yacc, double zacc) {
    return Eigen::Vector3d(xacc, -yacc, -zacc);
  }

  static Eigen::Vector3d gyroFrdToFlu(double xgyro, double ygyro, double zgyro) {
    return Eigen::Vector3d(xgyro, -ygyro, -zgyro);
  }

  // Apply the fixed Z->XY gyro correction established by v41/v42.
  static Eigen::Vector3d applyZxy(const Eigen::Vector3d& gyro_flu) {
    return Eigen::Vector3d(
        gyro_flu.x() + kGyroCx * gyro_flu.z(),
        gyro_flu.y() + kGyroCy * gyro_flu.z(),
        gyro_flu.z());
  }

  // Full correction used before feeding gyro to Kimera.
  // Accelerometer is used only as a gravity direction reference when |a| is
  // close to g; yaw remains gyro-only.
  Eigen::Vector3d correctGyro(uint64_t imu_us,
                              const Eigen::Vector3d& accel_flu,
                              const Eigen::Vector3d& gyro_flu) {
    const Eigen::Vector3d gyro_zxy = applyZxy(gyro_flu);

    double dt = 0.0;
    if (last_imu_us_ != 0 && imu_us > last_imu_us_) {
      dt = static_cast<double>(imu_us - last_imu_us_) * 1e-6;
    }
    last_imu_us_ = imu_us;

    if (dt <= 0.0 || dt > 0.03) return gyro_zxy;

    const double acc_norm = accel_flu.norm();
    const bool gravity_valid =
        acc_norm > 1e-6 &&
        std::abs(acc_norm - kGravityMps2) < kGravityAccTolMps2;

    if (!initialized_) {
      if (gravity_valid) {
        gravity_body_ = accel_flu.normalized();
        initialized_ = true;
      }
      return gyro_zxy;
    }

    Eigen::Vector3d corrected = gyro_zxy;
    if (gravity_valid) {
      const Eigen::Vector3d measured_gravity = accel_flu.normalized();
      const Eigen::Vector3d gravity_error =
          gravity_body_.cross(measured_gravity);
      corrected -= kGravityKp * gravity_error;
    }

    // Propagate expected gravity direction in body coordinates using the same
    // corrected angular rate that is returned to the VIO pipeline.
    const Eigen::Vector3d theta = -corrected * dt;
    const double angle = theta.norm();
    if (angle > 1e-12) {
      gravity_body_ =
          Eigen::AngleAxisd(angle, theta / angle) * gravity_body_;
      gravity_body_.normalize();
    }

    return corrected;
  }

 private:
  bool initialized_ = false;
  uint64_t last_imu_us_ = 0;
  Eigen::Vector3d gravity_body_ = Eigen::Vector3d(0.0, 0.0, 1.0);
};

}  // namespace jtzero
