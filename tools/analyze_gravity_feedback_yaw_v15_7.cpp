// JT-ZERO Stage 11 gravity-feedback timing diagnostic v15.7.
// Offline only. Does NOT run Kimera and does NOT modify production correction.
// Reproduces tools/jtzero_imu_correction.h logic on a recorded combined CSV and
// reports exactly when static gravity feedback becomes active relative to yaw.

#include "jtzero_imu_correction.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kYawActiveRadS = 5.0 * M_PI / 180.0;

struct ImuRow {
  int64_t source_ns = 0;
  int64_t mapped_ns = 0;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : line) {
    if (ch == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  out.push_back(cur);
  return out;
}

long long toI64(const std::string& s) { return s.empty() ? 0 : std::stoll(s); }
double toDouble(const std::string& s) { return s.empty() ? 0.0 : std::stod(s); }

std::vector<ImuRow> loadImu(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open combined CSV: " + path);

  std::vector<ImuRow> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    const auto c = splitCsv(line);
    if (c.size() < 14 || c[0] != "IMU") continue;
    ImuRow s;
    s.source_ns = toI64(c[2]);
    s.mapped_ns = toI64(c[3]);
    s.ax = toDouble(c[8]);
    s.ay = toDouble(c[9]);
    s.az = toDouble(c[10]);
    s.gx = toDouble(c[11]);
    s.gy = toDouble(c[12]);
    s.gz = toDouble(c[13]);
    if (s.source_ns > 0 && s.mapped_ns > 0) out.push_back(s);
  }
  if (out.empty()) throw std::runtime_error("No IMU rows in combined CSV");
  return out;
}

enum class Phase { PRE, YAW, POST };
const char* phaseName(Phase p) {
  switch (p) {
    case Phase::PRE: return "PRE";
    case Phase::YAW: return "YAW";
    case Phase::POST: return "POST";
  }
  return "?";
}

struct PhaseStats {
  size_t samples = 0;
  size_t gravity_mag_ok = 0;
  size_t gyro_quiet = 0;
  size_t accel_quiet = 0;
  size_t static_sample = 0;
  size_t static_confirmed = 0;
  size_t correction_applied = 0;
  double correction_active_sec = 0.0;
  Eigen::Vector3d correction_integral_rad = Eigen::Vector3d::Zero();
  double max_correction_rad_s = 0.0;
};

class DiagnosticCorrection {
 public:
  struct Step {
    double dt = 0.0;
    Eigen::Vector3d gyro_zxy = Eigen::Vector3d::Zero();
    Eigen::Vector3d correction = Eigen::Vector3d::Zero();
    double acc_norm = 0.0;
    double accel_residual = 0.0;
    bool gravity_magnitude_ok = false;
    bool gyro_quiet = false;
    bool accel_quiet = false;
    bool static_sample = false;
    bool static_confirmed = false;
    bool initialized = false;
    bool correction_applied = false;
  };

  Step process(uint64_t imu_us,
               const Eigen::Vector3d& accel_flu,
               const Eigen::Vector3d& gyro_flu) {
    Step s;
    s.gyro_zxy = jtzero::ImuCorrection::applyZxy(gyro_flu);

    if (last_imu_us_ != 0 && imu_us > last_imu_us_) {
      s.dt = static_cast<double>(imu_us - last_imu_us_) * 1e-6;
    }
    last_imu_us_ = imu_us;
    if (s.dt <= 0.0 || s.dt > 0.03) {
      s.initialized = initialized_;
      return s;
    }

    if (!accel_lp_initialized_) {
      accel_lp_ = accel_flu;
      accel_lp_initialized_ = true;
    } else {
      const double alpha = std::exp(-s.dt / jtzero::ImuCorrection::kAccelLpTauSec);
      accel_lp_ = alpha * accel_lp_ + (1.0 - alpha) * accel_flu;
    }

    s.acc_norm = accel_flu.norm();
    s.accel_residual = (accel_flu - accel_lp_).norm();
    s.gravity_magnitude_ok =
        s.acc_norm > 1e-6 &&
        std::abs(s.acc_norm - jtzero::ImuCorrection::kGravityMps2) <=
            jtzero::ImuCorrection::kGravityAccTolMps2;
    s.gyro_quiet = s.gyro_zxy.norm() <= jtzero::ImuCorrection::kStaticGyroMaxRadS;
    s.accel_quiet = s.accel_residual <= jtzero::ImuCorrection::kStaticAccelResidualMaxMps2;
    s.static_sample = s.gravity_magnitude_ok && s.gyro_quiet && s.accel_quiet;

    if (s.static_sample) static_time_sec_ += s.dt;
    else static_time_sec_ = 0.0;
    s.static_confirmed = static_time_sec_ >= jtzero::ImuCorrection::kStaticHoldSec;

    if (!initialized_) {
      if (s.static_confirmed) {
        gravity_body_ = accel_lp_.normalized();
        initialized_ = true;
      }
      s.initialized = initialized_;
      return s;
    }

    Eigen::Vector3d corrected = s.gyro_zxy;
    if (s.static_confirmed) {
      const Eigen::Vector3d measured_gravity = accel_lp_.normalized();
      const Eigen::Vector3d gravity_error = gravity_body_.cross(measured_gravity);
      const double error_norm = gravity_error.norm();
      if (error_norm <= std::sin(jtzero::ImuCorrection::kMaxGravityErrorRad)) {
        s.correction = jtzero::ImuCorrection::kGravityKp * gravity_error;
        const double n = s.correction.norm();
        if (n > jtzero::ImuCorrection::kMaxGravityCorrectionRadS) {
          s.correction *= jtzero::ImuCorrection::kMaxGravityCorrectionRadS / n;
        }
        corrected -= s.correction;
        s.correction_applied = s.correction.norm() > 0.0;
      }
    }

    const Eigen::Vector3d theta = -corrected * s.dt;
    const double angle = theta.norm();
    if (angle > 1e-12) {
      gravity_body_ = Eigen::AngleAxisd(angle, theta / angle) * gravity_body_;
      gravity_body_.normalize();
    }

    s.initialized = initialized_;
    return s;
  }

 private:
  bool initialized_ = false;
  bool accel_lp_initialized_ = false;
  uint64_t last_imu_us_ = 0;
  double static_time_sec_ = 0.0;
  Eigen::Vector3d gravity_body_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d accel_lp_ = Eigen::Vector3d::Zero();
};

void addStats(PhaseStats& st, const DiagnosticCorrection::Step& s) {
  ++st.samples;
  if (s.gravity_magnitude_ok) ++st.gravity_mag_ok;
  if (s.gyro_quiet) ++st.gyro_quiet;
  if (s.accel_quiet) ++st.accel_quiet;
  if (s.static_sample) ++st.static_sample;
  if (s.static_confirmed) ++st.static_confirmed;
  if (s.correction_applied) {
    ++st.correction_applied;
    st.correction_active_sec += s.dt;
    st.correction_integral_rad += s.correction * s.dt;
    st.max_correction_rad_s = std::max(st.max_correction_rad_s, s.correction.norm());
  }
}

void printStats(const char* name, const PhaseStats& s) {
  std::cout << name << ": samples=" << s.samples
            << " static_sample=" << s.static_sample
            << " static_confirmed=" << s.static_confirmed
            << " correction_applied=" << s.correction_applied
            << " correction_active_s=" << std::fixed << std::setprecision(3)
            << s.correction_active_sec << "\n";
  std::cout << "  correction integral deg XYZ: ["
            << s.correction_integral_rad.x() * kRadToDeg << " "
            << s.correction_integral_rad.y() * kRadToDeg << " "
            << s.correction_integral_rad.z() * kRadToDeg << "]\n";
  std::cout << "  max correction deg/s: " << s.max_correction_rad_s * kRadToDeg << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string input = argc > 1 ? argv[1] : "/home/vio/jtzero_yaw_only_v13.csv";
    const std::string output = argc > 2 ? argv[2] : "/home/vio/jtzero_gravity_timing_v15_7.csv";
    const auto imu = loadImu(input);

    size_t first_yaw = imu.size();
    size_t last_yaw = 0;
    for (size_t i = 0; i < imu.size(); ++i) {
      const Eigen::Vector3d g = jtzero::ImuCorrection::gyroFrdToFlu(imu[i].gx, imu[i].gy, imu[i].gz);
      const Eigen::Vector3d zxy = jtzero::ImuCorrection::applyZxy(g);
      if (std::abs(zxy.z()) >= kYawActiveRadS) {
        if (first_yaw == imu.size()) first_yaw = i;
        last_yaw = i;
      }
    }
    if (first_yaw == imu.size()) throw std::runtime_error("No yaw interval found with |gz| >= 5 deg/s");

    const uint64_t t0_us = static_cast<uint64_t>(imu.front().source_ns / 1000LL);
    const uint64_t yaw_start_us = static_cast<uint64_t>(imu[first_yaw].source_ns / 1000LL);
    const uint64_t yaw_end_us = static_cast<uint64_t>(imu[last_yaw].source_ns / 1000LL);

    std::ofstream csv(output);
    if (!csv) throw std::runtime_error("Cannot open output CSV: " + output);
    csv << "t_s,phase,acc_norm,gyro_zxy_norm_deg_s,gyro_z_deg_s,accel_residual,gravity_mag_ok,gyro_quiet,accel_quiet,static_sample,static_confirmed,initialized,correction_applied,corr_x_deg_s,corr_y_deg_s,corr_z_deg_s\n";

    DiagnosticCorrection corr;
    PhaseStats pre, yaw, post;
    bool seen_first_correction = false;
    double first_correction_t = std::numeric_limits<double>::quiet_NaN();
    double first_post_correction_t = std::numeric_limits<double>::quiet_NaN();
    double last_correction_t = std::numeric_limits<double>::quiet_NaN();

    for (const auto& r : imu) {
      const uint64_t us = static_cast<uint64_t>(r.source_ns / 1000LL);
      const Eigen::Vector3d acc = jtzero::ImuCorrection::accelFrdToFlu(r.ax, r.ay, r.az);
      const Eigen::Vector3d gyro = jtzero::ImuCorrection::gyroFrdToFlu(r.gx, r.gy, r.gz);
      const auto s = corr.process(us, acc, gyro);
      const Phase phase = us < yaw_start_us ? Phase::PRE : (us <= yaw_end_us ? Phase::YAW : Phase::POST);
      PhaseStats* st = phase == Phase::PRE ? &pre : (phase == Phase::YAW ? &yaw : &post);
      addStats(*st, s);

      const double t = static_cast<double>(us - t0_us) * 1e-6;
      if (s.correction_applied) {
        if (!seen_first_correction) {
          first_correction_t = t;
          seen_first_correction = true;
        }
        if (phase == Phase::POST && std::isnan(first_post_correction_t)) first_post_correction_t = t;
        last_correction_t = t;
      }

      csv << std::fixed << std::setprecision(9)
          << t << ',' << phaseName(phase) << ','
          << s.acc_norm << ',' << s.gyro_zxy.norm() * kRadToDeg << ','
          << s.gyro_zxy.z() * kRadToDeg << ',' << s.accel_residual << ','
          << s.gravity_magnitude_ok << ',' << s.gyro_quiet << ',' << s.accel_quiet << ','
          << s.static_sample << ',' << s.static_confirmed << ',' << s.initialized << ','
          << s.correction_applied << ','
          << s.correction.x() * kRadToDeg << ','
          << s.correction.y() * kRadToDeg << ','
          << s.correction.z() * kRadToDeg << '\n';
    }

    const double yaw_start_t = static_cast<double>(yaw_start_us - t0_us) * 1e-6;
    const double yaw_end_t = static_cast<double>(yaw_end_us - t0_us) * 1e-6;

    std::cout << "============================================================\n";
    std::cout << "JT-ZERO GRAVITY FEEDBACK TIMING v15.7\n";
    std::cout << "============================================================\n";
    std::cout << "input: " << input << "\n";
    std::cout << "IMU rows: " << imu.size() << "\n";
    std::cout << "yaw detector: |ZXY gyro Z| >= 5 deg/s\n";
    std::cout << std::fixed << std::setprecision(3)
              << "yaw interval: " << yaw_start_t << " .. " << yaw_end_t
              << " s (" << yaw_end_t - yaw_start_t << " s)\n";
    std::cout << "static threshold: gyro <= "
              << jtzero::ImuCorrection::kStaticGyroMaxRadS * kRadToDeg << " deg/s, hold >= "
              << jtzero::ImuCorrection::kStaticHoldSec << " s\n\n";

    printStats("PRE ", pre);
    printStats("YAW ", yaw);
    printStats("POST", post);

    std::cout << "\nfirst correction t_s: " << first_correction_t << "\n";
    std::cout << "first POST correction t_s: " << first_post_correction_t << "\n";
    if (!std::isnan(first_post_correction_t)) {
      std::cout << "delay yaw_end -> first POST correction: "
                << first_post_correction_t - yaw_end_t << " s\n";
    }
    std::cout << "last correction t_s: " << last_correction_t << "\n";
    std::cout << "CSV: " << output << "\n";
    std::cout << "RESULT: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
