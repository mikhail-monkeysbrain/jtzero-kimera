// JT-ZERO Stage 11 gravity PRE-event diagnostic v15.11.
// Offline only. Replays the production gravity-feedback logic on recorded IMU,
// prints every PRE-yaw correction event at high precision, and associates it
// with the nearest backend states from a CURRENT replay CSV.

#include "jtzero_imu_correction.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
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

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : line) {
    if (ch == ',') { out.push_back(cur); cur.clear(); }
    else cur.push_back(ch);
  }
  out.push_back(cur);
  return out;
}

long long toI64(const std::string& s) { return s.empty() ? 0LL : std::stoll(s); }
double toDouble(const std::string& s) { return s.empty() ? 0.0 : std::stod(s); }

struct ImuRow {
  int64_t source_ns = 0;
  int64_t mapped_ns = 0;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
};

struct StateRow {
  long long keyframe = 0;
  int64_t timestamp_ns = 0;
  double px = 0, py = 0, pz = 0;
  double roll = 0, pitch = 0, yaw = 0;
};

std::vector<ImuRow> loadImu(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open IMU CSV: " + path);
  std::vector<ImuRow> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    auto c = splitCsv(line);
    if (c.size() < 14 || c[0] != "IMU") continue;
    ImuRow r;
    r.source_ns = toI64(c[2]);
    r.mapped_ns = toI64(c[3]);
    r.ax = toDouble(c[8]); r.ay = toDouble(c[9]); r.az = toDouble(c[10]);
    r.gx = toDouble(c[11]); r.gy = toDouble(c[12]); r.gz = toDouble(c[13]);
    if (r.source_ns > 0 && r.mapped_ns > 0) out.push_back(r);
  }
  if (out.empty()) throw std::runtime_error("No IMU rows found");
  return out;
}

std::vector<StateRow> loadStates(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open state CSV: " + path);
  std::vector<StateRow> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    auto c = splitCsv(line);
    if (c.size() < 11) continue;
    StateRow s;
    s.keyframe = toI64(c[0]);
    s.timestamp_ns = toI64(c[1]);
    s.px = toDouble(c[2]); s.py = toDouble(c[3]); s.pz = toDouble(c[4]);
    s.roll = toDouble(c[8]); s.pitch = toDouble(c[9]); s.yaw = toDouble(c[10]);
    out.push_back(s);
  }
  if (out.empty()) throw std::runtime_error("No backend states found");
  return out;
}

class DiagnosticCorrection {
 public:
  struct Step {
    double dt = 0.0;
    Eigen::Vector3d gyro_in = Eigen::Vector3d::Zero();
    Eigen::Vector3d corrected = Eigen::Vector3d::Zero();
    Eigen::Vector3d correction = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_body_before = Eigen::Vector3d::Zero();
    Eigen::Vector3d measured_gravity = Eigen::Vector3d::Zero();
    bool static_confirmed = false;
    bool initialized = false;
    bool correction_applied = false;
  };

  Step process(uint64_t imu_us,
               const Eigen::Vector3d& accel_flu,
               const Eigen::Vector3d& gyro_flu) {
    Step s;
    s.gyro_in = jtzero::ImuCorrection::applyZxy(gyro_flu);
    s.corrected = s.gyro_in;

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

    const double acc_norm = accel_flu.norm();
    const bool gravity_magnitude_ok =
        acc_norm > 1e-6 &&
        std::abs(acc_norm - jtzero::ImuCorrection::kGravityMps2) <=
            jtzero::ImuCorrection::kGravityAccTolMps2;
    const bool gyro_quiet = s.gyro_in.norm() <= jtzero::ImuCorrection::kStaticGyroMaxRadS;
    const bool accel_quiet =
        (accel_flu - accel_lp_).norm() <= jtzero::ImuCorrection::kStaticAccelResidualMaxMps2;
    const bool static_sample = gravity_magnitude_ok && gyro_quiet && accel_quiet;

    if (static_sample) static_time_sec_ += s.dt;
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

    s.gravity_body_before = gravity_body_;
    if (s.static_confirmed) {
      s.measured_gravity = accel_lp_.normalized();
      s.gravity_error = gravity_body_.cross(s.measured_gravity);
      const double error_norm = s.gravity_error.norm();
      if (error_norm <= std::sin(jtzero::ImuCorrection::kMaxGravityErrorRad)) {
        s.correction = jtzero::ImuCorrection::kGravityKp * s.gravity_error;
        const double n = s.correction.norm();
        if (n > jtzero::ImuCorrection::kMaxGravityCorrectionRadS) {
          s.correction *= jtzero::ImuCorrection::kMaxGravityCorrectionRadS / n;
        }
        s.corrected -= s.correction;
        s.correction_applied = s.correction.norm() > 0.0;
      }
    }

    const Eigen::Vector3d theta = -s.corrected * s.dt;
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
  Eigen::Vector3d gravity_body_ = Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d accel_lp_ = Eigen::Vector3d::Zero();
};

struct NeighborStates {
  const StateRow* before = nullptr;
  const StateRow* after = nullptr;
};

NeighborStates neighbors(const std::vector<StateRow>& s, int64_t ts) {
  auto it = std::lower_bound(s.begin(), s.end(), ts,
      [](const StateRow& a, int64_t t) { return a.timestamp_ns < t; });
  NeighborStates n;
  if (it != s.end()) n.after = &*it;
  if (it != s.begin()) { auto p = it; --p; n.before = &*p; }
  if (it != s.end() && it->timestamp_ns == ts) n.before = &*it;
  return n;
}

void printState(const char* label, const StateRow* s, int64_t event_ts) {
  if (!s) {
    std::cout << "  " << label << ": NONE\n";
    return;
  }
  const double dt = static_cast<double>(s->timestamp_ns - event_ts) * 1e-9;
  std::cout << "  " << label << ": kf=" << s->keyframe
            << " dt_event=" << std::showpos << std::fixed << std::setprecision(6) << dt
            << std::noshowpos << " s P_mm=["
            << s->px * 1000.0 << ' ' << s->py * 1000.0 << ' ' << s->pz * 1000.0
            << "] RPY_deg=[" << s->roll << ' ' << s->pitch << ' ' << s->yaw << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string imu_path = argc > 1 ? argv[1] : "/home/vio/jtzero_yaw_only_v13.csv";
    const std::string state_path = argc > 2 ? argv[2] : "/home/vio/jtzero_gravity_v15_8_CURRENT.csv";
    const std::string out_path = argc > 3 ? argv[3] : "/home/vio/jtzero_gravity_pre_events_v15_11.csv";

    const auto imu = loadImu(imu_path);
    const auto states = loadStates(state_path);

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
    if (first_yaw == imu.size()) throw std::runtime_error("No yaw interval found");

    const int64_t t0_mapped_ns = imu.front().mapped_ns;
    const int64_t yaw_start_mapped_ns = imu[first_yaw].mapped_ns;
    const int64_t yaw_end_mapped_ns = imu[last_yaw].mapped_ns;

    std::ofstream csv(out_path);
    if (!csv) throw std::runtime_error("Cannot create output CSV: " + out_path);
    csv << "event,mapped_t_s,source_t_s,dt_s,corr_x_deg_s,corr_y_deg_s,corr_z_deg_s,"
           "corr_dt_x_deg,corr_dt_y_deg,corr_dt_z_deg,cum_x_deg,cum_y_deg,cum_z_deg,"
           "gyro_in_x_deg_s,gyro_in_y_deg_s,gyro_in_z_deg_s,"
           "gyro_out_x_deg_s,gyro_out_y_deg_s,gyro_out_z_deg_s,"
           "gravity_error_x,gravity_error_y,gravity_error_z,"
           "gravity_body_x,gravity_body_y,gravity_body_z,"
           "measured_gravity_x,measured_gravity_y,measured_gravity_z\n";

    DiagnosticCorrection corr;
    Eigen::Vector3d cumulative_rad = Eigen::Vector3d::Zero();
    size_t event = 0;
    std::cout << "============================================================\n";
    std::cout << "JT-ZERO PRE GRAVITY CORRECTION EVENTS v15.11\n";
    std::cout << "============================================================\n";
    std::cout << "IMU rows: " << imu.size() << "\nbackend states: " << states.size() << "\n";
    std::cout << std::fixed << std::setprecision(6)
              << "yaw mapped interval: "
              << (yaw_start_mapped_ns - t0_mapped_ns) * 1e-9 << " .. "
              << (yaw_end_mapped_ns - t0_mapped_ns) * 1e-9 << " s\n\n";

    const int64_t source0 = imu.front().source_ns;
    for (const auto& r : imu) {
      const uint64_t us = static_cast<uint64_t>(r.source_ns / 1000LL);
      const Eigen::Vector3d acc = jtzero::ImuCorrection::accelFrdToFlu(r.ax, r.ay, r.az);
      const Eigen::Vector3d gyro = jtzero::ImuCorrection::gyroFrdToFlu(r.gx, r.gy, r.gz);
      const auto s = corr.process(us, acc, gyro);
      if (r.mapped_ns >= yaw_start_mapped_ns) continue;
      if (!s.correction_applied) continue;

      ++event;
      const Eigen::Vector3d event_rad = s.correction * s.dt;
      cumulative_rad += event_rad;
      const double mt = (r.mapped_ns - t0_mapped_ns) * 1e-9;
      const double st = (r.source_ns - source0) * 1e-9;
      const auto n = neighbors(states, r.mapped_ns);

      std::cout << "EVENT " << event
                << " mapped_t=" << std::setprecision(9) << mt
                << " s source_t=" << st << " s dt=" << s.dt << " s\n";
      std::cout << std::scientific << std::setprecision(12)
                << "  correction deg/s = [" << (s.correction * kRadToDeg).transpose() << "]\n"
                << "  correction*dt deg = [" << (event_rad * kRadToDeg).transpose() << "]\n"
                << "  cumulative deg = [" << (cumulative_rad * kRadToDeg).transpose() << "]\n"
                << "  gyro_in deg/s = [" << (s.gyro_in * kRadToDeg).transpose() << "]\n"
                << "  gyro_out deg/s = [" << (s.corrected * kRadToDeg).transpose() << "]\n"
                << "  gravity_error = [" << s.gravity_error.transpose() << "]\n"
                << "  gravity_body = [" << s.gravity_body_before.transpose() << "]\n"
                << "  measured_gravity = [" << s.measured_gravity.transpose() << "]\n";
      std::cout << std::defaultfloat;
      printState("state before", n.before, r.mapped_ns);
      printState("state after ", n.after, r.mapped_ns);
      std::cout << '\n';

      csv << std::fixed << std::setprecision(12)
          << event << ',' << mt << ',' << st << ',' << s.dt << ','
          << s.correction.x()*kRadToDeg << ',' << s.correction.y()*kRadToDeg << ',' << s.correction.z()*kRadToDeg << ','
          << event_rad.x()*kRadToDeg << ',' << event_rad.y()*kRadToDeg << ',' << event_rad.z()*kRadToDeg << ','
          << cumulative_rad.x()*kRadToDeg << ',' << cumulative_rad.y()*kRadToDeg << ',' << cumulative_rad.z()*kRadToDeg << ','
          << s.gyro_in.x()*kRadToDeg << ',' << s.gyro_in.y()*kRadToDeg << ',' << s.gyro_in.z()*kRadToDeg << ','
          << s.corrected.x()*kRadToDeg << ',' << s.corrected.y()*kRadToDeg << ',' << s.corrected.z()*kRadToDeg << ','
          << s.gravity_error.x() << ',' << s.gravity_error.y() << ',' << s.gravity_error.z() << ','
          << s.gravity_body_before.x() << ',' << s.gravity_body_before.y() << ',' << s.gravity_body_before.z() << ','
          << s.measured_gravity.x() << ',' << s.measured_gravity.y() << ',' << s.measured_gravity.z() << '\n';
    }

    std::cout << "============================================================\n";
    std::cout << "PRE correction events: " << event << "\n";
    std::cout << std::scientific << std::setprecision(12)
              << "PRE cumulative correction deg XYZ: ["
              << (cumulative_rad * kRadToDeg).transpose() << "]\n";
    std::cout << std::defaultfloat << "CSV: " << out_path << "\nRESULT: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
