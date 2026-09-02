// JT-ZERO v38: phase-resolved IMU residual diagnostic without Kimera backend.
// Compares three orientation branches on the same HIGHRES_IMU stream:
//   A) gyro-integrated R/P/Y
//   B) FC ATTITUDE roll/pitch + gyro-integrated yaw
//   C) accelerometer tilt (updated only while still) + gyro-integrated yaw
// Route is event-driven: physical 0 -> right yaw -> physical 0. Russian GUI.
#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>

namespace jtzero_v38 {
using namespace jtzero_v10;

constexpr const char* kCsv38 = "/home/vio/jtzero_live_imu_phase_residual_v38.csv";
constexpr const char* kWindow38 = "JT-ZERO: ФАЗОВЫЙ IMU RESIDUAL v38";
constexpr int kCalSamples38 = 500;
constexpr int kStaticSamples38 = 1000;
constexpr int kStableSamples38 = 120;
constexpr double kGyroStill38 = 0.035;
constexpr double kAccTol38 = 0.45;
constexpr double kG38 = 9.81;

enum class Phase38 {
  WAIT_ZERO = 0,
  CALIBRATE,
  STATIC_0_BEFORE,
  ROTATION,
  STATIC_ROTATED,
  RETURN,
  STATIC_0_AFTER,
  DONE
};

constexpr int kMeasuredPhases38 = 5;
constexpr const char* kPhaseNames38[kMeasuredPhases38] = {
  "STATIC_0_BEFORE", "ROTATION", "STATIC_ROTATED", "RETURN", "STATIC_0_AFTER"
};

struct Att38 {
  bool valid = false;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct Branch38 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d V = Eigen::Vector3d::Zero();
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
};

struct PhaseStats38 {
  int n = 0;
  double dt = 0.0;
  Eigen::Vector3d dv_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d dv_fc = Eigen::Vector3d::Zero();
  Eigen::Vector3d dv_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_a_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_a_fc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_a_acc = Eigen::Vector3d::Zero();
  double max_a_gyro_xy = 0.0;
  double max_a_fc_xy = 0.0;
  double max_a_acc_xy = 0.0;
};

struct Row38 {
  uint64_t us = 0;
  int phase = -1;
  double dt = 0.0;
  double yaw_rel = 0.0;
  double fc_roll = 0.0;
  double fc_pitch = 0.0;
  double acc_roll = 0.0;
  double acc_pitch_fc_sign = 0.0;
  Eigen::Vector3d raw_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_fc = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_fc = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_acc = Eigen::Vector3d::Zero();
};

struct State38 {
  Phase38 phase = Phase38::WAIT_ZERO;
  uint64_t last_us = 0;
  int cal_n = 0;
  int static_n = 0;
  int stable_n = 0;
  Eigen::Vector3d cal_acc_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d cal_gyro_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d BA = Eigen::Vector3d::Zero();
  Eigen::Vector3d BG = Eigen::Vector3d::Zero();
  double yaw_ref = 0.0;
  double yaw_int = 0.0;
  double acc_roll_hold = 0.0;
  double acc_pitch_hold = 0.0;
  bool acc_tilt_valid = false;
  Branch38 gyro;
  Branch38 fc;
  Branch38 acc;
  std::array<PhaseStats38, kMeasuredPhases38> stats;
  std::vector<Row38> rows;
};

static double wrap38(double x) {
  while (x > 180.0) x -= 360.0;
  while (x < -180.0) x += 360.0;
  return x;
}

static bool still38(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
  return gyro.norm() <= kGyroStill38 && std::abs(acc.norm() - kG38) <= kAccTol38;
}

static void accelTilt38(const Eigen::Vector3d& a, double* roll_deg, double* pitch_fc_sign_deg) {
  const Eigen::Vector3d u = a.normalized();
  const double r = std::atan2(u.y(), u.z()) * 180.0 / kPi;
  const double p_raw = std::atan2(-u.x(), std::sqrt(u.y()*u.y() + u.z()*u.z())) * 180.0 / kPi;
  *roll_deg = r;
  *pitch_fc_sign_deg = -p_raw;
}

static int measuredIndex38(Phase38 p) {
  switch (p) {
    case Phase38::STATIC_0_BEFORE: return 0;
    case Phase38::ROTATION: return 1;
    case Phase38::STATIC_ROTATED: return 2;
    case Phase38::RETURN: return 3;
    case Phase38::STATIC_0_AFTER: return 4;
    default: return -1;
  }
}

static const char* phaseLabel38(Phase38 p) {
  switch (p) {
    case Phase38::WAIT_ZERO: return "ОЖИДАНИЕ ФИЗИЧЕСКОГО НУЛЯ";
    case Phase38::CALIBRATE: return "КАЛИБРОВКА BA/BG — НЕ ДВИГАТЬ";
    case Phase38::STATIC_0_BEFORE: return "СТАТИКА В НУЛЕ — НЕ ДВИГАТЬ";
    case Phase38::ROTATION: return "ПЛАВНО ПОВЕРНИТЕ ВПРАВО";
    case Phase38::STATIC_ROTATED: return "СТАТИКА ПОСЛЕ ПОВОРОТА";
    case Phase38::RETURN: return "ВЕРНИТЕ В ФИЗИЧЕСКИЙ НОЛЬ";
    case Phase38::STATIC_0_AFTER: return "ФИНАЛЬНАЯ СТАТИКА В НУЛЕ";
    case Phase38::DONE: return "ТЕСТ ЗАВЕРШЁН";
  }
  return "";
}

static Eigen::Matrix3d RfromFc38(double roll, double pitch, double yaw) {
  return fcRnedFlu(roll, pitch, yaw);
}

static void initializeAfterCalibration38(State38& s, const Att38& att) {
  const Eigen::Vector3d mean_acc = s.cal_acc_sum / double(s.cal_n);
  s.BG = s.cal_gyro_sum / double(s.cal_n);
  s.yaw_int = att.yaw;
  const Eigen::Matrix3d R0 = RfromFc38(att.roll, att.pitch, s.yaw_int);
  const Eigen::Vector3d expected_acc = R0.transpose() * Eigen::Vector3d(0, 0, -kG38);
  s.BA = mean_acc - expected_acc;
  s.gyro.R = R0;
  s.fc.R = R0;
  double ar = 0.0, ap = 0.0;
  accelTilt38(mean_acc, &ar, &ap);
  s.acc_roll_hold = ar;
  s.acc_pitch_hold = ap;
  s.acc_tilt_valid = true;
  s.acc.R = RfromFc38(ar, ap, s.yaw_int);
  s.gyro.V.setZero();
  s.fc.V.setZero();
  s.acc.V.setZero();
  s.last_us = 0;
  s.static_n = 0;
  s.stable_n = 0;
  s.phase = Phase38::STATIC_0_BEFORE;
  std::cout << "[CAL] BA=[" << s.BA.transpose() << "] BG=[" << s.BG.transpose() << "]\n";
}

static void accumulateStats38(PhaseStats38& ps,
                              double dt,
                              const Eigen::Vector3d& ag,
                              const Eigen::Vector3d& af,
                              const Eigen::Vector3d& aa) {
  ++ps.n;
  ps.dt += dt;
  ps.dv_gyro += ag * dt;
  ps.dv_fc += af * dt;
  ps.dv_acc += aa * dt;
  ps.sum_a_gyro += ag;
  ps.sum_a_fc += af;
  ps.sum_a_acc += aa;
  ps.max_a_gyro_xy = std::max(ps.max_a_gyro_xy, std::hypot(ag.x(), ag.y()));
  ps.max_a_fc_xy = std::max(ps.max_a_fc_xy, std::hypot(af.x(), af.y()));
  ps.max_a_acc_xy = std::max(ps.max_a_acc_xy, std::hypot(aa.x(), aa.y()));
}

static void processImu38(State38& s,
                         const Att38& att,
                         uint64_t us,
                         const Eigen::Vector3d& raw_acc,
                         const Eigen::Vector3d& raw_gyro) {
  const bool is_still = still38(raw_acc, raw_gyro);

  if (s.phase == Phase38::CALIBRATE) {
    if (is_still && att.valid) {
      s.cal_acc_sum += raw_acc;
      s.cal_gyro_sum += raw_gyro;
      ++s.cal_n;
      if (s.cal_n >= kCalSamples38) initializeAfterCalibration38(s, att);
    } else {
      s.cal_n = 0;
      s.cal_acc_sum.setZero();
      s.cal_gyro_sum.setZero();
    }
    return;
  }

  if (s.phase == Phase38::WAIT_ZERO || s.phase == Phase38::DONE || !att.valid) return;

  double dt = 0.0;
  if (s.last_us && us > s.last_us) dt = (us - s.last_us) * 1e-6;
  s.last_us = us;
  if (dt <= 0.0 || dt > 0.03) return;

  const Eigen::Vector3d wc = raw_gyro - s.BG;
  const Eigen::Vector3d ac = raw_acc - s.BA;
  const Eigen::Vector3d th = wc * dt;
  const double ang = th.norm();
  if (ang > 1e-12) {
    s.gyro.R = s.gyro.R * Eigen::AngleAxisd(ang, th / ang).toRotationMatrix();
  }
  s.yaw_int += wc.z() * dt * 180.0 / kPi;

  s.fc.R = RfromFc38(att.roll, att.pitch, s.yaw_int);

  if (is_still) {
    double ar = 0.0, ap = 0.0;
    accelTilt38(raw_acc, &ar, &ap);
    constexpr double alpha = 0.02;
    if (!s.acc_tilt_valid) {
      s.acc_roll_hold = ar;
      s.acc_pitch_hold = ap;
      s.acc_tilt_valid = true;
    } else {
      s.acc_roll_hold = (1.0 - alpha) * s.acc_roll_hold + alpha * ar;
      s.acc_pitch_hold = (1.0 - alpha) * s.acc_pitch_hold + alpha * ap;
    }
  }
  s.acc.R = RfromFc38(s.acc_roll_hold, s.acc_pitch_hold, s.yaw_int);

  const Eigen::Vector3d gN(0, 0, kG38);
  const Eigen::Vector3d ag = s.gyro.R * ac + gN;
  const Eigen::Vector3d af = s.fc.R * ac + gN;
  const Eigen::Vector3d aa = s.acc.R * ac + gN;
  s.gyro.A = ag;
  s.fc.A = af;
  s.acc.A = aa;
  s.gyro.V += ag * dt;
  s.fc.V += af * dt;
  s.acc.V += aa * dt;

  const int mi = measuredIndex38(s.phase);
  if (mi >= 0) accumulateStats38(s.stats[mi], dt, ag, af, aa);

  Row38 r;
  r.us = us;
  r.phase = mi;
  r.dt = dt;
  r.yaw_rel = wrap38(s.yaw_int - s.yaw_ref);
  r.fc_roll = att.roll;
  r.fc_pitch = att.pitch;
  r.acc_roll = s.acc_roll_hold;
  r.acc_pitch_fc_sign = s.acc_pitch_hold;
  r.raw_acc = raw_acc;
  r.raw_gyro = raw_gyro;
  r.a_gyro = ag;
  r.a_fc = af;
  r.a_acc = aa;
  r.v_gyro = s.gyro.V;
  r.v_fc = s.fc.V;
  r.v_acc = s.acc.V;
  s.rows.push_back(r);

  if (is_still) ++s.stable_n; else s.stable_n = 0;

  if (s.phase == Phase38::STATIC_0_BEFORE ||
      s.phase == Phase38::STATIC_ROTATED ||
      s.phase == Phase38::STATIC_0_AFTER) {
    if (is_still) ++s.static_n; else s.static_n = 0;
    if (s.static_n >= kStaticSamples38) {
      s.static_n = 0;
      s.stable_n = 0;
      if (s.phase == Phase38::STATIC_0_BEFORE) {
        s.phase = Phase38::ROTATION;
        std::cout << "[STEP] baseline static complete. Rotate right, stop, then press SPACE.\n";
      } else if (s.phase == Phase38::STATIC_ROTATED) {
        s.phase = Phase38::RETURN;
        std::cout << "[STEP] rotated static complete. Return to physical zero, stop, then press SPACE.\n";
      } else {
        s.phase = Phase38::DONE;
        std::cout << "[TEST] final static complete.\n";
      }
    }
  }
}

static void save38(const State38& s) {
  std::ofstream f(kCsv38, std::ios::trunc);
  f << std::fixed << std::setprecision(9)
    << "imu_us,phase,phase_name,dt,yaw_rel_deg,fc_roll_deg,fc_pitch_deg,acc_roll_deg,acc_pitch_fc_sign_deg,"
       "ax,ay,az,gx,gy,gz,a_gyro_x,a_gyro_y,a_gyro_z,a_fc_x,a_fc_y,a_fc_z,a_acc_x,a_acc_y,a_acc_z,"
       "v_gyro_x,v_gyro_y,v_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_acc_x,v_acc_y,v_acc_z\n";
  for (const auto& r : s.rows) {
    const char* name = (r.phase >= 0 && r.phase < kMeasuredPhases38) ? kPhaseNames38[r.phase] : "OTHER";
    f << r.us << ',' << r.phase << ',' << name << ',' << r.dt << ',' << r.yaw_rel << ','
      << r.fc_roll << ',' << r.fc_pitch << ',' << r.acc_roll << ',' << r.acc_pitch_fc_sign << ','
      << r.raw_acc.x() << ',' << r.raw_acc.y() << ',' << r.raw_acc.z() << ','
      << r.raw_gyro.x() << ',' << r.raw_gyro.y() << ',' << r.raw_gyro.z() << ','
      << r.a_gyro.x() << ',' << r.a_gyro.y() << ',' << r.a_gyro.z() << ','
      << r.a_fc.x() << ',' << r.a_fc.y() << ',' << r.a_fc.z() << ','
      << r.a_acc.x() << ',' << r.a_acc.y() << ',' << r.a_acc.z() << ','
      << r.v_gyro.x() << ',' << r.v_gyro.y() << ',' << r.v_gyro.z() << ','
      << r.v_fc.x() << ',' << r.v_fc.y() << ',' << r.v_fc.z() << ','
      << r.v_acc.x() << ',' << r.v_acc.y() << ',' << r.v_acc.z() << '\n';
  }
}

static void printSummary38(const State38& s) {
  std::cout << std::fixed << std::setprecision(6)
            << "\n============================================================\n"
            << "JT-ZERO IMU PHASE RESIDUAL v38 RESULT\n"
            << "============================================================\n"
            << "BA=[" << s.BA.transpose() << "] m/s^2\n"
            << "BG=[" << s.BG.transpose() << "] rad/s\n";

  for (int i = 0; i < kMeasuredPhases38; ++i) {
    const auto& p = s.stats[i];
    Eigen::Vector3d mag = Eigen::Vector3d::Zero();
    Eigen::Vector3d maf = Eigen::Vector3d::Zero();
    Eigen::Vector3d maa = Eigen::Vector3d::Zero();
    if (p.n > 0) {
      mag = p.sum_a_gyro / double(p.n);
      maf = p.sum_a_fc / double(p.n);
      maa = p.sum_a_acc / double(p.n);
    }
    std::cout << "\n" << kPhaseNames38[i] << " n=" << p.n << " dt=" << p.dt << " s\n"
              << " GYRO mean axy=" << std::hypot(mag.x(), mag.y())
              << " max axy=" << p.max_a_gyro_xy
              << " dVxy=" << std::hypot(p.dv_gyro.x(), p.dv_gyro.y()) << " m/s\n"
              << " FC_RP mean axy=" << std::hypot(maf.x(), maf.y())
              << " max axy=" << p.max_a_fc_xy
              << " dVxy=" << std::hypot(p.dv_fc.x(), p.dv_fc.y()) << " m/s\n"
              << " ACC_RP mean axy=" << std::hypot(maa.x(), maa.y())
              << " max axy=" << p.max_a_acc_xy
              << " dVxy=" << std::hypot(p.dv_acc.x(), p.dv_acc.y()) << " m/s\n";
  }

  std::cout << "\nFINAL integrated Vxy:\n"
            << " GYRO=" << std::hypot(s.gyro.V.x(), s.gyro.V.y()) << " m/s\n"
            << " FC_RP=" << std::hypot(s.fc.V.x(), s.fc.V.y()) << " m/s\n"
            << " ACC_RP=" << std::hypot(s.acc.V.x(), s.acc.V.y()) << " m/s\n"
            << "CSV: " << kCsv38 << "\n"
            << "Open CSV:\n  code " << kCsv38 << "\n";
}

static void hud38(const State38& s,
                  const Att38& att,
                  const Eigen::Vector3d& raw_acc,
                  const Eigen::Vector3d& raw_gyro) {
  cv::Mat c(850, 1300, CV_8UC3, cv::Scalar(15,15,15));
  const cv::Scalar white(235,235,235), green(80,220,80), yellow(0,220,255), red(80,80,255);
  uiText(c, "JT-ZERO: ФАЗОВЫЙ IMU RESIDUAL v38", 30, 52, .72, white, 2);
  uiText(c, phaseLabel38(s.phase), 55, 120, .64,
         s.phase == Phase38::DONE ? green : yellow, 2);

  if (s.phase == Phase38::WAIT_ZERO) {
    uiText(c, "ПОСТАВЬТЕ СТЕНД В ФИЗИЧЕСКИЙ 0°", 55, 175, .55, white, 2);
    uiText(c, "SPACE — НАЧАТЬ", 55, 225, .55, green, 2);
  } else if (s.phase == Phase38::ROTATION) {
    uiText(c, "ПОВОРОТ 60–100°; ОСТАНОВИТЕСЬ", 55, 175, .52, white, 2);
    uiText(c, "КОГДА СТАБИЛЬНО — SPACE", 55, 225, .52, green, 2);
  } else if (s.phase == Phase38::RETURN) {
    uiText(c, "ВЕРНИТЕ ТОЧНО НА ИСХОДНУЮ МЕТКУ 0°", 55, 175, .50, white, 2);
    uiText(c, "КОГДА СТАБИЛЬНО — SPACE", 55, 225, .52, green, 2);
  } else if (s.phase == Phase38::DONE) {
    uiText(c, "SPACE — ВЫХОД", 55, 175, .52, white, 2);
  } else {
    uiText(c, "НЕ ДВИГАЙТЕ СТЕНД", 55, 175, .58, red, 2);
  }

  char b[256];
  snprintf(b, sizeof(b), "|gyro| %.4f rad/s    |acc| %.4f m/s²", raw_gyro.norm(), raw_acc.norm());
  uiText(c, b, 55, 315, .46, white, 1);
  snprintf(b, sizeof(b), "Стабильность: %d/%d", s.stable_n, kStableSamples38);
  uiText(c, b, 55, 360, .46, white, 1);
  if (s.phase == Phase38::CALIBRATE) {
    snprintf(b, sizeof(b), "Калибровка: %d/%d", s.cal_n, kCalSamples38);
    uiText(c, b, 55, 405, .46, white, 1);
  }
  if (s.phase == Phase38::STATIC_0_BEFORE || s.phase == Phase38::STATIC_ROTATED || s.phase == Phase38::STATIC_0_AFTER) {
    snprintf(b, sizeof(b), "Статический сбор: %d/%d", s.static_n, kStaticSamples38);
    uiText(c, b, 55, 405, .46, white, 1);
  }

  const double yaw_rel = wrap38(s.yaw_int - s.yaw_ref);
  snprintf(b, sizeof(b), "YAW gyro от старта: %+.2f°", yaw_rel);
  uiText(c, b, 55, 470, .48, white, 1);
  snprintf(b, sizeof(b), "FC R/P: %+.2f° / %+.2f°", att.roll, att.pitch);
  uiText(c, b, 55, 515, .48, white, 1);
  snprintf(b, sizeof(b), "ACC R/P: %+.2f° / %+.2f°", s.acc_roll_hold, s.acc_pitch_hold);
  uiText(c, b, 55, 560, .48, white, 1);

  snprintf(b, sizeof(b), "GYRO: axy %.3f  Vxy %.3f", std::hypot(s.gyro.A.x(), s.gyro.A.y()), std::hypot(s.gyro.V.x(), s.gyro.V.y()));
  uiText(c, b, 55, 640, .52, yellow, 2);
  snprintf(b, sizeof(b), "FC_RP: axy %.3f  Vxy %.3f", std::hypot(s.fc.A.x(), s.fc.A.y()), std::hypot(s.fc.V.x(), s.fc.V.y()));
  uiText(c, b, 55, 695, .52, green, 2);
  snprintf(b, sizeof(b), "ACC_RP: axy %.3f  Vxy %.3f", std::hypot(s.acc.A.x(), s.acc.A.y()), std::hypot(s.acc.V.x(), s.acc.V.y()));
  uiText(c, b, 55, 750, .52, white, 2);

  uiText(c, "ESC/Q — прервать", 940, 815, .40, white, 1);
  cv::imshow(kWindow38, c);
}

}  // namespace jtzero_v38

int main(int argc, char** argv) {
  using namespace jtzero_v38;
  google::InitGoogleLogging(argv[0]);

  int fd = -1;
  uint8_t sys = 0, comp = 0;
  mavlink_status_t mst{};
  mavlink_message_t msg{};
  Att38 att;
  Eigen::Vector3d raw_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_gyro = Eigen::Vector3d::Zero();
  State38 s;
  uint64_t last_msg_us = 0;

  try {
    fd = openSerial();
    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    const int64_t deadline = monotonicNs() + 10000000000LL;
    while (monotonicNs() < deadline && !sys) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[2048];
      const ssize_t n = read(fd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &mst) && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          sys = msg.sysid;
          comp = msg.compid;
          break;
        }
      }
    }
    if (!sys) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 100);
    requestRate(fd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 200);
    cv::setNumThreads(1);
    cv::namedWindow(kWindow38, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindow38, 1300, 850);

    std::cout << "\nJT-ZERO IMU PHASE RESIDUAL v38\n"
              << "Physical 0 -> right yaw -> same physical 0. Follow Russian GUI.\n";

    int64_t next_hud = 0;
    bool aborted = false;
    while (true) {
      pollfd p{fd, POLLIN, 0};
      poll(&p, 1, 5);
      if (p.revents & POLLIN) {
        uint8_t b[8192];
        const ssize_t n = read(fd, b, sizeof(b));
        if (n > 0) {
          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &mst)) continue;
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(&msg, &a);
              att.valid = true;
              att.roll = a.roll * 180.0 / kPi;
              att.pitch = a.pitch * 180.0 / kPi;
              att.yaw = a.yaw * 180.0 / kPi;
            } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg, &h);
              if (h.time_usec == last_msg_us) continue;
              last_msg_us = h.time_usec;
              raw_acc = Eigen::Vector3d(h.xacc, -h.yacc, -h.zacc);
              raw_gyro = Eigen::Vector3d(h.xgyro, -h.ygyro, -h.zgyro);
              processImu38(s, att, h.time_usec, raw_acc, raw_gyro);
            }
          }
        }
      }

      if (monotonicNs() >= next_hud) {
        hud38(s, att, raw_acc, raw_gyro);
        next_hud = monotonicNs() + 33333333LL;
      }

      const int key = cv::waitKey(1) & 255;
      if (key == 27 || key == 'q' || key == 'Q') {
        aborted = true;
        break;
      }
      if (key == ' ') {
        if (s.phase == Phase38::WAIT_ZERO && att.valid) {
          s.phase = Phase38::CALIBRATE;
          s.yaw_ref = att.yaw;
          s.cal_n = 0;
          s.cal_acc_sum.setZero();
          s.cal_gyro_sum.setZero();
          std::cout << "[ZERO] physical zero confirmed. Calibrating BA/BG.\n";
        } else if (s.phase == Phase38::ROTATION && s.stable_n >= kStableSamples38) {
          s.phase = Phase38::STATIC_ROTATED;
          s.static_n = 0;
          s.stable_n = 0;
          std::cout << "[CONFIRM] rotated position confirmed. Static collection started.\n";
        } else if (s.phase == Phase38::RETURN && s.stable_n >= kStableSamples38) {
          s.phase = Phase38::STATIC_0_AFTER;
          s.static_n = 0;
          s.stable_n = 0;
          std::cout << "[CONFIRM] physical zero return confirmed. Final static collection started.\n";
        } else if (s.phase == Phase38::DONE) {
          break;
        }
      }
    }

    save38(s);
    printSummary38(s);
    std::cout << "aborted: " << (aborted ? "yes" : "no") << "\n";

    requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    requestRate(fd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
    close(fd);
    cv::destroyAllWindows();
    return aborted ? 2 : 0;
  } catch (const std::exception& e) {
    if (fd >= 0) close(fd);
    cv::destroyAllWindows();
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
