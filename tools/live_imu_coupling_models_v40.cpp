// JT-ZERO v40: multi-speed gyro coupling/model diagnostic.
// No Kimera backend. HIGHRES_IMU + ATTITUDE only.
// Russian event-driven GUI. Mechanical route:
// 0 -> slow right -> 0 -> slow left -> 0 -> fast right -> 0 -> fast left -> 0.
// Compares several gyro preprocessing models in parallel and checks whether
// yaw-induced false tilt/velocity is explained by a fixed sensor/body axis mismatch.
#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>
#include <limits>

namespace jtzero_v40 {
using namespace jtzero_v10;

constexpr const char* kCsv40 = "/home/vio/jtzero_live_imu_coupling_models_v40.csv";
constexpr const char* kWindow40 = "JT-ZERO: МОДЕЛИ КОРРЕКЦИИ IMU v40";
constexpr double kG40 = 9.81;
constexpr double kGyroStill40 = 0.035;
constexpr double kAccTol40 = 0.45;
constexpr int kCalN40 = 600;
constexpr int kStableN40 = 120;
constexpr int kStaticN40 = 600;

// Mean dynamic coupling measured by v39.
constexpr double kV39Kxz40 = -0.5 * (0.010577 + 0.009283);
constexpr double kV39Kyz40 =  0.5 * (0.031466 + 0.029970);

enum class Phase40 {
  WAIT_ZERO = 0,
  CALIBRATE,
  STATIC_0_A,
  SLOW_R,
  STATIC_SR,
  RETURN_SR,
  STATIC_0_B,
  SLOW_L,
  STATIC_SL,
  RETURN_SL,
  STATIC_0_C,
  FAST_R,
  STATIC_FR,
  RETURN_FR,
  STATIC_0_D,
  FAST_L,
  STATIC_FL,
  RETURN_FL,
  STATIC_0_E,
  DONE
};

constexpr int kMeasured40 = 17;
constexpr const char* kPhaseNames40[kMeasured40] = {
  "STATIC_0_A", "SLOW_R", "STATIC_SR", "RETURN_SR", "STATIC_0_B",
  "SLOW_L", "STATIC_SL", "RETURN_SL", "STATIC_0_C",
  "FAST_R", "STATIC_FR", "RETURN_FR", "STATIC_0_D",
  "FAST_L", "STATIC_FL", "RETURN_FL", "STATIC_0_E"
};

enum class Model40 {
  RAW = 0,
  V39_TO_GEOM,
  Y_TO_GEOM,
  GEOM_AXIS,
  FC_RATES,
  COUNT
};

constexpr int kModels40 = static_cast<int>(Model40::COUNT);
constexpr const char* kModelNames40[kModels40] = {
  "RAW",
  "V39_TO_GEOM",
  "Y_TO_GEOM",
  "GEOM_AXIS",
  "FC_RATES"
};

struct Att40 {
  bool valid = false;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double rs = 0.0;
  double ps = 0.0;
  double ys = 0.0;
};

struct LinReg40 {
  int n = 0;
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;

  void add(double x, double y) {
    ++n;
    sx += x;
    sy += y;
    sxx += x * x;
    syy += y * y;
    sxy += x * y;
  }

  double slope() const {
    const double d = n * sxx - sx * sx;
    if (std::abs(d) <= 1e-18) return 0.0;
    return (n * sxy - sx * sy) / d;
  }

  double corr() const {
    const double dx = n * sxx - sx * sx;
    const double dy = n * syy - sy * sy;
    const double d = dx * dy;
    if (d <= 1e-24) return 0.0;
    return (n * sxy - sx * sy) / std::sqrt(d);
  }
};

struct ModelState40 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d V = Eigen::Vector3d::Zero();
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
};

struct PhaseStats40 {
  int n = 0;
  double dt = 0.0;
  double max_dt = 0.0;
  int bad_dt = 0;
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  LinReg40 gx_gz;
  LinReg40 gy_gz;
  std::array<Eigen::Vector3d, kModels40> dv{};
  std::array<double, kModels40> sum_axy{};
  std::array<double, kModels40> max_axy{};
  std::array<double, kModels40> sum_grav_err{};
  std::array<double, kModels40> max_grav_err{};

  PhaseStats40() {
    for (auto& x : dv) x.setZero();
  }
};

struct Row40 {
  uint64_t us = 0;
  int phase = -1;
  double dt = 0.0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d fc_rates = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, kModels40> w_model{};
  std::array<Eigen::Vector3d, kModels40> a_model{};
  std::array<Eigen::Vector3d, kModels40> v_model{};
  std::array<double, kModels40> grav_err{};
};

struct State40 {
  Phase40 phase = Phase40::WAIT_ZERO;
  uint64_t last_us = 0;
  int cal_n = 0;
  int stable_n = 0;
  int static_n = 0;
  Eigen::Vector3d cal_acc_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d cal_gyro_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d BA = Eigen::Vector3d::Zero();
  Eigen::Vector3d BG = Eigen::Vector3d::Zero();
  Eigen::Vector3d u0 = Eigen::Vector3d(0, 0, 1);
  double kx_geom = 0.0;
  double ky_geom = 0.0;
  std::array<ModelState40, kModels40> model;
  std::array<PhaseStats40, kMeasured40> stats;
  std::vector<Row40> rows;
};

static double clamp40(double x) {
  return std::max(-1.0, std::min(1.0, x));
}

static bool still40(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
  return gyro.norm() <= kGyroStill40 && std::abs(acc.norm() - kG40) <= kAccTol40;
}

static Eigen::Matrix3d expR40(const Eigen::Vector3d& th) {
  const double q = th.norm();
  if (q <= 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(q, th / q).toRotationMatrix();
}

static double gravityErr40(const Eigen::Matrix3d& R, const Eigen::Vector3d& acc) {
  if (acc.norm() <= 1e-9) return 0.0;
  const Eigen::Vector3d measured = acc.normalized();
  const Eigen::Vector3d predicted = R.transpose() * Eigen::Vector3d(0, 0, 1);
  return std::acos(clamp40(measured.dot(predicted))) * 180.0 / kPi;
}

static int measuredIndex40(Phase40 p) {
  switch (p) {
    case Phase40::STATIC_0_A: return 0;
    case Phase40::SLOW_R: return 1;
    case Phase40::STATIC_SR: return 2;
    case Phase40::RETURN_SR: return 3;
    case Phase40::STATIC_0_B: return 4;
    case Phase40::SLOW_L: return 5;
    case Phase40::STATIC_SL: return 6;
    case Phase40::RETURN_SL: return 7;
    case Phase40::STATIC_0_C: return 8;
    case Phase40::FAST_R: return 9;
    case Phase40::STATIC_FR: return 10;
    case Phase40::RETURN_FR: return 11;
    case Phase40::STATIC_0_D: return 12;
    case Phase40::FAST_L: return 13;
    case Phase40::STATIC_FL: return 14;
    case Phase40::RETURN_FL: return 15;
    case Phase40::STATIC_0_E: return 16;
    default: return -1;
  }
}

static bool staticPhase40(Phase40 p) {
  switch (p) {
    case Phase40::STATIC_0_A:
    case Phase40::STATIC_SR:
    case Phase40::STATIC_0_B:
    case Phase40::STATIC_SL:
    case Phase40::STATIC_0_C:
    case Phase40::STATIC_FR:
    case Phase40::STATIC_0_D:
    case Phase40::STATIC_FL:
    case Phase40::STATIC_0_E:
      return true;
    default:
      return false;
  }
}

static bool manualConfirmPhase40(Phase40 p) {
  switch (p) {
    case Phase40::SLOW_R:
    case Phase40::RETURN_SR:
    case Phase40::SLOW_L:
    case Phase40::RETURN_SL:
    case Phase40::FAST_R:
    case Phase40::RETURN_FR:
    case Phase40::FAST_L:
    case Phase40::RETURN_FL:
      return true;
    default:
      return false;
  }
}

static const char* phaseLabel40(Phase40 p) {
  switch (p) {
    case Phase40::WAIT_ZERO: return "ПОСТАВЬТЕ СТЕНД В ФИЗИЧЕСКИЙ НОЛЬ";
    case Phase40::CALIBRATE: return "КАЛИБРОВКА — НЕ ДВИГАТЬ";
    case Phase40::STATIC_0_A: return "ИСХОДНАЯ СТАТИКА — НЕ ДВИГАТЬ";
    case Phase40::SLOW_R: return "МЕДЛЕННО ПОВЕРНИТЕ ВПРАВО НА 60–100°";
    case Phase40::STATIC_SR: return "СТАТИКА СПРАВА — НЕ ДВИГАТЬ";
    case Phase40::RETURN_SR: return "МЕДЛЕННО ВЕРНИТЕ В ФИЗИЧЕСКИЙ НОЛЬ";
    case Phase40::STATIC_0_B: return "СТАТИКА В НУЛЕ — НЕ ДВИГАТЬ";
    case Phase40::SLOW_L: return "МЕДЛЕННО ПОВЕРНИТЕ ВЛЕВО НА 60–100°";
    case Phase40::STATIC_SL: return "СТАТИКА СЛЕВА — НЕ ДВИГАТЬ";
    case Phase40::RETURN_SL: return "МЕДЛЕННО ВЕРНИТЕ В ФИЗИЧЕСКИЙ НОЛЬ";
    case Phase40::STATIC_0_C: return "СТАТИКА В НУЛЕ — НЕ ДВИГАТЬ";
    case Phase40::FAST_R: return "БЫСТРО, НО ПЛАВНО ПОВЕРНИТЕ ВПРАВО НА 60–100°";
    case Phase40::STATIC_FR: return "СТАТИКА СПРАВА — НЕ ДВИГАТЬ";
    case Phase40::RETURN_FR: return "БЫСТРО, НО ПЛАВНО ВЕРНИТЕ В НОЛЬ";
    case Phase40::STATIC_0_D: return "СТАТИКА В НУЛЕ — НЕ ДВИГАТЬ";
    case Phase40::FAST_L: return "БЫСТРО, НО ПЛАВНО ПОВЕРНИТЕ ВЛЕВО НА 60–100°";
    case Phase40::STATIC_FL: return "СТАТИКА СЛЕВА — НЕ ДВИГАТЬ";
    case Phase40::RETURN_FL: return "БЫСТРО, НО ПЛАВНО ВЕРНИТЕ В НОЛЬ";
    case Phase40::STATIC_0_E: return "ФИНАЛЬНАЯ СТАТИКА — НЕ ДВИГАТЬ";
    case Phase40::DONE: return "ТЕСТ ЗАВЕРШЁН";
  }
  return "";
}

static void advanceAfterStatic40(State40& s) {
  s.static_n = 0;
  s.stable_n = 0;
  switch (s.phase) {
    case Phase40::STATIC_0_A: s.phase = Phase40::SLOW_R; break;
    case Phase40::STATIC_SR: s.phase = Phase40::RETURN_SR; break;
    case Phase40::STATIC_0_B: s.phase = Phase40::SLOW_L; break;
    case Phase40::STATIC_SL: s.phase = Phase40::RETURN_SL; break;
    case Phase40::STATIC_0_C: s.phase = Phase40::FAST_R; break;
    case Phase40::STATIC_FR: s.phase = Phase40::RETURN_FR; break;
    case Phase40::STATIC_0_D: s.phase = Phase40::FAST_L; break;
    case Phase40::STATIC_FL: s.phase = Phase40::RETURN_FL; break;
    case Phase40::STATIC_0_E: s.phase = Phase40::DONE; break;
    default: break;
  }
  std::cout << "[STEP] " << phaseLabel40(s.phase) << "\n";
}

static void advanceManual40(State40& s) {
  s.static_n = 0;
  s.stable_n = 0;
  switch (s.phase) {
    case Phase40::SLOW_R: s.phase = Phase40::STATIC_SR; break;
    case Phase40::RETURN_SR: s.phase = Phase40::STATIC_0_B; break;
    case Phase40::SLOW_L: s.phase = Phase40::STATIC_SL; break;
    case Phase40::RETURN_SL: s.phase = Phase40::STATIC_0_C; break;
    case Phase40::FAST_R: s.phase = Phase40::STATIC_FR; break;
    case Phase40::RETURN_FR: s.phase = Phase40::STATIC_0_D; break;
    case Phase40::FAST_L: s.phase = Phase40::STATIC_FL; break;
    case Phase40::RETURN_FL: s.phase = Phase40::STATIC_0_E; break;
    default: return;
  }
  std::cout << "[CONFIRM] position confirmed. " << phaseLabel40(s.phase) << "\n";
}

static Eigen::Vector3d correctedGyro40(Model40 m,
                                       const State40& s,
                                       const Eigen::Vector3d& wc,
                                       const Eigen::Vector3d& fc_rates) {
  Eigen::Vector3d w = wc;
  const double wz = wc.z();
  switch (m) {
    case Model40::RAW:
      return wc;

    case Model40::V39_TO_GEOM:
      // Convert the systematic v39 dynamic coupling into the coupling expected
      // for rotation around the gravity/world-vertical axis at the initial tilt.
      w.x() += (s.kx_geom - kV39Kxz40) * wz;
      w.y() += (s.ky_geom - kV39Kyz40) * wz;
      return w;

    case Model40::Y_TO_GEOM:
      // v39 Kxz was already close to ax/az; change only Y coupling.
      w.y() += (s.ky_geom - kV39Kyz40) * wz;
      return w;

    case Model40::GEOM_AXIS:
      // Diagnostic idealization for the yaw stand: angular velocity is forced
      // parallel to the initial gravity axis. Not intended as a flight fix.
      if (std::abs(s.u0.z()) > 1e-6) {
        w.x() = s.kx_geom * wz;
        w.y() = s.ky_geom * wz;
      }
      return w;

    case Model40::FC_RATES:
      return fc_rates;

    case Model40::COUNT:
      break;
  }
  return wc;
}

static void initialize40(State40& s, const Att40& att) {
  const Eigen::Vector3d mean_acc = s.cal_acc_sum / double(s.cal_n);
  s.BG = s.cal_gyro_sum / double(s.cal_n);
  const Eigen::Matrix3d R0 = fcRnedFlu(att.roll, att.pitch, att.yaw);
  const Eigen::Vector3d expected_acc = R0.transpose() * Eigen::Vector3d(0, 0, kG40);
  s.BA = mean_acc - expected_acc;
  s.u0 = mean_acc.normalized();
  if (std::abs(s.u0.z()) > 1e-6) {
    s.kx_geom = s.u0.x() / s.u0.z();
    s.ky_geom = s.u0.y() / s.u0.z();
  }
  for (auto& m : s.model) {
    m.R = R0;
    m.V.setZero();
    m.A.setZero();
  }
  s.last_us = 0;
  s.static_n = 0;
  s.stable_n = 0;
  s.phase = Phase40::STATIC_0_A;
  std::cout << "[CAL] BA=[" << s.BA.transpose() << "] BG=[" << s.BG.transpose() << "]\n";
  std::cout << "[GEOM] u0=[" << s.u0.transpose() << "] Kxz=" << s.kx_geom
            << " Kyz=" << s.ky_geom << "\n";
  std::cout << "[V39] measured Kxz=" << kV39Kxz40 << " Kyz=" << kV39Kyz40 << "\n";
}

static void process40(State40& s,
                      const Att40& att,
                      uint64_t us,
                      const Eigen::Vector3d& raw_acc,
                      const Eigen::Vector3d& raw_gyro) {
  const bool is_still = still40(raw_acc, raw_gyro);

  if (s.phase == Phase40::CALIBRATE) {
    if (is_still && att.valid) {
      s.cal_acc_sum += raw_acc;
      s.cal_gyro_sum += raw_gyro;
      ++s.cal_n;
      if (s.cal_n >= kCalN40) initialize40(s, att);
    } else {
      s.cal_n = 0;
      s.cal_acc_sum.setZero();
      s.cal_gyro_sum.setZero();
    }
    return;
  }

  if (s.phase == Phase40::WAIT_ZERO || s.phase == Phase40::DONE || !att.valid) return;

  double dt = 0.0;
  if (s.last_us && us > s.last_us) dt = (us - s.last_us) * 1e-6;
  s.last_us = us;
  if (dt <= 0.0 || dt > 0.03) return;

  const Eigen::Vector3d wc = raw_gyro - s.BG;
  const Eigen::Vector3d ac = raw_acc - s.BA;
  const Eigen::Vector3d fc_rates(att.rs, -att.ps, -att.ys);
  const int pi = measuredIndex40(s.phase);

  Row40 row;
  row.us = us;
  row.phase = pi;
  row.dt = dt;
  row.acc = raw_acc;
  row.gyro = wc;
  row.fc_rates = fc_rates;

  for (int i = 0; i < kModels40; ++i) {
    const Model40 model_id = static_cast<Model40>(i);
    const Eigen::Vector3d wm = correctedGyro40(model_id, s, wc, fc_rates);
    ModelState40& m = s.model[i];
    const Eigen::Matrix3d Rhalf = m.R * expR40(wm * (0.5 * dt));
    m.R = m.R * expR40(wm * dt);
    m.A = Rhalf * ac + Eigen::Vector3d(0, 0, -kG40);
    m.V += m.A * dt;

    row.w_model[i] = wm;
    row.a_model[i] = m.A;
    row.v_model[i] = m.V;
    row.grav_err[i] = gravityErr40(m.R, raw_acc);

    if (pi >= 0) {
      PhaseStats40& ps = s.stats[pi];
      ps.dv[i] += m.A * dt;
      const double axy = std::hypot(m.A.x(), m.A.y());
      ps.sum_axy[i] += axy;
      ps.max_axy[i] = std::max(ps.max_axy[i], axy);
      ps.sum_grav_err[i] += row.grav_err[i];
      ps.max_grav_err[i] = std::max(ps.max_grav_err[i], row.grav_err[i]);
    }
  }

  if (pi >= 0) {
    PhaseStats40& ps = s.stats[pi];
    ++ps.n;
    ps.dt += dt;
    ps.max_dt = std::max(ps.max_dt, dt);
    if (dt < 0.002 || dt > 0.008) ++ps.bad_dt;
    ps.sum_acc += raw_acc;
    ps.sum_gyro += wc;
    if (std::abs(wc.z()) > 0.05) {
      ps.gx_gz.add(wc.z(), wc.x());
      ps.gy_gz.add(wc.z(), wc.y());
    }
  }

  s.rows.push_back(row);

  if (is_still) ++s.stable_n;
  else s.stable_n = 0;

  if (staticPhase40(s.phase)) {
    if (is_still) ++s.static_n;
    else s.static_n = 0;
    if (s.static_n >= kStaticN40) advanceAfterStatic40(s);
  }
}

static Eigen::Vector3d meanVec40(const Eigen::Vector3d& sum, int n) {
  if (n <= 0) return Eigen::Vector3d::Zero();
  return sum / double(n);
}

static void save40(const State40& s) {
  std::ofstream f(kCsv40, std::ios::trunc);
  f << std::fixed << std::setprecision(9);
  f << "imu_us,phase,phase_name,dt,ax,ay,az,gx,gy,gz,fc_p_flu,fc_q_flu,fc_r_flu";
  for (int i = 0; i < kModels40; ++i) {
    const std::string n = kModelNames40[i];
    f << ',' << n << "_wx," << n << "_wy," << n << "_wz"
      << ',' << n << "_ax," << n << "_ay," << n << "_az"
      << ',' << n << "_vx," << n << "_vy," << n << "_vz"
      << ',' << n << "_grav_err_deg";
  }
  f << '\n';

  for (const Row40& r : s.rows) {
    const char* phase_name = (r.phase >= 0 && r.phase < kMeasured40) ? kPhaseNames40[r.phase] : "OTHER";
    f << r.us << ',' << r.phase << ',' << phase_name << ',' << r.dt
      << ',' << r.acc.x() << ',' << r.acc.y() << ',' << r.acc.z()
      << ',' << r.gyro.x() << ',' << r.gyro.y() << ',' << r.gyro.z()
      << ',' << r.fc_rates.x() << ',' << r.fc_rates.y() << ',' << r.fc_rates.z();
    for (int i = 0; i < kModels40; ++i) {
      f << ',' << r.w_model[i].x() << ',' << r.w_model[i].y() << ',' << r.w_model[i].z()
        << ',' << r.a_model[i].x() << ',' << r.a_model[i].y() << ',' << r.a_model[i].z()
        << ',' << r.v_model[i].x() << ',' << r.v_model[i].y() << ',' << r.v_model[i].z()
        << ',' << r.grav_err[i];
    }
    f << '\n';
  }
}

static void summary40(const State40& s) {
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n============================================================\n";
  std::cout << "JT-ZERO IMU COUPLING MODELS v40 RESULT\n";
  std::cout << "============================================================\n";
  std::cout << "BA=[" << s.BA.transpose() << "] m/s^2\n";
  std::cout << "BG=[" << s.BG.transpose() << "] rad/s\n";
  std::cout << "gravity axis u0=[" << s.u0.transpose() << "]\n";
  std::cout << "expected geometry Kxz=" << s.kx_geom << " Kyz=" << s.ky_geom << "\n";
  std::cout << "v39 measured Kxz=" << kV39Kxz40 << " Kyz=" << kV39Kyz40 << "\n";
  std::cout << "required delta Kx=" << (s.kx_geom - kV39Kxz40)
            << " Ky=" << (s.ky_geom - kV39Kyz40) << "\n";

  const std::array<int, 8> dyn = {1, 3, 5, 7, 9, 11, 13, 15};
  std::cout << "\n---------------- DYNAMIC COUPLING BY SPEED/DIRECTION ----------------\n";
  for (int idx : dyn) {
    const PhaseStats40& ps = s.stats[idx];
    std::cout << kPhaseNames40[idx]
              << " n=" << ps.n
              << " dt=" << ps.dt
              << " Kxz=" << ps.gx_gz.slope()
              << " corrX=" << ps.gx_gz.corr()
              << " Kyz=" << ps.gy_gz.slope()
              << " corrY=" << ps.gy_gz.corr()
              << " meanWz=" << meanVec40(ps.sum_gyro, ps.n).z()
              << "\n";
  }

  std::cout << "\n---------------- MODEL TOTALS ----------------\n";
  std::array<double, kModels40> dyn_dvx{};
  std::array<double, kModels40> dyn_dvy{};
  for (int idx : dyn) {
    for (int m = 0; m < kModels40; ++m) {
      dyn_dvx[m] += s.stats[idx].dv[m].x();
      dyn_dvy[m] += s.stats[idx].dv[m].y();
    }
  }

  for (int m = 0; m < kModels40; ++m) {
    double worst_static_grav = 0.0;
    double worst_static_axy = 0.0;
    for (int i = 0; i < kMeasured40; ++i) {
      if ((i % 2) == 0 && s.stats[i].n > 0) {
        worst_static_grav = std::max(worst_static_grav,
                                     s.stats[i].sum_grav_err[m] / s.stats[i].n);
        worst_static_axy = std::max(worst_static_axy,
                                    s.stats[i].sum_axy[m] / s.stats[i].n);
      }
    }
    const double dynamic_dv = std::hypot(dyn_dvx[m], dyn_dvy[m]);
    const double final_vxy = std::hypot(s.model[m].V.x(), s.model[m].V.y());
    std::cout << kModelNames40[m]
              << ": dynamic dVxy=" << dynamic_dv
              << " m/s, final Vxy=" << final_vxy
              << " m/s, worst static gravity err=" << worst_static_grav
              << " deg, worst static axy=" << worst_static_axy
              << " m/s^2\n";
  }

  std::cout << "\n---------------- DECISION AID ----------------\n";
  const double raw_final = std::hypot(s.model[static_cast<int>(Model40::RAW)].V.x(),
                                      s.model[static_cast<int>(Model40::RAW)].V.y());
  const double fixed_final = std::hypot(s.model[static_cast<int>(Model40::V39_TO_GEOM)].V.x(),
                                        s.model[static_cast<int>(Model40::V39_TO_GEOM)].V.y());
  const double y_final = std::hypot(s.model[static_cast<int>(Model40::Y_TO_GEOM)].V.x(),
                                    s.model[static_cast<int>(Model40::Y_TO_GEOM)].V.y());
  const double geom_final = std::hypot(s.model[static_cast<int>(Model40::GEOM_AXIS)].V.x(),
                                       s.model[static_cast<int>(Model40::GEOM_AXIS)].V.y());

  if (fixed_final < 0.5 * raw_final) {
    std::cout << "STRONG: fixed v39->geometry gyro-axis correction materially reduces runaway.\n";
  } else {
    std::cout << "WEAK: fixed v39->geometry correction does not reduce runaway by >=50%.\n";
  }
  if (y_final < 0.7 * raw_final) {
    std::cout << "Y-AXIS: correcting only Y coupling removes a substantial part of the error.\n";
  } else {
    std::cout << "Y-AXIS: Y-only correction is insufficient.\n";
  }
  if (geom_final < 0.3 * raw_final) {
    std::cout << "GEOMETRY: enforcing rotation about measured gravity axis nearly cures the stand test.\n";
  } else {
    std::cout << "GEOMETRY: gravity-axis idealization is not sufficient; coupling alone is not the full cause.\n";
  }

  double max_speed_spread_x = 0.0;
  double max_speed_spread_y = 0.0;
  const std::array<int, 4> outward = {1, 5, 9, 13};
  for (int a = 0; a < 4; ++a) {
    for (int b = a + 1; b < 4; ++b) {
      max_speed_spread_x = std::max(max_speed_spread_x,
        std::abs(s.stats[outward[a]].gx_gz.slope() - s.stats[outward[b]].gx_gz.slope()));
      max_speed_spread_y = std::max(max_speed_spread_y,
        std::abs(s.stats[outward[a]].gy_gz.slope() - s.stats[outward[b]].gy_gz.slope()));
    }
  }
  std::cout << "coupling spread across outward slow/fast/directions: dKx=" << max_speed_spread_x
            << " dKy=" << max_speed_spread_y << "\n";

  int bad_dt = 0;
  for (const auto& ps : s.stats) bad_dt += ps.bad_dt;
  std::cout << "abnormal dt samples=" << bad_dt << "\n";
  std::cout << "CSV: " << kCsv40 << "\n";
  std::cout << "Open CSV:\n  code " << kCsv40 << "\n";
}

static void hud40(const State40& s,
                  const Att40& att,
                  const Eigen::Vector3d& acc,
                  const Eigen::Vector3d& gyro) {
  cv::Mat canvas(900, 1380, CV_8UC3, cv::Scalar(15, 15, 15));
  const cv::Scalar white(235, 235, 235);
  const cv::Scalar green(80, 220, 80);
  const cv::Scalar yellow(0, 220, 255);
  const cv::Scalar red(80, 80, 255);

  uiText(canvas, "JT-ZERO: ПРОВЕРКА МОДЕЛЕЙ IMU v40", 35, 55, .68, white, 2);
  uiText(canvas, phaseLabel40(s.phase), 45, 125, .56,
         s.phase == Phase40::DONE ? green : yellow, 2);

  if (s.phase == Phase40::WAIT_ZERO) {
    uiText(canvas, "SPACE — ПОДТВЕРДИТЬ ФИЗИЧЕСКИЙ НОЛЬ", 45, 185, .50, white, 2);
  } else if (manualConfirmPhase40(s.phase)) {
    uiText(canvas, "После остановки дождитесь 120/120 и нажмите SPACE", 45, 185, .47, white, 2);
  }

  char b[256];
  snprintf(b, sizeof(b), "Стабильность %d/%d   |gyro| %.4f   |acc| %.4f",
           s.stable_n, kStableN40, gyro.norm(), acc.norm());
  uiText(canvas, b, 45, 250, .45, white, 1);

  snprintf(b, sizeof(b), "FC R/P/Y: %+.2f  %+.2f  %+.2f", att.roll, att.pitch, att.yaw);
  uiText(canvas, b, 45, 305, .45, white, 1);

  snprintf(b, sizeof(b), "Ожидаемая ось yaw: Kxz=%+.4f  Kyz=%+.4f", s.kx_geom, s.ky_geom);
  uiText(canvas, b, 45, 365, .46, green, 1);

  snprintf(b, sizeof(b), "RAW Vxy %.3f м/с",
           std::hypot(s.model[static_cast<int>(Model40::RAW)].V.x(),
                      s.model[static_cast<int>(Model40::RAW)].V.y()));
  uiText(canvas, b, 45, 440, .52, red, 2);

  snprintf(b, sizeof(b), "V39->GEOM Vxy %.3f м/с",
           std::hypot(s.model[static_cast<int>(Model40::V39_TO_GEOM)].V.x(),
                      s.model[static_cast<int>(Model40::V39_TO_GEOM)].V.y()));
  uiText(canvas, b, 45, 500, .50, white, 2);

  snprintf(b, sizeof(b), "Y->GEOM Vxy %.3f м/с",
           std::hypot(s.model[static_cast<int>(Model40::Y_TO_GEOM)].V.x(),
                      s.model[static_cast<int>(Model40::Y_TO_GEOM)].V.y()));
  uiText(canvas, b, 45, 560, .50, white, 2);

  snprintf(b, sizeof(b), "GEOM-AXIS Vxy %.3f м/с",
           std::hypot(s.model[static_cast<int>(Model40::GEOM_AXIS)].V.x(),
                      s.model[static_cast<int>(Model40::GEOM_AXIS)].V.y()));
  uiText(canvas, b, 45, 620, .50, green, 2);

  uiText(canvas, "Маршрут: медленно вправо/влево, затем быстро вправо/влево", 45, 720, .43, yellow, 1);
  uiText(canvas, "Точный угол не нужен. Возврат в механический ноль должен быть точным.", 45, 770, .42, white, 1);
  uiText(canvas, "ESC/Q — прервать", 45, 850, .38, white, 1);
  cv::imshow(kWindow40, canvas);
}

}  // namespace jtzero_v40

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  int fd = -1;
  uint8_t sys = 0;
  uint8_t comp = 0;
  mavlink_status_t mav_status{};
  mavlink_message_t msg{};
  jtzero_v40::Att40 att;
  jtzero_v40::State40 state;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  bool aborted = false;
  uint64_t last_imu_us = 0;

  try {
    fd = jtzero_v10::openSerial();
    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    const int64_t deadline = jtzero_v10::monotonicNs() + 10000000000LL;
    while (jtzero_v10::monotonicNs() < deadline && !sys) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t buf[2048];
      const ssize_t n = read(fd, buf, sizeof(buf));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &mav_status) &&
            msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          sys = msg.sysid;
          comp = msg.compid;
          break;
        }
      }
    }
    if (!sys) throw std::runtime_error("HEARTBEAT timeout");

    jtzero_v10::requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 100);
    jtzero_v10::requestRate(fd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 200);
    cv::namedWindow(jtzero_v40::kWindow40);
    std::cout << "\nJT-ZERO IMU COUPLING MODELS v40\nFollow Russian GUI.\n";

    int64_t next_hud = 0;
    while (true) {
      pollfd p{fd, POLLIN, 0};
      poll(&p, 1, 5);
      if (p.revents & POLLIN) {
        uint8_t buf[8192];
        const ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) {
          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &mav_status)) continue;
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(&msg, &a);
              att.valid = true;
              att.roll = a.roll * 180.0 / jtzero_v10::kPi;
              att.pitch = a.pitch * 180.0 / jtzero_v10::kPi;
              att.yaw = a.yaw * 180.0 / jtzero_v10::kPi;
              att.rs = a.rollspeed;
              att.ps = a.pitchspeed;
              att.ys = a.yawspeed;
            } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg, &h);
              if (h.time_usec == last_imu_us) continue;
              last_imu_us = h.time_usec;
              acc = Eigen::Vector3d(h.xacc, -h.yacc, -h.zacc);
              gyro = Eigen::Vector3d(h.xgyro, -h.ygyro, -h.zgyro);
              jtzero_v40::process40(state, att, h.time_usec, acc, gyro);
            }
          }
        }
      }

      if (jtzero_v10::monotonicNs() >= next_hud) {
        jtzero_v40::hud40(state, att, acc, gyro);
        next_hud = jtzero_v10::monotonicNs() + 33333333LL;
      }

      const int key = cv::waitKey(1) & 255;
      if (key == 27 || key == 'q' || key == 'Q') {
        aborted = true;
        break;
      }

      if (key == ' ' && state.phase == jtzero_v40::Phase40::WAIT_ZERO && att.valid) {
        state.phase = jtzero_v40::Phase40::CALIBRATE;
        state.cal_n = 0;
        state.cal_acc_sum.setZero();
        state.cal_gyro_sum.setZero();
        std::cout << "[ZERO] physical zero confirmed. Calibrating.\n";
      } else if (key == ' ' && jtzero_v40::manualConfirmPhase40(state.phase) &&
                 state.stable_n >= jtzero_v40::kStableN40) {
        jtzero_v40::advanceManual40(state);
      } else if (key == ' ' && state.phase == jtzero_v40::Phase40::DONE) {
        break;
      }

      if (state.phase == jtzero_v40::Phase40::DONE) {
        jtzero_v40::hud40(state, att, acc, gyro);
        const int k2 = cv::waitKey(1) & 255;
        if (k2 == ' ' || k2 == 27 || k2 == 'q' || k2 == 'Q') break;
      }
    }

    jtzero_v40::save40(state);
    jtzero_v40::summary40(state);
    std::cout << "aborted: " << (aborted ? "yes" : "no") << "\n";

    jtzero_v10::requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    jtzero_v10::requestRate(fd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
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
