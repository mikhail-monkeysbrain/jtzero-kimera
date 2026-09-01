// JT-ZERO v27: offline yaw cross-axis gyro diagnostic from v26 CSV.
// Fits the smallest identifiable model from the existing yaw-only stand run:
//   w_corr = (w_raw - BG_static) + c + k * wz_static
// where c=[cx,cy,cz] is a residual constant bias correction and
// k=[kxz,kyz,kzz] captures yaw-rate leakage/scale visible in this dataset.
// The v26 route is mostly yaw, so this tool deliberately does NOT claim to
// identify a full 3x3 gyro calibration matrix.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

struct Row {
  int phase = 0;
  std::string phase_name;
  double dt = 0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_static = Eigen::Vector3d::Zero();
  Eigen::Vector3d fc_euler_deg = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string x;
  while (std::getline(ss, x, ',')) out.push_back(x);
  return out;
}

double val(const std::vector<std::string>& v,
           const std::unordered_map<std::string, size_t>& h,
           const std::string& k) {
  auto it = h.find(k);
  if (it == h.end() || it->second >= v.size() || v[it->second].empty()) return 0.0;
  return std::stod(v[it->second]);
}

Eigen::Matrix3d fcR(const Eigen::Vector3d& e_deg) {
  // v26 logs Eigen::Matrix3d::eulerAngles(0,1,2), therefore reconstruct as Rx*Ry*Rz.
  Eigen::Vector3d e = e_deg * (kPi / 180.0);
  return Eigen::AngleAxisd(e.x(), Eigen::Vector3d::UnitX()).toRotationMatrix() *
         Eigen::AngleAxisd(e.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(e.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d logR(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  if (!std::isfinite(aa.angle())) return Eigen::Vector3d::Zero();
  return aa.axis() * aa.angle();
}

double rotDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  return logR(A.transpose() * B).norm() * 180.0 / kPi;
}

double gravDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  Eigen::Vector3d za = A.transpose() * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d zb = B.transpose() * Eigen::Vector3d::UnitZ();
  double d = std::clamp(za.dot(zb), -1.0, 1.0);
  return std::acos(d) * 180.0 / kPi;
}

struct Metrics {
  double rms_rot = 0, max_rot = 0, rms_grav = 0, max_grav = 0;
  double final_rot = 0, final_grav = 0, max_vxy = 0;
  Eigen::Vector3d final_v = Eigen::Vector3d::Zero();
  std::vector<double> rot, grav;
  std::vector<Eigen::Vector3d> vel;
};

Metrics integrate(const std::vector<Row>& rows,
                  const Eigen::Vector3d& c,
                  const Eigen::Vector3d& k) {
  Metrics m;
  if (rows.empty()) return m;
  Eigen::Matrix3d R = fcR(rows.front().fc_euler_deg);
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  double ssr = 0, ssg = 0;
  size_t n = 0;
  m.rot.resize(rows.size(), 0.0);
  m.grav.resize(rows.size(), 0.0);
  m.vel.resize(rows.size(), Eigen::Vector3d::Zero());

  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    if (i > 0 && r.dt > 0.001 && r.dt < 0.02) {
      Eigen::Vector3d w0 = r.gyr - r.bg_static;
      Eigen::Vector3d wc = w0 + c + k * w0.z();
      Eigen::Vector3d th = wc * r.dt;
      double a = th.norm();
      if (a > 1e-12) R = R * Eigen::AngleAxisd(a, th / a).toRotationMatrix();
      Eigen::Vector3d ac = r.acc - r.ba;
      Eigen::Vector3d aw = R * ac + Eigen::Vector3d(0, 0, kG);
      v += aw * r.dt;
    }
    const Eigen::Matrix3d F = fcR(r.fc_euler_deg);
    double er = rotDeg(F, R);
    double eg = gravDeg(F, R);
    m.rot[i] = er;
    m.grav[i] = eg;
    m.vel[i] = v;
    ssr += er * er;
    ssg += eg * eg;
    ++n;
    m.max_rot = std::max(m.max_rot, er);
    m.max_grav = std::max(m.max_grav, eg);
    m.max_vxy = std::max(m.max_vxy, v.head<2>().norm());
  }
  if (n) {
    m.rms_rot = std::sqrt(ssr / n);
    m.rms_grav = std::sqrt(ssg / n);
  }
  m.final_rot = m.rot.back();
  m.final_grav = m.grav.back();
  m.final_v = m.vel.back();
  return m;
}

void printMetrics(const char* name, const Metrics& m, const std::vector<Row>& rows) {
  std::cout << "\n" << name << "\n";
  std::cout << "  RMS rot=" << m.rms_rot << " deg   max rot=" << m.max_rot << " deg\n";
  std::cout << "  RMS grav=" << m.rms_grav << " deg  max grav=" << m.max_grav << " deg\n";
  std::cout << "  final rot=" << m.final_rot << " deg final grav=" << m.final_grav << " deg\n";
  std::cout << "  final V=[" << m.final_v.transpose() << "] m/s   max Vxy=" << m.max_vxy << " m/s\n";
  static const char* names[4] = {"+30", "0_after_plus", "-30", "0_final"};
  for (int p = 0; p < 4; ++p) {
    double sr = 0, sg = 0, mr = 0, mg = 0;
    size_t n = 0;
    for (size_t i = 0; i < rows.size(); ++i) if (rows[i].phase == p) {
      sr += m.rot[i] * m.rot[i];
      sg += m.grav[i] * m.grav[i];
      mr = std::max(mr, m.rot[i]);
      mg = std::max(mg, m.grav[i]);
      ++n;
    }
    if (n) std::cout << "    " << names[p]
                     << ": RMSrot=" << std::sqrt(sr/n) << " maxRot=" << mr
                     << " RMSgrav=" << std::sqrt(sg/n) << " maxGrav=" << mg << " deg\n";
  }
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " /home/vio/jtzero_live_full_chain_v26.csv\n";
    return 2;
  }
  std::ifstream f(argv[1]);
  if (!f) { std::cerr << "cannot open " << argv[1] << "\n"; return 2; }

  std::string line;
  if (!std::getline(f, line)) return 2;
  auto hh = split(line);
  std::unordered_map<std::string, size_t> h;
  for (size_t i = 0; i < hh.size(); ++i) h[hh[i]] = i;

  std::vector<Row> rows;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    auto v = split(line);
    try {
      Row r;
      r.phase = (int)val(v,h,"phase");
      auto it = h.find("phase_name");
      if (it != h.end() && it->second < v.size()) r.phase_name = v[it->second];
      r.dt = val(v,h,"dt");
      r.acc << val(v,h,"ax"), val(v,h,"ay"), val(v,h,"az");
      r.gyr << val(v,h,"gx"), val(v,h,"gy"), val(v,h,"gz");
      r.ba << val(v,h,"ba_x"), val(v,h,"ba_y"), val(v,h,"ba_z");
      r.bg_static << val(v,h,"bg_static_x"), val(v,h,"bg_static_y"), val(v,h,"bg_static_z");
      r.fc_euler_deg << val(v,h,"fc_roll"), val(v,h,"fc_pitch"), val(v,h,"fc_yaw");
      rows.push_back(r);
    } catch (...) {}
  }
  if (rows.size() < 20) { std::cerr << "too few rows\n"; return 2; }

  const Eigen::Vector3d bg_static = rows.front().bg_static;

  // Build FC body-rate reference from consecutive logged FC rotation matrices.
  // Fit residual against [1, wz_static]. This is the part identifiable from yaw excitation.
  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  Eigen::Matrix<double,2,3> B = Eigen::Matrix<double,2,3>::Zero();
  size_t used = 0, dyn = 0;
  double ss_base = 0, ss_fit = 0;
  std::vector<Eigen::Vector3d> wfc(rows.size(), Eigen::Vector3d::Zero());
  std::vector<char> good(rows.size(), 0);

  for (size_t i = 1; i < rows.size(); ++i) {
    double dt = rows[i].dt;
    if (!(dt > 0.001 && dt < 0.02)) continue;
    Eigen::Matrix3d R0 = fcR(rows[i-1].fc_euler_deg);
    Eigen::Matrix3d R1 = fcR(rows[i].fc_euler_deg);
    Eigen::Vector3d wf = logR(R0.transpose() * R1) / dt;
    if (!wf.allFinite() || wf.norm() > 4.0) continue;
    Eigen::Vector3d w0 = rows[i].gyr - bg_static;
    double wz = w0.z();
    Eigen::Vector2d x(1.0, wz);
    Eigen::Vector3d y = wf - w0;
    // Give dynamic yaw more weight, while retaining static rows to anchor intercept.
    double weight = (std::abs(wz) > 0.03) ? 4.0 : 1.0;
    H += weight * (x * x.transpose());
    B += weight * (x * y.transpose());
    wfc[i] = wf;
    good[i] = 1;
    ++used;
    if (std::abs(wz) > 0.03) ++dyn;
    ss_base += y.squaredNorm();
  }

  Eigen::Matrix<double,2,3> P = H.ldlt().solve(B);
  Eigen::Vector3d c = P.row(0).transpose();
  Eigen::Vector3d k = P.row(1).transpose();

  for (size_t i = 1; i < rows.size(); ++i) if (good[i]) {
    Eigen::Vector3d w0 = rows[i].gyr - bg_static;
    Eigen::Vector3d wf_hat = w0 + c + k * w0.z();
    ss_fit += (wfc[i] - wf_hat).squaredNorm();
  }

  Metrics base = integrate(rows, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  Metrics fit = integrate(rows, c, k);

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "============================================================\n";
  std::cout << "JT-ZERO OFFLINE YAW CROSS-AXIS ANALYZER v27\n";
  std::cout << "============================================================\n";
  std::cout << "rows: " << rows.size() << "   fitted intervals: " << used
            << "   dynamic yaw intervals: " << dyn << "\n";
  std::cout << "BG static: [" << bg_static.transpose() << "] rad/s\n";
  std::cout << "residual constant c: [" << c.transpose() << "] rad/s\n";
  std::cout << "yaw coupling k:      [" << k.transpose() << "]\n";
  std::cout << "  interpretation: wx += kx*wz, wy += ky*wz, wz += kz*wz\n";
  std::cout << "  approximate yaw->X angle = " << std::atan(k.x())*180.0/kPi << " deg\n";
  std::cout << "  approximate yaw->Y angle = " << std::atan(k.y())*180.0/kPi << " deg\n";
  std::cout << "  approximate Z scale = " << (1.0 + k.z()) << "\n";
  std::cout << "rate-fit RMS before=" << std::sqrt(ss_base/std::max<size_t>(1,used))
            << " rad/s after=" << std::sqrt(ss_fit/std::max<size_t>(1,used)) << " rad/s\n";

  printMetrics("STATIC BG ONLY", base, rows);
  printMetrics("STATIC BG + YAW-AFFINE CORRECTION", fit, rows);

  std::cout << "\nDecision helper:\n";
  double ratio = fit.rms_grav / std::max(1e-12, base.rms_grav);
  std::cout << "  gravity RMS ratio corrected/static = " << ratio << "\n";
  if (ratio < 0.35 && fit.max_grav < 1.0) {
    std::cout << "  RESULT: yaw-correlated cross-axis/scale error explains most of the remaining drift.\n";
  } else if (ratio < 0.75) {
    std::cout << "  RESULT: yaw-correlated calibration error is material, but not sufficient alone.\n";
  } else {
    std::cout << "  RESULT: simple yaw cross-axis/scale model is insufficient. Investigate FC attitude semantics/timing or richer IMU calibration.\n";
  }
  std::cout << "  NOTE: this yaw-only run cannot identify a full 3x3 gyro calibration matrix.\n";
  return 0;
}
