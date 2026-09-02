// JT-ZERO v42: independent fixed-coefficient validation of v41 gyro correction.
// Uses a NEW v40 validation CSV and NEVER optimizes coefficients.
// Fixed coefficients are taken from the previous fitset (v41 all-linear):
//   wx_corr = wx + 0.014570 * wz
//   wy_corr = wy + 0.082383 * wz
//   wz_corr = wz

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kCx = 0.014570;
constexpr double kCy = 0.082383;
constexpr const char* kDefaultCsv = "/home/vio/jtzero_live_imu_coupling_models_v40_validation.csv";

struct Sample {
  double dt = 0.0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct Segment {
  std::string name;
  std::string start_static;
  std::string dynamic_phase;
  std::string end_static;
  std::vector<Sample> dynamic;
  Eigen::Vector3d u_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d u_end = Eigen::Vector3d::Zero();
  double mechanical_delta_deg = 0.0;
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  bool quoted = false;
  for (char c : line) {
    if (c == '"') quoted = !quoted;
    else if (c == ',' && !quoted) { out.push_back(cur); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

double clamp1(double x) { return std::max(-1.0, std::min(1.0, x)); }

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  if (a.norm() < 1e-12 || b.norm() < 1e-12) return 0.0;
  return std::acos(clamp1(a.normalized().dot(b.normalized()))) * 180.0 / kPi;
}

Eigen::Matrix3d expR(const Eigen::Vector3d& th) {
  const double a = th.norm();
  if (a < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a, th / a).toRotationMatrix();
}

Eigen::Vector3d meanStaticGravity(const std::map<std::string, std::vector<Sample>>& phases,
                                  const std::string& name) {
  auto it = phases.find(name);
  if (it == phases.end() || it->second.empty())
    throw std::runtime_error("Missing static phase: " + name);
  Eigen::Vector3d s = Eigen::Vector3d::Zero();
  for (const auto& x : it->second) s += x.acc;
  if (s.norm() < 1e-12) throw std::runtime_error("Invalid gravity phase: " + name);
  return s.normalized();
}

std::vector<Segment> buildSegments(const std::map<std::string, std::vector<Sample>>& phases) {
  struct D { const char* n; const char* a; const char* d; const char* b; };
  const std::array<D,8> defs{{
    {"SLOW_R",    "STATIC_0_A", "SLOW_R",    "STATIC_SR"},
    {"RETURN_SR", "STATIC_SR",  "RETURN_SR", "STATIC_0_B"},
    {"SLOW_L",    "STATIC_0_B", "SLOW_L",    "STATIC_SL"},
    {"RETURN_SL", "STATIC_SL",  "RETURN_SL", "STATIC_0_C"},
    {"FAST_R",    "STATIC_0_C", "FAST_R",    "STATIC_FR"},
    {"RETURN_FR", "STATIC_FR",  "RETURN_FR", "STATIC_0_D"},
    {"FAST_L",    "STATIC_0_D", "FAST_L",    "STATIC_FL"},
    {"RETURN_FL", "STATIC_FL",  "RETURN_FL", "STATIC_0_E"},
  }};
  std::vector<Segment> out;
  for (const auto& d : defs) {
    auto it = phases.find(d.d);
    if (it == phases.end() || it->second.empty())
      throw std::runtime_error(std::string("Missing dynamic phase: ") + d.d);
    Segment s;
    s.name = d.n; s.start_static = d.a; s.dynamic_phase = d.d; s.end_static = d.b;
    s.dynamic = it->second;
    s.u_start = meanStaticGravity(phases, d.a);
    s.u_end = meanStaticGravity(phases, d.b);
    s.mechanical_delta_deg = angleDeg(s.u_start, s.u_end);
    out.push_back(std::move(s));
  }
  return out;
}

Eigen::Vector3d predict(const Segment& s, bool corrected) {
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  for (const auto& sm : s.dynamic) {
    Eigen::Vector3d w = sm.gyro;
    if (corrected) {
      const double wz = w.z();
      w.x() += kCx * wz;
      w.y() += kCy * wz;
    }
    dR = dR * expR(w * sm.dt);
  }
  return (dR.transpose() * s.u_start).normalized();
}

struct Totals { double rms=0, mean=0, max=0; };

Totals printBlock(const char* title, const std::vector<Segment>& segs, bool corrected) {
  double ss=0, sum=0, mx=0;
  std::cout << "\n---------------- " << title << " ----------------\n";
  for (const auto& s : segs) {
    const double e = angleDeg(predict(s, corrected), s.u_end);
    ss += e*e; sum += e; mx = std::max(mx,e);
    std::cout << std::setw(10) << std::left << s.name
              << " endpoint_err=" << std::setw(10) << std::right << e
              << " deg  mechanical_delta=" << s.mechanical_delta_deg << " deg\n";
  }
  Totals t;
  t.rms = std::sqrt(ss / segs.size());
  t.mean = sum / segs.size();
  t.max = mx;
  std::cout << "TOTAL RMS=" << t.rms << " deg mean=" << t.mean << " max=" << t.max << "\n";
  return t;
}
}

int main(int argc, char** argv) {
  try {
    const std::string path = argc > 1 ? argv[1] : kDefaultCsv;
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open CSV: " + path);

    std::string line;
    if (!std::getline(f,line)) throw std::runtime_error("Empty CSV");
    const auto hdr = splitCsv(line);
    std::map<std::string,int> col;
    for (int i=0;i<(int)hdr.size();++i) col[hdr[i]]=i;
    for (const char* req : {"phase_name","dt","ax","ay","az","gx","gy","gz"})
      if (!col.count(req)) throw std::runtime_error(std::string("Missing column: ")+req);

    std::map<std::string,std::vector<Sample>> phases;
    size_t rows=0;
    while (std::getline(f,line)) {
      if (line.empty()) continue;
      const auto v = splitCsv(line);
      auto get=[&](const char* n)->const std::string&{
        int i=col.at(n); if (i<0 || i>=(int)v.size()) throw std::runtime_error("Malformed CSV row"); return v[i]; };
      const std::string phase=get("phase_name");
      if (phase=="OTHER") continue;
      Sample s;
      s.dt=std::stod(get("dt"));
      if (s.dt<=0.0 || s.dt>0.03) continue;
      s.acc=Eigen::Vector3d(std::stod(get("ax")),std::stod(get("ay")),std::stod(get("az")));
      s.gyro=Eigen::Vector3d(std::stod(get("gx")),std::stod(get("gy")),std::stod(get("gz")));
      phases[phase].push_back(s); ++rows;
    }

    const auto segs=buildSegments(phases);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n============================================================\n";
    std::cout << "JT-ZERO FIXED-COEFFICIENT VALIDATION v42 RESULT\n";
    std::cout << "============================================================\n";
    std::cout << "CSV: " << path << "\nrows used: " << rows << "\n";
    std::cout << "FIXED coefficients: cx=" << kCx << " cy=" << kCy << "\n";
    std::cout << "NO OPTIMIZATION WAS PERFORMED ON THIS DATASET.\n";

    const Totals raw=printBlock("RAW",segs,false);
    const Totals fix=printBlock("FIXED LINEAR CORRECTION",segs,true);

    std::cout << "\n---------------- DECISION AID ----------------\n";
    std::cout << "RAW RMS: " << raw.rms << " deg\n";
    std::cout << "FIXED RMS: " << fix.rms << " deg\n";
    std::cout << "improvement x" << (fix.rms>1e-12 ? raw.rms/fix.rms : 0.0) << "\n";
    if (fix.rms <= 0.30 && fix.max <= 0.60)
      std::cout << "RESULT: PASS - fixed correction generalizes to the independent run.\n";
    else if (fix.rms <= 0.60)
      std::cout << "RESULT: PARTIAL - correction generalizes, but residual is larger than target.\n";
    else
      std::cout << "RESULT: FAIL - fixed correction does not generalize sufficiently.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
