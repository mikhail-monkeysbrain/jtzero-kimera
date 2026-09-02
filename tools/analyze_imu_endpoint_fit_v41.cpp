// JT-ZERO v41: offline endpoint-constrained gyro coupling analyzer.
// Uses the existing v40 CSV. No new mechanical run is required.
// The key idea: for each dynamic segment, reset from the measured accelerometer
// gravity direction at the segment start, integrate gyro only over that segment,
// then compare predicted gravity with the measured static endpoint gravity.
// This separates cumulative drift from real mechanical tilt of the stand.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr const char* kDefaultCsv = "/home/vio/jtzero_live_imu_coupling_models_v40.csv";

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
  bool slow = false;
  bool fast = false;
  std::vector<Sample> dynamic;
  Eigen::Vector3d u_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d u_end = Eigen::Vector3d::Zero();
  double mechanical_delta_deg = 0.0;
};

struct Params {
  double cx = 0.0;  // wx += cx*wz
  double cy = 0.0;  // wy += cy*wz
  double qx = 0.0;  // wx += qx*wz*|wz|
  double qy = 0.0;  // wy += qy*wz*|wz|
};

struct EvalResult {
  double rms_deg = 0.0;
  double mean_deg = 0.0;
  double max_deg = 0.0;
  std::vector<double> per_segment_deg;
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  bool quoted = false;
  for (char c : line) {
    if (c == '"') {
      quoted = !quoted;
    } else if (c == ',' && !quoted) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

double clamp1(double x) {
  return std::max(-1.0, std::min(1.0, x));
}

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  if (a.norm() < 1e-12 || b.norm() < 1e-12) return 0.0;
  return std::acos(clamp1(a.normalized().dot(b.normalized()))) * 180.0 / kPi;
}

Eigen::Matrix3d expR(const Eigen::Vector3d& th) {
  const double a = th.norm();
  if (a < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a, th / a).toRotationMatrix();
}

Eigen::Vector3d correctedW(const Eigen::Vector3d& w, const Params& p, bool quadratic) {
  Eigen::Vector3d c = w;
  const double z = w.z();
  c.x() += p.cx * z;
  c.y() += p.cy * z;
  if (quadratic) {
    const double zz = z * std::abs(z);
    c.x() += p.qx * zz;
    c.y() += p.qy * zz;
  }
  return c;
}

Eigen::Vector3d predictEndGravity(const Segment& s, const Params& p, bool quadratic) {
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  for (const Sample& sm : s.dynamic) {
    const Eigen::Vector3d w = correctedW(sm.gyro, p, quadratic);
    dR = dR * expR(w * sm.dt);
  }
  return (dR.transpose() * s.u_start).normalized();
}

EvalResult evaluate(const std::vector<Segment>& segs,
                    const std::vector<int>& ids,
                    const Params& p,
                    bool quadratic) {
  EvalResult r;
  if (ids.empty()) return r;
  double sse = 0.0;
  double sum = 0.0;
  for (int id : ids) {
    const double e = angleDeg(predictEndGravity(segs[id], p, quadratic), segs[id].u_end);
    r.per_segment_deg.push_back(e);
    sse += e * e;
    sum += e;
    r.max_deg = std::max(r.max_deg, e);
  }
  r.rms_deg = std::sqrt(sse / ids.size());
  r.mean_deg = sum / ids.size();
  return r;
}

double objective(const std::vector<Segment>& segs,
                 const std::vector<int>& ids,
                 const Params& p,
                 bool quadratic) {
  return evaluate(segs, ids, p, quadratic).rms_deg;
}

Params optimizeLinear(const std::vector<Segment>& segs,
                      const std::vector<int>& ids,
                      Params p = Params{}) {
  double step = 0.02;
  double best = objective(segs, ids, p, false);
  while (step > 1e-5) {
    bool improved = false;
    Params bestp = p;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        Params q = p;
        q.cx += dx * step;
        q.cy += dy * step;
        const double v = objective(segs, ids, q, false);
        if (v + 1e-12 < best) {
          best = v;
          bestp = q;
          improved = true;
        }
      }
    }
    if (improved) {
      p = bestp;
    } else {
      step *= 0.5;
    }
  }
  return p;
}

Params optimizeQuadratic(const std::vector<Segment>& segs,
                         const std::vector<int>& ids,
                         Params p) {
  std::array<double, 4> step{0.01, 0.01, 0.02, 0.02};
  double best = objective(segs, ids, p, true);
  int guard = 0;
  while (*std::max_element(step.begin(), step.end()) > 2e-5 && guard++ < 10000) {
    bool improved = false;
    for (int j = 0; j < 4; ++j) {
      for (int sign : {-1, 1}) {
        Params q = p;
        if (j == 0) q.cx += sign * step[j];
        if (j == 1) q.cy += sign * step[j];
        if (j == 2) q.qx += sign * step[j];
        if (j == 3) q.qy += sign * step[j];
        const double v = objective(segs, ids, q, true);
        if (v + 1e-12 < best) {
          best = v;
          p = q;
          improved = true;
        }
      }
    }
    if (!improved) {
      for (double& x : step) x *= 0.5;
    }
  }
  return p;
}

Eigen::Vector3d meanStaticGravity(const std::map<std::string, std::vector<Sample>>& phases,
                                  const std::string& name) {
  auto it = phases.find(name);
  if (it == phases.end() || it->second.empty()) {
    throw std::runtime_error("Missing static phase: " + name);
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const Sample& s : it->second) sum += s.acc;
  if (sum.norm() < 1e-9) throw std::runtime_error("Invalid static gravity: " + name);
  return sum.normalized();
}

std::vector<Segment> buildSegments(const std::map<std::string, std::vector<Sample>>& phases) {
  struct Def { const char* name; const char* a; const char* d; const char* b; bool slow; bool fast; };
  const std::array<Def, 8> defs{{
      {"SLOW_R",    "STATIC_0_A", "SLOW_R",    "STATIC_SR",  true,  false},
      {"RETURN_SR", "STATIC_SR",  "RETURN_SR", "STATIC_0_B",true,  false},
      {"SLOW_L",    "STATIC_0_B", "SLOW_L",    "STATIC_SL",  true,  false},
      {"RETURN_SL", "STATIC_SL",  "RETURN_SL", "STATIC_0_C",true,  false},
      {"FAST_R",    "STATIC_0_C", "FAST_R",    "STATIC_FR",  false, true},
      {"RETURN_FR", "STATIC_FR",  "RETURN_FR", "STATIC_0_D",false, true},
      {"FAST_L",    "STATIC_0_D", "FAST_L",    "STATIC_FL",  false, true},
      {"RETURN_FL", "STATIC_FL",  "RETURN_FL", "STATIC_0_E",false, true},
  }};

  std::vector<Segment> out;
  for (const auto& d : defs) {
    Segment s;
    s.name = d.name;
    s.start_static = d.a;
    s.dynamic_phase = d.d;
    s.end_static = d.b;
    s.slow = d.slow;
    s.fast = d.fast;
    auto it = phases.find(d.d);
    if (it == phases.end() || it->second.empty()) {
      throw std::runtime_error(std::string("Missing dynamic phase: ") + d.d);
    }
    s.dynamic = it->second;
    s.u_start = meanStaticGravity(phases, d.a);
    s.u_end = meanStaticGravity(phases, d.b);
    s.mechanical_delta_deg = angleDeg(s.u_start, s.u_end);
    out.push_back(std::move(s));
  }
  return out;
}

void printParams(const char* label, const Params& p, bool quadratic) {
  std::cout << label << " cx=" << p.cx << " cy=" << p.cy;
  if (quadratic) std::cout << " qx=" << p.qx << " qy=" << p.qy;
  std::cout << '\n';
}

void printEval(const char* label,
               const std::vector<Segment>& segs,
               const std::vector<int>& ids,
               const Params& p,
               bool quadratic) {
  const EvalResult r = evaluate(segs, ids, p, quadratic);
  std::cout << label << " RMS=" << r.rms_deg
            << " deg mean=" << r.mean_deg
            << " max=" << r.max_deg << "\n";
  for (size_t j = 0; j < ids.size(); ++j) {
    const Segment& s = segs[ids[j]];
    std::cout << "  " << std::setw(10) << std::left << s.name
              << " endpoint_err=" << std::setw(10) << std::right << r.per_segment_deg[j]
              << " deg  mechanical_delta=" << s.mechanical_delta_deg << " deg\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string path = argc > 1 ? argv[1] : kDefaultCsv;
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open CSV: " + path);

    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("Empty CSV");
    const auto header = splitCsv(line);
    std::map<std::string, int> col;
    for (int i = 0; i < static_cast<int>(header.size()); ++i) col[header[i]] = i;
    for (const char* req : {"phase_name", "dt", "ax", "ay", "az", "gx", "gy", "gz"}) {
      if (!col.count(req)) throw std::runtime_error(std::string("Missing column: ") + req);
    }

    std::map<std::string, std::vector<Sample>> phases;
    size_t rows = 0;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      const auto v = splitCsv(line);
      auto get = [&](const char* name) -> const std::string& {
        const int i = col.at(name);
        if (i < 0 || i >= static_cast<int>(v.size())) throw std::runtime_error("Malformed CSV row");
        return v[i];
      };
      const std::string phase = get("phase_name");
      if (phase == "OTHER") continue;
      Sample s;
      s.dt = std::stod(get("dt"));
      s.acc = Eigen::Vector3d(std::stod(get("ax")), std::stod(get("ay")), std::stod(get("az")));
      s.gyro = Eigen::Vector3d(std::stod(get("gx")), std::stod(get("gy")), std::stod(get("gz")));
      if (s.dt <= 0.0 || s.dt > 0.03) continue;
      phases[phase].push_back(s);
      ++rows;
    }

    const std::vector<Segment> segs = buildSegments(phases);
    std::vector<int> all, slow, fast;
    for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
      all.push_back(i);
      if (segs[i].slow) slow.push_back(i);
      if (segs[i].fast) fast.push_back(i);
    }

    const Params raw{};
    const Params fit_slow_lin = optimizeLinear(segs, slow);
    const Params fit_fast_lin = optimizeLinear(segs, fast);
    const Params fit_all_lin = optimizeLinear(segs, all);
    const Params fit_slow_quad = optimizeQuadratic(segs, slow, fit_slow_lin);
    const Params fit_fast_quad = optimizeQuadratic(segs, fast, fit_fast_lin);
    const Params fit_all_quad = optimizeQuadratic(segs, all, fit_all_lin);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n============================================================\n";
    std::cout << "JT-ZERO ENDPOINT-CONSTRAINED GYRO FIT v41 RESULT\n";
    std::cout << "============================================================\n";
    std::cout << "CSV: " << path << "\n";
    std::cout << "rows used: " << rows << "\n\n";

    std::cout << "---------------- MECHANICAL ENDPOINT TILT ----------------\n";
    for (const auto& s : segs) {
      std::cout << std::setw(10) << std::left << s.name
                << " start->end gravity change=" << std::setw(10) << std::right
                << s.mechanical_delta_deg << " deg\n";
    }

    std::cout << "\n---------------- RAW ENDPOINT CONSISTENCY ----------------\n";
    printEval("RAW all", segs, all, raw, false);

    std::cout << "\n---------------- LINEAR z->x/y FIT ----------------\n";
    printParams("FIT slow", fit_slow_lin, false);
    printEval("TRAIN slow", segs, slow, fit_slow_lin, false);
    printEval("VALID fast", segs, fast, fit_slow_lin, false);
    std::cout << '\n';
    printParams("FIT fast", fit_fast_lin, false);
    printEval("TRAIN fast", segs, fast, fit_fast_lin, false);
    printEval("VALID slow", segs, slow, fit_fast_lin, false);
    std::cout << '\n';
    printParams("FIT all", fit_all_lin, false);
    printEval("ALL linear", segs, all, fit_all_lin, false);

    std::cout << "\n---------------- QUADRATIC RATE-DEPENDENT FIT ----------------\n";
    printParams("FIT slow quad", fit_slow_quad, true);
    printEval("TRAIN slow quad", segs, slow, fit_slow_quad, true);
    printEval("VALID fast quad", segs, fast, fit_slow_quad, true);
    std::cout << '\n';
    printParams("FIT fast quad", fit_fast_quad, true);
    printEval("TRAIN fast quad", segs, fast, fit_fast_quad, true);
    printEval("VALID slow quad", segs, slow, fit_fast_quad, true);
    std::cout << '\n';
    printParams("FIT all quad", fit_all_quad, true);
    printEval("ALL quadratic", segs, all, fit_all_quad, true);

    const double raw_rms = evaluate(segs, all, raw, false).rms_deg;
    const double lin_rms = evaluate(segs, all, fit_all_lin, false).rms_deg;
    const double quad_rms = evaluate(segs, all, fit_all_quad, true).rms_deg;
    const double slow_to_fast = evaluate(segs, fast, fit_slow_lin, false).rms_deg;
    const double fast_to_slow = evaluate(segs, slow, fit_fast_lin, false).rms_deg;

    std::cout << "\n---------------- DECISION AID ----------------\n";
    std::cout << "RAW RMS endpoint gravity error: " << raw_rms << " deg\n";
    std::cout << "Linear all-fit RMS: " << lin_rms << " deg\n";
    std::cout << "Quadratic all-fit RMS: " << quad_rms << " deg\n";
    std::cout << "Cross-validation linear slow->fast: " << slow_to_fast << " deg\n";
    std::cout << "Cross-validation linear fast->slow: " << fast_to_slow << " deg\n";

    if (lin_rms < raw_rms * 0.5 && slow_to_fast < raw_rms * 0.7 && fast_to_slow < raw_rms * 0.7) {
      std::cout << "RESULT: STRONG support for a repeatable linear z->x/y gyro correction.\n";
    } else if (lin_rms < raw_rms * 0.7) {
      std::cout << "RESULT: PARTIAL linear correction; cross-validation must decide whether it is safe.\n";
    } else {
      std::cout << "RESULT: linear coupling alone is not a stable explanation.\n";
    }

    if (quad_rms < lin_rms * 0.8) {
      std::cout << "RATE: quadratic rate dependence materially improves endpoint fit.\n";
    } else {
      std::cout << "RATE: little evidence that quadratic rate dependence is required.\n";
    }

    std::cout << "\nNo new motion was used: v41 is an offline re-analysis of v40.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << '\n';
    return 1;
  }
}
