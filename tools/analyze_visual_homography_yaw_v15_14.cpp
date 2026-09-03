#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct CamRow {
  uint64_t seq = 0;
  int64_t ts_ns = 0;
  uint64_t offset = 0;
  uint64_t bytes = 0;
};

struct ImuRow {
  int64_t mapped_ns = 0;
  double gx_flu = 0.0;
  double gy_flu = 0.0;
  double gz_flu = 0.0;
};

struct PairMetric {
  int64_t t0_ns = 0;
  int64_t t1_ns = 0;
  double t_s = 0.0;
  std::string phase;
  int detected = 0;
  int tracked = 0;
  int h_inliers = 0;
  double h_inlier_ratio = 0.0;
  double median_flow_px = 0.0;
  double median_h_res_px = 0.0;
  double p90_h_res_px = 0.0;
  int homography_solutions = 0;
};

static std::vector<std::string> splitCsv(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') { out.push_back(cur); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

static bool isNumber(const std::string& s) {
  if (s.empty()) return false;
  char* end = nullptr;
  std::strtod(s.c_str(), &end);
  return end && *end == '\0';
}

static double percentile(std::vector<double> v, double q) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  const double x = q * double(v.size() - 1);
  const size_t i = size_t(std::floor(x));
  const size_t j = std::min(i + 1, v.size() - 1);
  const double a = x - double(i);
  return v[i] * (1.0 - a) + v[j] * a;
}

static std::vector<CamRow> loadCameraIndex(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open camera index: " + path);
  std::vector<CamRow> rows;
  std::string line;
  while (std::getline(f, line)) {
    auto c = splitCsv(line);
    if (c.size() < 7 || !isNumber(c[0]) || !isNumber(c[2]) || !isNumber(c[5]) || !isNumber(c[6])) continue;
    CamRow r;
    r.seq = std::stoull(c[0]);
    r.ts_ns = std::stoll(c[2]);
    r.offset = std::stoull(c[5]);
    r.bytes = std::stoull(c[6]);
    rows.push_back(r);
  }
  return rows;
}

static std::vector<ImuRow> loadImu(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open combined csv: " + path);
  std::vector<ImuRow> rows;
  std::string line;
  constexpr double Cx = 0.014570;
  constexpr double Cy = 0.082383;
  while (std::getline(f, line)) {
    auto c = splitCsv(line);
    if (c.size() < 14 || c[0] != "IMU") continue;
    if (!isNumber(c[3]) || !isNumber(c[11]) || !isNumber(c[12]) || !isNumber(c[13])) continue;
    const double gx_frd = std::stod(c[11]);
    const double gy_frd = std::stod(c[12]);
    const double gz_frd = std::stod(c[13]);
    const double gx = gx_frd;
    const double gy = -gy_frd;
    const double gz = -gz_frd;
    ImuRow r;
    r.mapped_ns = std::stoll(c[3]);
    r.gx_flu = gx + Cx * gz;
    r.gy_flu = gy + Cy * gz;
    r.gz_flu = gz;
    rows.push_back(r);
  }
  return rows;
}

static cv::Mat decodeAt(std::ifstream& mj, const CamRow& r) {
  std::vector<uchar> buf(r.bytes);
  mj.clear();
  mj.seekg(std::streamoff(r.offset), std::ios::beg);
  mj.read(reinterpret_cast<char*>(buf.data()), std::streamsize(buf.size()));
  if (!mj) return {};
  return cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
}

static const char* phaseName(int64_t t, int64_t yaw0, int64_t yaw1) {
  if (t < yaw0) return "PRE";
  if (t <= yaw1) return "YAW";
  return "POST";
}

struct PhaseSummary {
  std::string name;
  int pairs = 0;
  std::vector<double> inlier_ratio;
  std::vector<double> flow;
  std::vector<double> hres;
  std::vector<double> hp90;
  std::vector<double> tracked;
  int near_planar = 0;
};

static void printSummary(const PhaseSummary& s) {
  std::cout << "\n[" << s.name << "] pairs=" << s.pairs << "\n";
  if (s.pairs == 0) return;
  std::cout << std::fixed << std::setprecision(3)
            << "  tracked median: " << percentile(s.tracked, 0.5) << "\n"
            << "  raw flow median px: " << percentile(s.flow, 0.5) << "\n"
            << "  homography inlier ratio median: " << percentile(s.inlier_ratio, 0.5) << "\n"
            << "  homography residual median px: " << percentile(s.hres, 0.5) << "\n"
            << "  homography residual p90 px: " << percentile(s.hp90, 0.5) << "\n"
            << "  near-planar pairs (ratio>=0.90 && median_res<=0.50px): "
            << s.near_planar << "/" << s.pairs << " ("
            << (100.0 * double(s.near_planar) / double(s.pairs)) << "%)\n";
}

int main(int argc, char** argv) {
  try {
    const std::string combined = argc > 1 ? argv[1] : "/home/vio/jtzero_yaw_only_v13.csv";
    const std::string camindex = argc > 2 ? argv[2] : "/home/vio/jtzero_yaw_only_v13_camera.csv";
    const std::string mjpeg = argc > 3 ? argv[3] : "/home/vio/jtzero_yaw_only_v13.mjpg";
    const std::string outcsv = argc > 4 ? argv[4] : "/home/vio/jtzero_visual_homography_v15_14.csv";

    auto cams = loadCameraIndex(camindex);
    auto imu = loadImu(combined);
    if (cams.size() < 2 || imu.empty()) throw std::runtime_error("not enough input rows");

    constexpr double rad2deg = 180.0 / M_PI;
    const double yaw_thresh = 5.0 / rad2deg;
    int64_t yaw0 = 0, yaw1 = 0;
    for (const auto& r : imu) {
      if (std::abs(r.gz_flu) >= yaw_thresh) {
        if (yaw0 == 0) yaw0 = r.mapped_ns;
        yaw1 = r.mapped_ns;
      }
    }
    if (yaw0 == 0 || yaw1 == 0) throw std::runtime_error("yaw interval not detected");

    std::vector<CamRow> selected;
    const int64_t select_period_ns = 30000000LL;
    int64_t last = std::numeric_limits<int64_t>::min() / 4;
    for (const auto& r : cams) {
      if (selected.empty() || r.ts_ns - last >= select_period_ns) {
        selected.push_back(r);
        last = r.ts_ns;
      }
    }

    std::ifstream mj(mjpeg, std::ios::binary);
    if (!mj) throw std::runtime_error("cannot open mjpeg: " + mjpeg);

    const cv::Mat K = (cv::Mat_<double>(3,3) <<
      568.53170752165227, 0.0, 315.98271077441063,
      0.0, 569.68005562865858, 239.88148589100641,
      0.0, 0.0, 1.0);

    std::ofstream csv(outcsv);
    csv << "pair,t0_ns,t1_ns,t_s,phase,detected,tracked,h_inliers,h_inlier_ratio,median_flow_px,median_h_res_px,p90_h_res_px,homography_solutions\n";

    std::cout << "============================================================\n"
              << "JT-ZERO VISUAL HOMOGRAPHY / OBSERVABILITY v15.14\n"
              << "============================================================\n"
              << "raw camera rows: " << cams.size() << "\n"
              << "selected camera rows: " << selected.size() << "\n"
              << std::fixed << std::setprecision(6)
              << "yaw mapped interval: " << yaw0 * 1e-9 << " .. " << yaw1 * 1e-9 << " s\n";

    PhaseSummary pre{"PRE"}, yaw{"YAW"}, post{"POST"};
    const int64_t t_ref = selected.front().ts_ns;
    int valid_pairs = 0;
    int failed_pairs = 0;

    cv::Mat prev = decodeAt(mj, selected.front());
    if (prev.empty()) throw std::runtime_error("cannot decode first selected frame");

    for (size_t i = 1; i < selected.size(); ++i) {
      cv::Mat cur = decodeAt(mj, selected[i]);
      if (cur.empty()) { ++failed_pairs; continue; }

      std::vector<cv::Point2f> p0;
      cv::goodFeaturesToTrack(prev, p0, 500, 0.01, 7.0, cv::noArray(), 7, false, 0.04);
      const int detected = int(p0.size());

      std::vector<cv::Point2f> p1;
      std::vector<uchar> st;
      std::vector<float> err;
      if (!p0.empty()) {
        cv::calcOpticalFlowPyrLK(prev, cur, p0, p1, st, err, cv::Size(21,21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01));
      }

      std::vector<cv::Point2f> a, b;
      a.reserve(p0.size()); b.reserve(p0.size());
      for (size_t k = 0; k < p0.size(); ++k) {
        if (!st[k]) continue;
        if (err[k] > 30.0f) continue;
        if (p1[k].x < 1 || p1[k].y < 1 || p1[k].x >= cur.cols-1 || p1[k].y >= cur.rows-1) continue;
        a.push_back(p0[k]); b.push_back(p1[k]);
      }

      PairMetric m;
      m.t0_ns = selected[i-1].ts_ns;
      m.t1_ns = selected[i].ts_ns;
      m.t_s = (m.t1_ns - t_ref) * 1e-9;
      const int64_t tm = (m.t0_ns + m.t1_ns) / 2;
      m.phase = phaseName(tm, yaw0, yaw1);
      m.detected = detected;
      m.tracked = int(a.size());

      std::vector<double> flow;
      flow.reserve(a.size());
      for (size_t k = 0; k < a.size(); ++k) flow.push_back(cv::norm(b[k] - a[k]));
      m.median_flow_px = percentile(flow, 0.5);

      if (a.size() >= 12) {
        cv::Mat mask;
        cv::Mat H = cv::findHomography(a, b, cv::RANSAC, 1.5, mask, 2000, 0.995);
        if (!H.empty()) {
          std::vector<cv::Point2f> pred;
          cv::perspectiveTransform(a, pred, H);
          std::vector<double> residuals;
          for (size_t k = 0; k < a.size(); ++k) {
            if (mask.at<uchar>(int(k),0)) {
              ++m.h_inliers;
              residuals.push_back(cv::norm(pred[k] - b[k]));
            }
          }
          m.h_inlier_ratio = double(m.h_inliers) / double(a.size());
          m.median_h_res_px = percentile(residuals, 0.5);
          m.p90_h_res_px = percentile(residuals, 0.9);

          std::vector<cv::Mat> Rs, Ts, Ns;
          try { m.homography_solutions = cv::decomposeHomographyMat(H, K, Rs, Ts, Ns); }
          catch (...) { m.homography_solutions = 0; }
        }
      }

      csv << i << ',' << m.t0_ns << ',' << m.t1_ns << ',' << std::setprecision(9) << m.t_s << ','
          << m.phase << ',' << m.detected << ',' << m.tracked << ',' << m.h_inliers << ','
          << m.h_inlier_ratio << ',' << m.median_flow_px << ',' << m.median_h_res_px << ','
          << m.p90_h_res_px << ',' << m.homography_solutions << '\n';

      PhaseSummary* s = m.phase == "PRE" ? &pre : (m.phase == "YAW" ? &yaw : &post);
      ++s->pairs;
      s->tracked.push_back(double(m.tracked));
      if (std::isfinite(m.median_flow_px)) s->flow.push_back(m.median_flow_px);
      if (m.h_inliers > 0) {
        s->inlier_ratio.push_back(m.h_inlier_ratio);
        s->hres.push_back(m.median_h_res_px);
        s->hp90.push_back(m.p90_h_res_px);
        if (m.h_inlier_ratio >= 0.90 && m.median_h_res_px <= 0.50) ++s->near_planar;
      }
      ++valid_pairs;
      prev = cur;
    }

    printSummary(pre);
    printSummary(yaw);
    printSummary(post);

    std::cout << "\nvalid analyzed pairs: " << valid_pairs
              << "\nfailed decode pairs: " << failed_pairs
              << "\nCSV: " << outcsv << "\n";

    if (yaw.pairs > 0 && !yaw.inlier_ratio.empty()) {
      const double r = percentile(yaw.inlier_ratio, 0.5);
      const double e = percentile(yaw.hres, 0.5);
      std::cout << "\nINTERPRETATION GATE:\n";
      if (r >= 0.90 && e <= 0.50) {
        std::cout << "  YAW frames are overwhelmingly explained by one homography.\n"
                  << "  This is strong evidence of near-planar / rotation-dominated visual geometry,\n"
                  << "  where monocular translation is weakly constrained and can become highly sensitive.\n";
      } else {
        std::cout << "  YAW frames are NOT cleanly explained by one homography.\n"
                  << "  Planar degeneracy alone is not sufficient; inspect tracking/outliers/model mismatch next.\n";
      }
    }

    std::cout << "RESULT: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 2;
  }
}
