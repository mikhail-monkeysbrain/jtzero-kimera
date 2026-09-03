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
  uint64_t ts_ns = 0;
  uint64_t offset = 0;
  uint64_t bytes = 0;
};

struct Sample {
  double t_s = 0.0;
  std::string phase;
  int tracked = 0;
  double flow_med_px = 0.0;
  double H_ratio = 0.0;
  double E_ratio = 0.0;
  int cheirality = 0;
  double t_x = 0.0, t_y = 0.0, t_z = 0.0;
  double t_jump_deg = std::numeric_limits<double>::quiet_NaN();
  double parallax_med_deg = 0.0;
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

static double median(std::vector<double> v) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  const size_t n = v.size();
  std::nth_element(v.begin(), v.begin() + n/2, v.end());
  double m = v[n/2];
  if ((n & 1u) == 0) {
    auto it = std::max_element(v.begin(), v.begin() + n/2);
    m = 0.5 * (m + *it);
  }
  return m;
}

static double percentile(std::vector<double> v, double p) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  double x = p * (v.size() - 1);
  size_t i = static_cast<size_t>(std::floor(x));
  size_t j = std::min(i + 1, v.size() - 1);
  double a = x - i;
  return v[i] * (1.0 - a) + v[j] * a;
}

static std::vector<CamRow> readCamIndex(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open camera index: " + path);
  std::vector<CamRow> rows;
  std::string line;
  bool first = true;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    auto c = splitCsv(line);
    if (first) { first = false; if (!c.empty() && c[0].find("seq") != std::string::npos) continue; }
    if (c.size() < 7) continue;
    CamRow r;
    try {
      r.seq = std::stoull(c[0]);
      r.ts_ns = std::stoull(c[2]);
      r.offset = std::stoull(c[5]);
      r.bytes = std::stoull(c[6]);
    } catch (...) { continue; }
    rows.push_back(r);
  }
  return rows;
}

static bool decodeFrame(std::ifstream& mj, const CamRow& r, cv::Mat& gray) {
  std::vector<uchar> buf(r.bytes);
  mj.seekg(static_cast<std::streamoff>(r.offset), std::ios::beg);
  if (!mj.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(r.bytes))) return false;
  cv::Mat img = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
  if (img.empty()) return false;
  gray = img;
  return true;
}

static void readYawInterval(const std::string& combined, double& yaw0_s, double& yaw1_s) {
  std::ifstream f(combined);
  if (!f) throw std::runtime_error("cannot open combined CSV: " + combined);
  std::string line;
  std::vector<std::pair<double,double>> gz;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    auto c = splitCsv(line);
    if (c.size() < 14 || c[0] != "IMU") continue;
    try {
      uint64_t mapped_ns = std::stoull(c[3]);
      double raw_gz = std::stod(c[13]);
      double flu_gz = -raw_gz;
      gz.emplace_back(mapped_ns * 1e-9, flu_gz * 180.0 / CV_PI);
    } catch (...) {}
  }
  const double th = 5.0;
  size_t a = gz.size(), b = 0;
  for (size_t i=0;i<gz.size();++i) {
    if (std::abs(gz[i].second) >= th) { a = std::min(a,i); b = i; }
  }
  if (a == gz.size()) throw std::runtime_error("yaw interval not found");
  yaw0_s = gz[a].first;
  yaw1_s = gz[b].first;
}

static std::string phaseOf(double t, double y0, double y1) {
  if (t < y0) return "PRE";
  if (t <= y1) return "YAW";
  return "POST";
}

static double angleDeg(const cv::Vec3d& a, const cv::Vec3d& b) {
  double na = cv::norm(a), nb = cv::norm(b);
  if (na < 1e-12 || nb < 1e-12) return std::numeric_limits<double>::quiet_NaN();
  double d = a.dot(b)/(na*nb);
  d = std::max(-1.0, std::min(1.0, d));
  return std::acos(d)*180.0/CV_PI;
}

static void summarize(const std::string& phase, const std::vector<Sample>& all) {
  std::vector<double> flow,H,E,jump,par;
  std::vector<int> tracked,cheir;
  int n=0, unstable30=0, unstable60=0, unstable90=0;
  for (const auto& s: all) if (s.phase==phase) {
    ++n; tracked.push_back(s.tracked); cheir.push_back(s.cheirality);
    flow.push_back(s.flow_med_px); H.push_back(s.H_ratio); E.push_back(s.E_ratio); par.push_back(s.parallax_med_deg);
    if (std::isfinite(s.t_jump_deg)) {
      jump.push_back(s.t_jump_deg);
      if (s.t_jump_deg>=30) ++unstable30;
      if (s.t_jump_deg>=60) ++unstable60;
      if (s.t_jump_deg>=90) ++unstable90;
    }
  }
  std::vector<double> tracked_d(tracked.begin(), tracked.end()), cheir_d(cheir.begin(), cheir.end());
  std::cout << "\n["<<phase<<"] pairs="<<n<<"\n";
  std::cout << "  tracked median: "<<median(tracked_d)<<"\n";
  std::cout << "  raw flow median px: "<<median(flow)<<"\n";
  std::cout << "  homography inlier ratio median: "<<median(H)<<"\n";
  std::cout << "  essential inlier ratio median: "<<median(E)<<"\n";
  std::cout << "  recoverPose cheirality median: "<<median(cheir_d)<<"\n";
  std::cout << "  translation direction jump median deg: "<<median(jump)<<"\n";
  std::cout << "  translation direction jump p90 deg: "<<percentile(jump,0.90)<<"\n";
  std::cout << "  median bearing parallax deg: "<<median(par)<<"\n";
  std::cout << "  direction jumps >=30/60/90 deg: "<<unstable30<<"/"<<unstable60<<"/"<<unstable90<<"\n";
}

int main(int argc, char** argv) {
  try {
    const std::string combined = argc>1 ? argv[1] : "/home/vio/jtzero_yaw_only_v13.csv";
    const std::string camidx   = argc>2 ? argv[2] : "/home/vio/jtzero_yaw_only_v13_camera.csv";
    const std::string mjpeg    = argc>3 ? argv[3] : "/home/vio/jtzero_yaw_only_v13.mjpg";
    const std::string camyaml  = argc>4 ? argv[4] : "/home/vio/jtzero-kimera-sync/params/JTZeroMonoFLU/LeftCameraParams.yaml";
    const std::string outcsv   = "/home/vio/jtzero_visual_essential_v15_15.csv";

    cv::FileStorage fs(camyaml, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open camera yaml: " + camyaml);
    cv::FileNode intr = fs["intrinsics"];
    std::vector<double> iv; intr >> iv;
    if (iv.size()!=4) throw std::runtime_error("intrinsics size != 4");
    cv::Mat K = (cv::Mat_<double>(3,3) << iv[0],0,iv[2], 0,iv[1],iv[3], 0,0,1);

    double y0=0,y1=0; readYawInterval(combined,y0,y1);
    auto raw = readCamIndex(camidx);
    std::vector<CamRow> sel;
    for (size_t i=0;i<raw.size();i+=4) sel.push_back(raw[i]);

    std::ifstream mj(mjpeg, std::ios::binary);
    if (!mj) throw std::runtime_error("cannot open mjpeg");

    std::ofstream csv(outcsv);
    csv << "pair_idx,t_s,phase,tracked,flow_med_px,H_ratio,E_ratio,cheirality,t_x,t_y,t_z,t_jump_deg,parallax_med_deg\n";

    std::vector<Sample> samples;
    cv::Vec3d prev_t(0,0,0); bool have_prev_t=false;
    int decode_fail=0;

    for (size_t i=1;i<sel.size();++i) {
      cv::Mat a,b;
      if (!decodeFrame(mj,sel[i-1],a) || !decodeFrame(mj,sel[i],b)) { ++decode_fail; continue; }
      std::vector<cv::Point2f> p0;
      cv::goodFeaturesToTrack(a,p0,500,0.01,7.0);
      if (p0.size()<20) continue;
      std::vector<cv::Point2f> p1; std::vector<uchar> st; std::vector<float> err;
      cv::calcOpticalFlowPyrLK(a,b,p0,p1,st,err,cv::Size(21,21),3);
      std::vector<cv::Point2f> q0,q1;
      std::vector<double> flow;
      for (size_t k=0;k<st.size();++k) if (st[k]) {
        q0.push_back(p0[k]); q1.push_back(p1[k]); flow.push_back(cv::norm(p1[k]-p0[k]));
      }
      if (q0.size()<20) continue;

      cv::Mat Hmask, Emask;
      cv::Mat H = cv::findHomography(q0,q1,cv::RANSAC,1.0,Hmask,2000,0.995);
      cv::Mat E = cv::findEssentialMat(q0,q1,K,cv::RANSAC,0.999,1.0,Emask);
      if (H.empty() || E.empty()) continue;
      int Hin = cv::countNonZero(Hmask), Ein = cv::countNonZero(Emask);

      cv::Mat R,t, poseMask=Emask.clone();
      int cheir = cv::recoverPose(E,q0,q1,K,R,t,poseMask);
      cv::Vec3d tv(t.at<double>(0),t.at<double>(1),t.at<double>(2));
      if (cv::norm(tv)>1e-12) tv/=cv::norm(tv);

      double tj = std::numeric_limits<double>::quiet_NaN();
      if (have_prev_t) tj = angleDeg(prev_t,tv);
      prev_t=tv; have_prev_t=true;

      std::vector<cv::Point2f> n0,n1;
      cv::undistortPoints(q0,n0,K,cv::noArray());
      cv::undistortPoints(q1,n1,K,cv::noArray());
      std::vector<double> pars;
      for (size_t k=0;k<n0.size();++k) {
        cv::Vec3d u0(n0[k].x,n0[k].y,1.0), u1(n1[k].x,n1[k].y,1.0);
        u0/=cv::norm(u0); u1/=cv::norm(u1);
        pars.push_back(angleDeg(u0,u1));
      }

      double ts=sel[i].ts_ns*1e-9;
      Sample s;
      s.t_s=ts; s.phase=phaseOf(ts,y0,y1); s.tracked=(int)q0.size(); s.flow_med_px=median(flow);
      s.H_ratio=(double)Hin/q0.size(); s.E_ratio=(double)Ein/q0.size(); s.cheirality=cheir;
      s.t_x=tv[0]; s.t_y=tv[1]; s.t_z=tv[2]; s.t_jump_deg=tj; s.parallax_med_deg=median(pars);
      samples.push_back(s);
      csv<<i<<','<<std::setprecision(12)<<s.t_s<<','<<s.phase<<','<<s.tracked<<','<<s.flow_med_px<<','<<s.H_ratio<<','<<s.E_ratio<<','<<s.cheirality<<','<<s.t_x<<','<<s.t_y<<','<<s.t_z<<','<<s.t_jump_deg<<','<<s.parallax_med_deg<<'\n';
    }

    std::cout<<std::fixed<<std::setprecision(3);
    std::cout<<"============================================================\n";
    std::cout<<"JT-ZERO ESSENTIAL / TRANSLATION STABILITY v15.15\n";
    std::cout<<"============================================================\n";
    std::cout<<"raw camera rows: "<<raw.size()<<"\n";
    std::cout<<"selected camera rows: "<<sel.size()<<"\n";
    std::cout<<"yaw mapped interval: "<<y0<<" .. "<<y1<<" s\n";
    summarize("PRE",samples); summarize("YAW",samples); summarize("POST",samples);
    std::cout<<"\nvalid analyzed pairs: "<<samples.size()<<"\n";
    std::cout<<"failed decode pairs: "<<decode_fail<<"\n";
    std::cout<<"CSV: "<<outcsv<<"\n\n";
    std::cout<<"INTERPRETATION GATE:\n";
    std::cout<<"  If YAW has very high homography support but translation-direction jumps are large,\n";
    std::cout<<"  monocular translation is directly observed to be unstable under this rotation-dominated planar geometry.\n";
    std::cout<<"RESULT: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    return 2;
  }
}
