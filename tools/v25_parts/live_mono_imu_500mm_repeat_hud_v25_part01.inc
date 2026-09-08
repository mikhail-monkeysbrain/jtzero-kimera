// JT-ZERO 500 mm x6 v18 — startup convergence validation.
// Keeps v12 raw recording + stabilized endpoints + TF-Luna logging.
// Adds a mandatory post-startup VIO warm-up before LEG1:
//   1) 3 s static IMU startup gate,
//   2) enable fresh IMU/camera feed,
//   3) wait >=12 s while stationary,
//   4) require 8 consecutive stable backend states before measurements.
// Goal: isolate the systematic short LEG1 seen in v12 from T_BS effects.

#ifndef JTZERO_LEG_COUNT
#define JTZERO_LEG_COUNT 4
#define JTZERO_V25_OWNS_LEG_COUNT
#endif
#define JTZERO_V7_EMBEDDED
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef JTZERO_V7_EMBEDDED
#ifdef JTZERO_V25_OWNS_LEG_COUNT
#undef JTZERO_LEG_COUNT
#undef JTZERO_V25_OWNS_LEG_COUNT
#endif
#undef main

namespace {

constexpr const char* kV8CsvPath = "/home/vio/jtzero_500mm_v25_legs.csv";
constexpr const char* kV11CombinedPath = "/home/vio/jtzero_500mm_v25.csv";
constexpr const char* kV11CameraPath = "/home/vio/jtzero_500mm_v25_camera.csv";
constexpr const char* kV11MjpegPath = "/home/vio/jtzero_500mm_v25.mjpg";
constexpr const char* kV11AttitudePath = "/home/vio/jtzero_500mm_v25_attitude.csv";
constexpr const char* kV12RangePath = "/home/vio/jtzero_500mm_v25_range.csv";
constexpr const char* kV8WindowName = "JT-ZERO 500 мм A-B-A x2 v25 — closure test";
constexpr double kTruth500M = 0.500;
constexpr int kStableStatesNeeded = 4;
constexpr double kStableMaxSpeedMps = 0.020;
constexpr double kStableMaxSpanM = 0.008;
constexpr double kV8MaxLegErrorM = 0.030;
constexpr double kV8MaxStdM = 0.020;
constexpr double kV8MaxAbsDzM = 0.030;
constexpr double kV8MinPairAngleDeg = 160.0;
constexpr double kV25MaxStartSettleShiftM = 0.005;
constexpr double kV25MaxStartBackendAgeMs = 300.0;
constexpr int64_t kV25FatalLoopStallNs = 500000000LL;
bool gV25BFirst = false;

#ifdef JTZERO_V41_METRIC_CAMERA
struct V41MetricCamera {
  cv::Mat prev;
  double net_x_m=0.0, net_y_m=0.0, path_m=0.0;
  double last_step_m=0.0;
  int good_updates=0, rejected_updates=0;
  bool armed=false;
  double vertical_range_m=0.0;
  bool range_valid=false;
  double camera_height_offset_m=-0.020; // camera is 20 mm closer to ground than TF-Luna in JT-Zero rig.
  static constexpr double fx=568.53170752165227;
  static constexpr double fy=569.68005562865858;
  static constexpr double cx=315.98271077441063;
  static constexpr double cy=239.88148589100641;

  void reset(){
    prev.release(); net_x_m=net_y_m=path_m=last_step_m=0.0;
    good_updates=rejected_updates=0; armed=true;
  }
  void stop(){ armed=false; }
  double cameraHeight() const {
    return range_valid ? std::max(0.05, vertical_range_m + camera_height_offset_m) : 0.0;
  }
  double net() const { return std::hypot(net_x_m,net_y_m); }

  void update(const cv::Mat& gray){
    if(!armed || gray.empty() || !range_valid){ prev=gray.clone(); return; }
    if(prev.empty()){ prev=gray.clone(); return; }

    std::vector<cv::Point2f> p0;
    cv::goodFeaturesToTrack(prev,p0,350,0.01,7.0,cv::Mat(),7,false,0.04);
    if(p0.size()<40){ ++rejected_updates; prev=gray.clone(); return; }

    std::vector<cv::Point2f> p1; std::vector<unsigned char> st; std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,cv::Size(21,21),3);
    std::vector<cv::Point2f> a,b;
    a.reserve(p0.size()); b.reserve(p0.size());
    for(size_t i=0;i<p0.size();++i){
      if(!st[i] || err[i]>30.0f) continue;
      const float dx=p1[i].x-p0[i].x, dy=p1[i].y-p0[i].y;
      if(std::hypot(dx,dy)>80.0f) continue;
      a.emplace_back(p0[i].x-static_cast<float>(cx),p0[i].y-static_cast<float>(cy));
      b.emplace_back(p1[i].x-static_cast<float>(cx),p1[i].y-static_cast<float>(cy));
    }
    if(a.size()<30){ ++rejected_updates; prev=gray.clone(); return; }

    cv::Mat inliers;
    cv::Mat A=cv::estimateAffinePartial2D(a,b,inliers,cv::RANSAC,2.0,2000,0.99,10);
    int nin=0; if(!inliers.empty()) nin=cv::countNonZero(inliers);
    if(A.empty() || nin<20){ ++rejected_updates; prev=gray.clone(); return; }

    const double tx=A.at<double>(0,2), ty=A.at<double>(1,2);
    const double h=cameraHeight();
    // Ground translation is opposite to image motion. Magnitude is independent of camera/body axis naming.
    const double dx_m=-tx*h/fx;
    const double dy_m=-ty*h/fy;
    const double step=std::hypot(dx_m,dy_m);
    // Reject impossible one-frame jumps (>30 mm at ~30 fps).
    if(!std::isfinite(step) || step>0.030){ ++rejected_updates; prev=gray.clone(); return; }
    net_x_m+=dx_m; net_y_m+=dy_m; path_m+=step; last_step_m=step; ++good_updates;
    prev=gray.clone();
  }
};
V41MetricCamera gV41Cam;
static std::atomic<bool> gV41MouseSpace{false};
static void v41MouseCallback(int event,int,int,int,void*){
  if(event==cv::EVENT_LBUTTONDOWN) gV41MouseSpace.store(true);
}
#endif

// Controlled repeatability mode: four measured A->B passes.
// Returns B->A happen while WAIT_START and are intentionally not measured.
bool gV26ControlledAb4 = false;
int64_t gV26MoveStartWallNs = 0;
double gV26TargetMoveSec = 7.5;
double gV26DurationToleranceSec = 1.0;
double gV26ReferenceMoveSec = 7.5;
std::array<double,4> gV26MoveDurationsSec{{0.0,0.0,0.0,0.0}};

constexpr double kStartupStaticSec = 3.0;
constexpr size_t kStartupMinSamples = 450;
constexpr double kStartupMaxMeanGyroRadS = 0.010;
constexpr double kStartupMaxGyroStdRadS = 0.005;
constexpr double kStartupMinAccelNorm = 9.60;
constexpr double kStartupMaxAccelNorm = 10.00;
constexpr double kStartupMaxAccelNormStd = 0.080;
constexpr double kPostStartupWarmupSec = 12.0;
constexpr int kWarmupStableStatesNeeded = 8;
constexpr double kWarmupMaxSpeedMps = 0.020;
constexpr double kWarmupMaxSpanM = 0.008;
constexpr int64_t kWarmupLogPeriodNs = 1000000000LL;

struct StartupStaticGate {
  bool passed=false;
  uint64_t first_us=0,last_us=0;
  size_t n=0;
  Eigen::Vector3d sum_g=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_g2=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_a=Eigen::Vector3d::Zero();
  double sum_an=0.0,sum_an2=0.0;

  void reset(){
    first_us=last_us=0;n=0;
    sum_g.setZero();sum_g2.setZero();sum_a.setZero();
    sum_an=sum_an2=0.0;
  }

  bool add(uint64_t us,const Eigen::Vector3d&a,const Eigen::Vector3d&g){
    if(passed)return true;
    if(first_us==0)first_us=us;
    last_us=us;++n;
    sum_g+=g;sum_g2+=g.cwiseProduct(g);sum_a+=a;
    const double an=a.norm();sum_an+=an;sum_an2+=an*an;

    if(n<kStartupMinSamples)return false;
    const double elapsed=(last_us>first_us)?(last_us-first_us)*1e-6:0.0;
    if(elapsed<kStartupStaticSec)return false;

    const double dn=static_cast<double>(n);
    const Eigen::Vector3d mg=sum_g/dn;
    const Eigen::Vector3d vg=(sum_g2/dn-mg.cwiseProduct(mg)).cwiseMax(0.0);
    const Eigen::Vector3d sg=vg.cwiseSqrt();
    const Eigen::Vector3d ma=sum_a/dn;
    const double man=sum_an/dn;
    const double san=std::sqrt(std::max(0.0,sum_an2/dn-man*man));

    const bool ok=
      mg.norm()<=kStartupMaxMeanGyroRadS &&
      sg.maxCoeff()<=kStartupMaxGyroStdRadS &&
