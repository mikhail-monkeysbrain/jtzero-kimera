// JT-ZERO 500 mm x6 v18 — startup convergence validation.
// Keeps v12 raw recording + stabilized endpoints + TF-Luna logging.
// Adds a mandatory post-startup VIO warm-up before LEG1:
//   1) 3 s static IMU startup gate,
//   2) enable fresh IMU/camera feed,
//   3) wait >=12 s while stationary,
//   4) require 8 consecutive stable backend states before measurements.
// Goal: isolate the systematic short LEG1 seen in v12 from T_BS effects.

#define JTZERO_LEG_COUNT 2
#define JTZERO_V7_EMBEDDED
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef JTZERO_V7_EMBEDDED
#undef JTZERO_LEG_COUNT
#undef main

namespace {

constexpr const char* kV8CsvPath = "/home/vio/jtzero_500mm_v23_legs.csv";
constexpr const char* kV11CombinedPath = "/home/vio/jtzero_500mm_v23.csv";
constexpr const char* kV11CameraPath = "/home/vio/jtzero_500mm_v23_camera.csv";
constexpr const char* kV11MjpegPath = "/home/vio/jtzero_500mm_v23.mjpg";
constexpr const char* kV11AttitudePath = "/home/vio/jtzero_500mm_v23_attitude.csv";
constexpr const char* kV12RangePath = "/home/vio/jtzero_500mm_v23_range.csv";
constexpr const char* kV8WindowName = "JT-ZERO 500 мм x2 v23 — strict start gate";
constexpr double kTruth500M = 0.500;
constexpr int kStableStatesNeeded = 4;
constexpr double kStableMaxSpeedMps = 0.020;
constexpr double kStableMaxSpanM = 0.008;
constexpr double kV8MaxLegErrorM = 0.030;
constexpr double kV8MaxStdM = 0.020;
constexpr double kV8MaxAbsDzM = 0.030;
constexpr double kV8MinPairAngleDeg = 160.0;
constexpr double kV23MaxStartSettleShiftM = 0.005;
constexpr double kV23MaxStartBackendAgeMs = 300.0;
constexpr int64_t kV23FatalLoopStallNs = 500000000LL;

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
