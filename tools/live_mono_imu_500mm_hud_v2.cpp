#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "common/mavlink.h"
#include "camera_imu_timestamp_policy.hpp"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_bool(visualize);
DECLARE_int32(viz_type);
DECLARE_bool(use_lcd);
DECLARE_bool(log_output);
DECLARE_bool(extract_planes_from_the_scene);

namespace {

constexpr const char* kCameraDevice = "/dev/video0";
constexpr const char* kSerialDevice = "/dev/ttyAMA0";
constexpr const char* kCsvPath = "/home/vio/jtzero_live_500mm_hud_v2.csv";
constexpr const char* kWindowName = "JT-ZERO 500 mm LIVE HUD v2";
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kCameraFps = 120;
constexpr int kCameraBuffers = 6;
constexpr int kWarmupFrames = 30;
constexpr int kImuRateHz = 200;
constexpr int kExposureAbsolute = 50;
constexpr int kGain = 0;
constexpr uint8_t kCompanionSysId = 255;
constexpr uint8_t kCompanionCompId = 190;
constexpr int64_t kTimesyncPeriodNs = 100000000LL;
constexpr int64_t kMappingStaleNs = 2000000000LL;
constexpr int64_t kHudPeriodNs = 200000000LL;  // 5 Hz: keep VNC/UI from starving Kimera.
constexpr int64_t kBackendStaleNs = 1000000000LL;
constexpr double kMaxTimesyncRttMs = 10.0;
constexpr double kStartPhaseSec = 10.0;
constexpr double kMovePhaseSec = 15.0;
constexpr double kEndPhaseSec = 10.0;
constexpr double kAverageWindowSec = 5.0;
constexpr double kExpectedDistanceM = 0.500;
constexpr double kDirectionLearnDistanceM = 0.050;
constexpr double kJumpThresholdM = 0.080;
constexpr double kPi = 3.14159265358979323846;

struct CameraBuffer { void* start = nullptr; size_t length = 0; };
struct TimeSyncSample {
  int64_t t0_rpi_ns = 0, t1_rpi_ns = 0, fc_ns = 0, rpi_mid_ns = 0, rtt_ns = 0;
  bool good = false;
};
struct ClockMapping {
  bool valid = false;
  long double a = 1.0L;
  int64_t fc_ref_ns = 0;
  long double rpi_ref_ns = 0;
  double drift_ppm = 0.0;
  int64_t last_update_ns = 0;
  int64_t map(int64_t fc_ns) const {
    return static_cast<int64_t>(std::llround(
        rpi_ref_ns + a * static_cast<long double>(fc_ns - fc_ref_ns)));
  }
};
struct VioState {
  int64_t timestamp_ns = 0, keyframe = 0, callback_wall_ns = 0;
  double px = 0, py = 0, pz = 0, vx = 0, vy = 0, vz = 0;
  double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
  double bgx = 0, bgy = 0, bgz = 0;
  double bax = 0, bay = 0, baz = 0;
};

struct FrontendDebugState {
  int64_t timestamp_ns = 0;
  int64_t callback_wall_ns = 0;
  int64_t frame_id = -1;
  bool is_keyframe = false;
  size_t detected_features = 0;
  size_t tracked_features = 0;
  size_t mono_inliers = 0;
  size_t mono_putatives = 0;
  size_t mono_ransac_iters = 0;
  double feature_detection_time_s = 0.0;
  double feature_tracking_time_s = 0.0;
  double mono_ransac_time_s = 0.0;
  std::string mono_status = "NO_STATUS";
};
struct MeanState {
  bool valid = false; size_t count = 0;
  double px = 0, py = 0, pz = 0, vx = 0, vy = 0, vz = 0;
  double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
};
struct JumpEvent {
  bool valid = false; int64_t keyframe = 0, wall_ns = 0;
  double dp_mm = 0, dt_ms = 0, speed_mm_s = 0;
};
struct HudReference {
  bool have_baseline = false, have_direction = false;
  VioState baseline;
  double dir_x = 1.0, dir_y = 0.0;
};

[[noreturn]] void fail(const std::string& s) {
  throw std::runtime_error(s + ": " + std::strerror(errno));
}
int xioctl(int fd, unsigned long req, void* arg) {
  int r; do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR); return r;
}
int64_t monotonicNs() {
  timespec t{}; if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) fail("clock_gettime");
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}
int64_t timevalToNs(const timeval& t) {
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + static_cast<int64_t>(t.tv_usec) * 1000LL;
}
double nsToMs(int64_t ns) { return static_cast<double>(ns) / 1e6; }
double wrapDeg(double d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

void setCameraControl(int fd, uint32_t id, int32_t value) {
  v4l2_control c{}; c.id = id; c.value = value; xioctl(fd, VIDIOC_S_CTRL, &c);
}
void configureCamera(int fd) {
  v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = kWidth; fmt.fmt.pix.height = kHeight;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) fail("VIDIOC_S_FMT");
  v4l2_streamparm p{}; p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  p.parm.capture.timeperframe.numerator = 1; p.parm.capture.timeperframe.denominator = kCameraFps;
  if (xioctl(fd, VIDIOC_S_PARM, &p) == -1) fail("VIDIOC_S_PARM");
  setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0);
  setCameraControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, kExposureAbsolute);
  setCameraControl(fd, V4L2_CID_GAIN, kGain);
  setCameraControl(fd, V4L2_CID_AUTO_WHITE_BALANCE, 0);
  setCameraControl(fd, V4L2_CID_POWER_LINE_FREQUENCY, V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);
  setCameraControl(fd, V4L2_CID_BACKLIGHT_COMPENSATION, 0);
}
std::vector<CameraBuffer> initCameraBuffers(int fd) {
  v4l2_requestbuffers r{}; r.count = kCameraBuffers; r.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; r.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &r) == -1) fail("VIDIOC_REQBUFS");
  std::vector<CameraBuffer> buffers(r.count);
  for (uint32_t i = 0; i < r.count; ++i) {
    v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
    if (xioctl(fd, VIDIOC_QUERYBUF, &b) == -1) fail("VIDIOC_QUERYBUF");
    buffers[i].length = b.length;
    buffers[i].start = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (buffers[i].start == MAP_FAILED) fail("mmap");
    if (xioctl(fd, VIDIOC_QBUF, &b) == -1) fail("VIDIOC_QBUF");
  }
  return buffers;
}
void discardWarmup(int fd) {
  for (int n = 0; n < kWarmupFrames;) {
    pollfd p{fd, POLLIN, 0}; if (poll(&p, 1, 1000) <= 0) continue;
    v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &b) == -1) { if (errno == EAGAIN) continue; fail("DQBUF warmup"); }
    ++n; if (xioctl(fd, VIDIOC_QBUF, &b) == -1) fail("QBUF warmup");
  }
}
int openSerial() {
  int fd = open(kSerialDevice, O_RDWR | O_NOCTTY | O_NONBLOCK); if (fd == -1) fail("open serial");
  termios t{}; if (tcgetattr(fd, &t) != 0) fail("tcgetattr"); cfmakeraw(&t);
  if (cfsetispeed(&t, B460800) || cfsetospeed(&t, B460800)) fail("baud");
  t.c_cflag |= CLOCAL | CREAD; t.c_cflag &= ~CRTSCTS; t.c_cflag &= ~PARENB; t.c_cflag &= ~CSTOPB;
  t.c_cflag &= ~CSIZE; t.c_cflag |= CS8; t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) != 0) fail("tcsetattr"); tcflush(fd, TCIFLUSH); return fd;
}
void serialWriteAll(int fd, const uint8_t* data, size_t size) {
  for (size_t pos = 0; pos < size;) {
    ssize_t n = write(fd, data + pos, size - pos);
    if (n > 0) { pos += static_cast<size_t>(n); continue; }
    if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) { pollfd q{fd, POLLOUT, 0}; poll(&q, 1, 10); continue; }
    if (n == -1 && errno == EINTR) continue; fail("serial write");
  }
}
void sendMsg(int fd, const mavlink_message_t& m) {
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN]; uint16_t n = mavlink_msg_to_send_buffer(bytes, &m); serialWriteAll(fd, bytes, n);
}
void requestRate(int fd, uint8_t sys, uint8_t comp, uint32_t msgid, int hz) {
  mavlink_message_t m{}; float interval_us = hz > 0 ? static_cast<float>(1000000.0 / hz) : 0.0f;
  mavlink_msg_command_long_pack(kCompanionSysId, kCompanionCompId, &m, sys, comp,
      MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, interval_us, 0, 0, 0, 0, 0); sendMsg(fd, m);
}
void sendTimesync(int fd, int64_t t0, uint8_t sys, uint8_t comp) {
  mavlink_message_t m{}; mavlink_msg_timesync_pack(kCompanionSysId, kCompanionCompId, &m, 0, t0, sys, comp); sendMsg(fd, m);
}
ClockMapping estimateClockMapping(const std::vector<TimeSyncSample>& samples) {
  std::vector<const TimeSyncSample*> good; for (const auto& s : samples) if (s.good) good.push_back(&s);
  ClockMapping m; if (good.size() < 20) return m;
  int64_t fc0 = good.front()->fc_ns, rpi0 = good.front()->rpi_mid_ns;
  long double mx = 0, my = 0; for (auto* s : good) { mx += s->fc_ns - fc0; my += s->rpi_mid_ns - rpi0; }
  mx /= good.size(); my /= good.size(); long double sxx = 0, sxy = 0;
  for (auto* s : good) { long double dx = (s->fc_ns - fc0) - mx; long double dy = (s->rpi_mid_ns - rpi0) - my; sxx += dx*dx; sxy += dx*dy; }
  if (sxx <= 0) return m; m.a = sxy/sxx; m.fc_ref_ns = fc0;
  m.rpi_ref_ns = static_cast<long double>(rpi0) + (my - m.a*mx);
  m.drift_ppm = static_cast<double>((m.a - 1.0L)*1e6L); m.valid = true; m.last_update_ns = good.back()->t1_rpi_ns; return m;
}

class HudPipeline final : public VIO::MonoImuPipeline {
 public:
  explicit HudPipeline(const VIO::VioParams& params) : VIO::MonoImuPipeline(params) {}
  void installBackendCallback() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out) {
      if (!out) return;
      const auto& st = out->W_State_Blkf_; const auto p = st.pose_.translation(); const auto rpy = st.pose_.rotation().rpy(); const auto& v = st.velocity_;
      const auto bg = st.imu_bias_.gyroscope();
      const auto ba = st.imu_bias_.accelerometer();
      VioState s; s.timestamp_ns = st.timestamp_; s.keyframe = out->cur_kf_id_; s.callback_wall_ns = monotonicNs();
      s.px=p.x(); s.py=p.y(); s.pz=p.z(); s.vx=v.x(); s.vy=v.y(); s.vz=v.z();
      s.roll_deg=rpy.x()*180.0/kPi; s.pitch_deg=rpy.y()*180.0/kPi; s.yaw_deg=rpy.z()*180.0/kPi;
      s.bgx=bg.x(); s.bgy=bg.y(); s.bgz=bg.z();
      s.bax=ba.x(); s.bay=ba.y(); s.baz=ba.z();
      std::lock_guard<std::mutex> lock(mutex_);
      if (!states_.empty()) {
        const auto& prev = states_.back(); double dx=s.px-prev.px,dy=s.py-prev.py,dz=s.pz-prev.pz;
        double dp=std::sqrt(dx*dx+dy*dy+dz*dz), dt=static_cast<double>(s.timestamp_ns-prev.timestamp_ns)/1e9;
        if (dp>=kJumpThresholdM && dt>0 && dt<1) {
          jump_.valid=true; jump_.keyframe=s.keyframe; jump_.dp_mm=dp*1000; jump_.dt_ms=dt*1000; jump_.speed_mm_s=dp/dt*1000; jump_.wall_ns=s.callback_wall_ns;
          std::cout << std::fixed << std::setprecision(2) << "[VIO-JUMP] kf="<<s.keyframe<<" dP="<<jump_.dp_mm<<"mm dt="<<jump_.dt_ms<<"ms speed="<<jump_.speed_mm_s<<"mm/s\n";
        }
      }
      states_.push_back(s); latest_=s; have_latest_=true;
      if ((s.keyframe%10)==0) std::cout<<std::fixed<<std::setprecision(4)<<"[VIO] kf="<<s.keyframe<<" P=["<<s.px<<','<<s.py<<','<<s.pz<<"] V=["<<s.vx<<','<<s.vy<<','<<s.vz<<"] RPYdeg=["<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<"]\n";
    });
  }
  void installFrontendDebugCallback() {
    registerFrontendOutputCallback([this](const std::shared_ptr<VIO::FrontendOutputPacketBase>& out) {
      if (!out) return;
      FrontendDebugState d;
      d.timestamp_ns = out->timestamp_;
      d.callback_wall_ns = monotonicNs();
      d.is_keyframe = out->is_keyframe_;
      if (const auto* frame = out->getTrackingFrame()) d.frame_id = frame->id_;
      const auto& info = out->debug_tracker_info_;
      d.detected_features = info.nrDetectedFeatures_;
      d.tracked_features = info.nrTrackerFeatures_;
      d.mono_inliers = info.nrMonoInliers_;
      d.mono_putatives = info.nrMonoPutatives_;
      d.mono_ransac_iters = info.monoRansacIters_;
      d.feature_detection_time_s = info.featureDetectionTime_;
      d.feature_tracking_time_s = info.featureTrackingTime_;
      d.mono_ransac_time_s = info.monoRansacTime_;
      if (const auto* status = out->getTrackerStatus()) {
        d.mono_status = VIO::TrackerStatusSummary::asString(status->kfTrackingStatus_mono_);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      frontend_states_.push_back(std::move(d));
    });
  }
  bool latest(VioState* out) const { std::lock_guard<std::mutex> l(mutex_); if(!have_latest_) return false; *out=latest_; return true; }
  JumpEvent lastJump() const { std::lock_guard<std::mutex> l(mutex_); return jump_; }
  std::vector<VioState> states() const { std::lock_guard<std::mutex> l(mutex_); return states_; }
  std::vector<FrontendDebugState> frontendStates() const { std::lock_guard<std::mutex> l(mutex_); return frontend_states_; }
 private:
  mutable std::mutex mutex_;
  bool have_latest_=false;
  VioState latest_;
  JumpEvent jump_;
  std::vector<VioState> states_;
  std::vector<FrontendDebugState> frontend_states_;
};

MeanState meanWindow(const std::vector<VioState>& states,int64_t begin_ns,int64_t end_ns) {
  MeanState m; double rr=0,pr=0,yr=0; bool ref=false;
  for(const auto&s:states){ if(s.timestamp_ns<begin_ns||s.timestamp_ns>=end_ns)continue; if(!ref){rr=s.roll_deg;pr=s.pitch_deg;yr=s.yaw_deg;ref=true;}
    m.px+=s.px;m.py+=s.py;m.pz+=s.pz;m.vx+=s.vx;m.vy+=s.vy;m.vz+=s.vz;
    m.roll_deg+=rr+wrapDeg(s.roll_deg-rr);m.pitch_deg+=pr+wrapDeg(s.pitch_deg-pr);m.yaw_deg+=yr+wrapDeg(s.yaw_deg-yr);++m.count;}
  if(!m.count)return m; double n=static_cast<double>(m.count); m.px/=n;m.py/=n;m.pz/=n;m.vx/=n;m.vy/=n;m.vz/=n;m.roll_deg/=n;m.pitch_deg/=n;m.yaw_deg/=n;m.valid=true;return m;
}
const char* phaseName(int64_t ts,int64_t start,int64_t move,int64_t end,int64_t stop){if(ts<start)return"PRE";if(ts<move)return"START";if(ts<end)return"MOVE";if(ts<stop)return"END";return"POST";}
void writeCsv(const std::vector<VioState>& states,int64_t start,int64_t move,int64_t end,int64_t stop){std::ofstream f(kCsvPath,std::ios::trunc);if(!f)return;f<<"phase,keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";f<<std::fixed<<std::setprecision(9);for(const auto&s:states)f<<phaseName(s.timestamp_ns,start,move,end,stop)<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';}

void txt(cv::Mat& img,const std::string& s,cv::Point p,double sc,cv::Scalar c,int th=2){cv::putText(img,s,p,cv::FONT_HERSHEY_SIMPLEX,sc,c,th,cv::LINE_AA);}
cv::Scalar level(double q){q=std::abs(q);if(q<.35)return{80,220,80};if(q<.7)return{0,220,255};return{40,40,245};}
void gauge(cv::Mat& panel,const std::string& name,double value,double lim,const std::string& unit,int y){int x0=28,x1=panel.cols-28,c=(x0+x1)/2;double q=std::max(-1.0,std::min(1.0,value/lim));int x=static_cast<int>(c+q*(x1-x0)*.5);char b[128];std::snprintf(b,sizeof(b),"%s %+6.2f %s",name.c_str(),value,unit.c_str());txt(panel,b,{20,y-18},.78,{250,250,250},2);cv::line(panel,{x0,y+7},{x1,y+7},{130,130,130},4);cv::line(panel,{c,y-8},{c,y+22},{240,240,240},2);cv::circle(panel,{x,y+7},11,level(q),-1,cv::LINE_AA);}
std::string phase(int64_t now,int64_t move,int64_t end,int64_t stop){if(now<move)return"START";if(now<end)return"MOVE";if(now<stop)return"END";return"DONE";}

void renderHud(const cv::Mat& gray,const HudPipeline& pipeline,HudReference* ref,const std::string& ph,int64_t now,double left){
  cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{960,720},0,0,cv::INTER_NEAREST);
  int cx=video.cols/2,cy=video.rows/2;cv::line(video,{cx-45,cy},{cx+45,cy},{0,255,255},3);cv::line(video,{cx,cy-45},{cx,cy+45},{0,255,255},3);cv::circle(video,{cx,cy},12,{0,255,255},2);
  cv::Mat panel(900,320,CV_8UC3,{24,24,24}),canvas(900,1280,CV_8UC3,{8,8,8});video.copyTo(canvas(cv::Rect(0,90,960,720)));
  txt(canvas,"JT-ZERO LIVE 500 mm",{28,52},1.12,{245,245,245},3);txt(canvas,"5 Hz HUD - camera stays 120 FPS / VIO ~30 FPS",{28,82},.55,{190,190,190},1);
  txt(panel,"PHASE "+ph,{18,43},.9,ph=="MOVE"?cv::Scalar(0,230,255):cv::Scalar(90,220,90),3);char b[160];std::snprintf(b,sizeof(b),"%.1f s left",std::max(0.0,left));txt(panel,b,{18,76},.62,{190,190,190},1);
  VioState s;if(!pipeline.latest(&s)){txt(panel,"Waiting Kimera...",{18,150},.8,{240,240,240},2);}else{
    if(!ref->have_baseline){ref->baseline=s;ref->have_baseline=true;}
    double dx=s.px-ref->baseline.px,dy=s.py-ref->baseline.py,dz=s.pz-ref->baseline.pz;
    double dr=wrapDeg(s.roll_deg-ref->baseline.roll_deg),dp=wrapDeg(s.pitch_deg-ref->baseline.pitch_deg),dyaw=wrapDeg(s.yaw_deg-ref->baseline.yaw_deg),h=std::sqrt(dx*dx+dy*dy);
    if(!ref->have_direction&&ph=="MOVE"&&h>=kDirectionLearnDistanceM){ref->dir_x=dx/h;ref->dir_y=dy/h;ref->have_direction=true;std::cout<<"[HUD] direction learned\n";}
    double travel=ref->have_direction?dx*ref->dir_x+dy*ref->dir_y:h,cross=ref->have_direction?ref->dir_x*dy-ref->dir_y*dx:0,speed=std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);
    gauge(panel,"ROLL ",dr,5,"deg",130);gauge(panel,"PITCH",dp,5,"deg",220);gauge(panel,"YAW  ",dyaw,8,"deg",310);gauge(panel,"Z    ",dz*1000,50,"mm",400);gauge(panel,"CROSS",cross*1000,50,"mm",490);
    std::snprintf(b,sizeof(b),"TRAVEL %.0f / 500 mm",travel*1000);txt(panel,b,{18,585},.78,{245,245,245},2);double prog=std::max(0.0,std::min(1.0,travel/kExpectedDistanceM));cv::rectangle(panel,{22,608,274,28},{100,100,100},2);cv::rectangle(panel,{25,611,static_cast<int>(268*prog),22},{90,210,90},-1);
    std::snprintf(b,sizeof(b),"|V| %.1f mm/s",speed*1000);txt(panel,b,{18,674},.76,{230,230,230},2);std::snprintf(b,sizeof(b),"KF %lld",(long long)s.keyframe);txt(panel,b,{18,712},.68,{200,200,200},2);
    int64_t age=now-s.callback_wall_ns;if(age>kBackendStaleNs){cv::rectangle(panel,{8,738,304,112},{20,20,220},-1);txt(panel,"BACKEND STALE",{20,780},.82,{255,255,255},3);std::snprintf(b,sizeof(b),"no state %.1f s",age/1e9);txt(panel,b,{20,818},.66,{255,255,255},2);}else{JumpEvent j=pipeline.lastJump();if(j.valid&&now-j.wall_ns<3000000000LL){cv::rectangle(panel,{8,748,304,92},{20,20,220},-1);txt(panel,"VIO JUMP",{22,790},.86,{255,255,255},3);}else{txt(panel,"Keep gauges near center",{18,785},.57,{180,180,180},1);txt(panel,"ESC / Q = abort",{18,826},.62,{220,220,220},1);}}
  }
  panel.copyTo(canvas(cv::Rect(960,0,320,900)));cv::imshow(kWindowName,canvas);
}

void printMeasurement(const MeanState&a,const MeanState&b){if(!a.valid||!b.valid){std::cout<<"MEASUREMENT RESULT: FAIL\n";return;}double dx=b.px-a.px,dy=b.py-a.py,dz=b.pz-a.pz,d=std::sqrt(dx*dx+dy*dy+dz*dz),e=d-kExpectedDistanceM,v=std::sqrt(b.vx*b.vx+b.vy*b.vy+b.vz*b.vz);std::cout<<std::fixed<<std::setprecision(6)<<"START avg states:       "<<a.count<<"\nEND avg states:         "<<b.count<<"\nmeasured dP:            ["<<dx<<','<<dy<<','<<dz<<"] m\n3D measured distance:   "<<d*1000<<" mm\nabsolute error:         "<<e*1000<<" mm\nrelative error:         "<<e/kExpectedDistanceM*100<<" %\nscale measured/true:    "<<d/kExpectedDistanceM<<"\ndRoll:                  "<<wrapDeg(b.roll_deg-a.roll_deg)<<" deg\ndPitch:                 "<<wrapDeg(b.pitch_deg-a.pitch_deg)<<" deg\ndYaw:                   "<<wrapDeg(b.yaw_deg-a.yaw_deg)<<" deg\nEND mean |V|:           "<<v*1000<<" mm/s\n";}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  std::string params=argc>1?argv[1]:"params/JTZeroMono";double total_sec=kStartPhaseSec+kMovePhaseSec+kEndPhaseSec;
  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_rate_requested=false,pipeline_started=false,aborted=false;uint8_t target_system=0,target_component=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;
  try{
    VIO::VioParams vio_params(params);if(vio_params.camera_params_.empty())throw std::runtime_error("No camera params loaded");pipeline=std::make_shared<HudPipeline>(vio_params);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t mav_status{};mavlink_message_t mav_msg{};int64_t hb_deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<hb_deadline&&!target_system){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t bytes[2048];ssize_t n=read(serial_fd,bytes,sizeof(bytes));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&mav_msg,&mav_status)&&mav_msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){target_system=mav_msg.sysid;target_component=mav_msg.compid;break;}}
    if(!target_system)throw std::runtime_error("HEARTBEAT timeout");requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_rate_requested=true;
    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);
    cv::setNumThreads(1);cv::namedWindow(kWindowName,cv::WINDOW_NORMAL);cv::resizeWindow(kWindowName,1280,900);
    std::vector<TimeSyncSample> timesync;timesync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_selected=0;bool have_prev=false;VIO::FrameId frame_id=0;
    int64_t start=monotonicNs(),move=start+(int64_t)(kStartPhaseSec*1e9),end=move+(int64_t)(kMovePhaseSec*1e9),stop=end+(int64_t)(kEndPhaseSec*1e9),next_hud=start;bool move_msg=false,end_msg=false;HudReference ref;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    while(monotonicNs()<stop){int64_t now=monotonicNs();std::string ph=phase(now,move,end,stop);int64_t ph_end=now<move?move:(now<end?end:stop);double left=(ph_end-now)/1e9;
      if(!move_msg&&now>=move){move_msg=true;std::cout<<"\n>>>>>>>> MOVE NOW: exactly 500 mm, straight line. <<<<<<<<\n";}if(!end_msg&&now>=end){end_msg=true;std::cout<<"\n>>>>>>>> STOP. END PHASE: DO NOT MOVE. <<<<<<<<\n";}
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,target_system,target_component);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t bytes[8192];for(;;){ssize_t n=read(serial_fd,bytes,sizeof(bytes));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&mav_msg,&mav_status))continue;int64_t recv=monotonicNs();if(mav_msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&mav_msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;timesync.push_back(s);pending=0;mapping=estimateClockMapping(timesync);}}else if(mav_msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t imu{};mavlink_msg_highres_imu_decode(&mav_msg,&imu);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}int64_t mapped=mapping.map((int64_t)imu.time_usec*1000LL);VIO::ImuAccGyr data;data<<imu.xacc,imu.yacc,imu.zacc,imu.xgyro,imu.ygyro,imu.zgyro;pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapped,data));++imu_fed;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;bool due=last_selected==0||corrected-last_selected>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),buffers[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){++decoded;last_gray=gray;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(frame_id++,corrected,vio_params.camera_params_.at(0),gray.clone()));last_selected=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderHud(last_gray,*pipeline,&ref,ph,now,left);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();auto states=pipeline->states();writeCsv(states,start,move,end,stop);MeanState a=meanWindow(states,move-(int64_t)(kAverageWindowSec*1e9),move),b=meanWindow(states,stop-(int64_t)(kAverageWindowSec*1e9),stop);
    if(imu_rate_requested){requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_rate_requested=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bfr:buffers)if(bfr.start&&bfr.start!=MAP_FAILED)munmap(bfr.start,bfr.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    std::cout<<"\nJT-ZERO LIVE MONO+IMU 500 MM HUD V2 RESULT\n"<<"aborted:               "<<(aborted?"yes":"no")<<"\nraw camera frames:     "<<raw<<"\nrejected raw pairs:    "<<rejected<<"\nselected frames:       "<<selected<<"\ndecoded frames:        "<<decoded<<"\nIMU received:          "<<imu_rx<<"\nIMU fed to Kimera:     "<<imu_fed<<"\nTIMESYNC samples:      "<<timesync.size()<<"\nmapping valid:         "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm:     "<<mapping.drift_ppm<<"\nbackend states:        "<<states.size()<<"\n";printMeasurement(a,b);bool pass=!aborted&&mapping.valid&&selected>=total_sec*20&&imu_fed>=total_sec*150&&states.size()>=80&&a.valid&&b.valid;std::cout<<"CSV:                    "<<kCsvPath<<"\nPIPELINE RESULT:        "<<(pass?"PASS":"FAIL")<<"\n";return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_rate_requested&&target_system){try{requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&b:buffers)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
