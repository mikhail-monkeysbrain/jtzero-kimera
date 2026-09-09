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
#include <atomic>
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
#include <string>
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

constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kCameraFps = 120;
constexpr int kVioFps = 30;
constexpr int kCameraBuffers = 6;
constexpr int kImuRateHz = 200;
constexpr int kAttitudeRateHz = 50;
constexpr int kRangeRateHz = 50;
constexpr int kExposureAbsolute = 50;  // UVC units: 100 us => 5 ms.
constexpr int kGain = 0;
constexpr uint8_t kCompanionSysId = 255;
constexpr uint8_t kCompanionCompId = 190;
constexpr int64_t kTimesyncPeriodNs = 100000000LL;
constexpr int64_t kMappingStaleNs = 2000000000LL;
constexpr int64_t kHudPeriodNs = 100000000LL;
constexpr int64_t kBackendReadyAgeNs = 500000000LL;
constexpr int64_t kSelectedPeriodNs = 1000000000LL / kVioFps;
constexpr int64_t kFinalHoldNs = 3000000000LL;
constexpr double kStartupStaticSec = 3.0;
constexpr size_t kStartupMinSamples = 450;
constexpr double kStartupMaxMeanGyroRadS = 0.010;
constexpr double kStartupMaxGyroStdRadS = 0.005;
constexpr double kStartupMinAccelNorm = 9.60;
constexpr double kStartupMaxAccelNorm = 10.00;
constexpr double kStartupMaxAccelNormStd = 0.080;
constexpr double kWarmupSec = 12.0;
constexpr int kWarmupStableStatesNeeded = 8;
constexpr double kWarmupMaxSpeedMps = 0.020;
constexpr double kWarmupMaxSpanM = 0.008;
constexpr double kMaxTimesyncRttMs = 10.0;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kWindow = "JT-ZERO — CLEAN-01 STANDALONE";

struct CameraBuffer { void* start=nullptr; size_t length=0; };

struct TimeSyncSample {
  int64_t t0=0,t1=0,fc_ns=0,rpi_mid=0,rtt=0;
  bool good=false;
};

struct ClockMapping {
  bool valid=false;
  long double a=1.0L;
  int64_t fc_ref=0;
  long double rpi_ref=0;
  int64_t last_update=0;
  double drift_ppm=0.0;
  int64_t map(int64_t fc_ns) const {
    return static_cast<int64_t>(std::llround(
      rpi_ref + a * static_cast<long double>(fc_ns-fc_ref)));
  }
};

struct VioState {
  int64_t timestamp_ns=0;
  int64_t callback_wall_ns=0;
  int64_t keyframe=0;
  double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
  double roll=0,pitch=0,yaw=0;
  double bgx=0,bgy=0,bgz=0,bax=0,bay=0,baz=0;
};

struct FrontendState {
  int64_t timestamp_ns=0, callback_wall_ns=0, frame_id=-1;
  int is_keyframe=0;
  size_t detected=0,tracked=0,inliers=0,putatives=0,ransac_iters=0;
  std::string mono_status;
  int mono_pose_valid=0;
  double mono_tx=0,mono_ty=0,mono_tz=0;
};

struct ImuRow {
  int64_t recv_wall_ns=0, mapped_ns=0;
  uint64_t fc_us=0;
  double raw_ax=0,raw_ay=0,raw_az=0,raw_gx=0,raw_gy=0,raw_gz=0;
  double flu_ax=0,flu_ay=0,flu_az=0,flu_gx=0,flu_gy=0,flu_gz=0;
};

struct AttRow {
  int64_t recv_wall_ns=0;
  uint32_t time_boot_ms=0;
  double roll=0,pitch=0,yaw=0;
};

struct RangeRow {
  int64_t recv_wall_ns=0;
  uint32_t time_boot_ms=0;
  uint16_t current_cm=0;
  uint8_t id=0,orientation=0,type=0,quality=0;
};

struct FrameRow {
  uint32_t sequence=0;
  int64_t timestamp_ns=0;
  uint64_t offset=0;
  uint32_t bytes=0;
};

struct SelectedFrame {
  uint32_t sequence=0;
  int64_t timestamp_ns=0;
  std::vector<unsigned char> jpeg;
};

struct EventRow {
  std::string event;
  int64_t wall_ns=0;
  int64_t state_timestamp_ns=0;
  int64_t keyframe=-1;
  double px=0,py=0,pz=0;
};


struct StartupStaticGate {
  bool passed=false;
  uint64_t first_us=0,last_us=0;
  size_t n=0;
  double sgx=0,sgy=0,sgz=0,sgx2=0,sgy2=0,sgz2=0;
  double sax=0,say=0,saz=0,san=0,san2=0;

  void reset(){
    first_us=last_us=0;n=0;
    sgx=sgy=sgz=sgx2=sgy2=sgz2=0;
    sax=say=saz=san=san2=0;
  }
  bool add(uint64_t us,double ax,double ay,double az,double gx,double gy,double gz){
    if(passed)return true;
    if(first_us==0)first_us=us;
    last_us=us;++n;
    sgx+=gx;sgy+=gy;sgz+=gz;sgx2+=gx*gx;sgy2+=gy*gy;sgz2+=gz*gz;
    sax+=ax;say+=ay;saz+=az;
    const double an=std::sqrt(ax*ax+ay*ay+az*az);san+=an;san2+=an*an;
    if(n<kStartupMinSamples)return false;
    const double elapsed=(last_us>first_us)?(last_us-first_us)*1e-6:0.0;
    if(elapsed<kStartupStaticSec)return false;
    const double dn=static_cast<double>(n);
    const double mgx=sgx/dn,mgy=sgy/dn,mgz=sgz/dn;
    const double gnorm=std::sqrt(mgx*mgx+mgy*mgy+mgz*mgz);
    const double sx=std::sqrt(std::max(0.0,sgx2/dn-mgx*mgx));
    const double sy=std::sqrt(std::max(0.0,sgy2/dn-mgy*mgy));
    const double sz=std::sqrt(std::max(0.0,sgz2/dn-mgz*mgz));
    const double man=san/dn;
    const double sanstd=std::sqrt(std::max(0.0,san2/dn-man*man));
    const bool ok=gnorm<=kStartupMaxMeanGyroRadS &&
                  std::max({sx,sy,sz})<=kStartupMaxGyroStdRadS &&
                  man>=kStartupMinAccelNorm && man<=kStartupMaxAccelNorm &&
                  sanstd<=kStartupMaxAccelNormStd;
    if(ok){passed=true;return true;}
    reset();return false;
  }
  double elapsedSec()const{
    return (first_us&&last_us>first_us)?(last_us-first_us)*1e-6:0.0;
  }
};

struct WarmupGate {
  bool started=false,passed=false;
  int64_t start_wall_ns=0,last_kf=-1;
  std::vector<VioState> stable;
  void start(int64_t now){started=true;passed=false;start_wall_ns=now;last_kf=-1;stable.clear();}
  double elapsed(int64_t now)const{return started?(now-start_wall_ns)*1e-9:0.0;}
  double speed(const VioState&s)const{return std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);}
  double span()const{
    if(stable.empty())return 0.0;
    const auto&a=stable.front();double m=0;
    for(const auto&s:stable){
      const double dx=s.px-a.px,dy=s.py-a.py,dz=s.pz-a.pz;
      m=std::max(m,std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    return m;
  }
  bool update(const VioState&s,int64_t now){
    if(passed)return true;
    if(s.keyframe==last_kf)return false;
    last_kf=s.keyframe;
    if(speed(s)>kWarmupMaxSpeedMps){stable.clear();}
    else{
      if(!stable.empty()){
        const auto&a=stable.front();
        const double dx=s.px-a.px,dy=s.py-a.py,dz=s.pz-a.pz;
        if(std::sqrt(dx*dx+dy*dy+dz*dz)>kWarmupMaxSpanM)stable.clear();
      }
      stable.push_back(s);
      if(static_cast<int>(stable.size())>kWarmupStableStatesNeeded)stable.erase(stable.begin());
    }
    if(elapsed(now)>=kWarmupSec &&
       static_cast<int>(stable.size())>=kWarmupStableStatesNeeded &&
       span()<=kWarmupMaxSpanM)passed=true;
    return passed;
  }
};

enum class Phase { STATIC_CHECK, WARMUP, READY_A, MOVING, HOLD_B, DONE, ABORTED };


[[noreturn]] void fail(const std::string& what) {
  throw std::runtime_error(what + ": " + std::strerror(errno));
}
int xioctl(int fd,unsigned long req,void* arg) {
  int r; do { r=ioctl(fd,req,arg); } while(r==-1&&errno==EINTR); return r;
}
int64_t monoNs() {
  timespec t{};
  if(clock_gettime(CLOCK_MONOTONIC,&t)!=0) fail("clock_gettime");
  return static_cast<int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;
}
int64_t timevalNs(const timeval& t) {
  return static_cast<int64_t>(t.tv_sec)*1000000000LL+
         static_cast<int64_t>(t.tv_usec)*1000LL;
}
double deg(double r){ return r*180.0/kPi; }

void ru(cv::Mat& img,const std::string& s,cv::Point p,int size,
        cv::Scalar color,int weight=cv::QT_FONT_NORMAL) {
  cv::addText(img,s,p,"DejaVu Sans",size,color,weight,cv::QT_STYLE_NORMAL,0);
}

void setCtrl(int fd,uint32_t id,int32_t value) {
  v4l2_control c{}; c.id=id; c.value=value;
  if(xioctl(fd,VIDIOC_S_CTRL,&c)==-1) {
    std::cerr<<"[КАМЕРА] предупреждение: control "<<id<<" value "<<value<<" не применён\n";
  }
}

void configureCamera(int fd) {
  v4l2_format fmt{}; fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width=kWidth; fmt.fmt.pix.height=kHeight;
  fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field=V4L2_FIELD_ANY;
  if(xioctl(fd,VIDIOC_S_FMT,&fmt)==-1) fail("VIDIOC_S_FMT");
  if(fmt.fmt.pix.width!=kWidth || fmt.fmt.pix.height!=kHeight ||
     fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG) {
    throw std::runtime_error("камера не приняла MJPEG 640x480");
  }
  v4l2_streamparm p{}; p.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
  p.parm.capture.timeperframe.numerator=1;
  p.parm.capture.timeperframe.denominator=kCameraFps;
  if(xioctl(fd,VIDIOC_S_PARM,&p)==-1) fail("VIDIOC_S_PARM");
  setCtrl(fd,V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL);
  setCtrl(fd,V4L2_CID_EXPOSURE_AUTO_PRIORITY,0);
  setCtrl(fd,V4L2_CID_EXPOSURE_ABSOLUTE,kExposureAbsolute);
  setCtrl(fd,V4L2_CID_GAIN,kGain);
  setCtrl(fd,V4L2_CID_AUTO_WHITE_BALANCE,0);
  setCtrl(fd,V4L2_CID_POWER_LINE_FREQUENCY,V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);
}

std::vector<CameraBuffer> initBuffers(int fd) {
  v4l2_requestbuffers r{}; r.count=kCameraBuffers;
  r.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; r.memory=V4L2_MEMORY_MMAP;
  if(xioctl(fd,VIDIOC_REQBUFS,&r)==-1) fail("VIDIOC_REQBUFS");
  if(r.count<2) throw std::runtime_error("слишком мало camera buffers");
  std::vector<CameraBuffer> out(r.count);
  for(uint32_t i=0;i<r.count;++i) {
    v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory=V4L2_MEMORY_MMAP; b.index=i;
    if(xioctl(fd,VIDIOC_QUERYBUF,&b)==-1) fail("VIDIOC_QUERYBUF");
    out[i].length=b.length;
    out[i].start=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
    if(out[i].start==MAP_FAILED) fail("mmap");
    if(xioctl(fd,VIDIOC_QBUF,&b)==-1) fail("VIDIOC_QBUF");
  }
  return out;
}

int openSerial(const std::string& dev) {
  int fd=open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd==-1) fail("open serial");
  termios t{};
  if(tcgetattr(fd,&t)!=0) fail("tcgetattr");
  cfmakeraw(&t);
  if(cfsetispeed(&t,B460800)||cfsetospeed(&t,B460800)) fail("baud");
  t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB;
  t.c_cflag&=~CSTOPB; t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
  t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
  if(tcsetattr(fd,TCSANOW,&t)!=0) fail("tcsetattr");
  tcflush(fd,TCIFLUSH);
  return fd;
}

void writeAll(int fd,const uint8_t* p,size_t n) {
  size_t off=0;
  while(off<n) {
    ssize_t w=write(fd,p+off,n-off);
    if(w>0){ off+=static_cast<size_t>(w); continue; }
    if(w==-1&&(errno==EAGAIN||errno==EWOULDBLOCK)){
      pollfd q{fd,POLLOUT,0}; poll(&q,1,10); continue;
    }
    if(w==-1&&errno==EINTR) continue;
    fail("serial write");
  }
}
void sendMsg(int fd,const mavlink_message_t& m) {
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  uint16_t n=mavlink_msg_to_send_buffer(b,&m); writeAll(fd,b,n);
}
void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz) {
  mavlink_message_t m{};
  float us=hz>0?static_cast<float>(1000000.0/hz):0.0f;
  mavlink_msg_command_long_pack(kCompanionSysId,kCompanionCompId,&m,sys,comp,
    MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,us,0,0,0,0,0);
  sendMsg(fd,m);
}
void sendTimesync(int fd,int64_t t0,uint8_t sys,uint8_t comp) {
  mavlink_message_t m{};
  mavlink_msg_timesync_pack(kCompanionSysId,kCompanionCompId,&m,0,t0,sys,comp);
  sendMsg(fd,m);
}
ClockMapping estimateMapping(const std::vector<TimeSyncSample>& v) {
  std::vector<const TimeSyncSample*> g;
  for(const auto& s:v) if(s.good) g.push_back(&s);
  ClockMapping m; if(g.size()<20) return m;
  const int64_t fc0=g.front()->fc_ns,r0=g.front()->rpi_mid;
  long double mx=0,my=0;
  for(const auto* s:g){mx+=s->fc_ns-fc0;my+=s->rpi_mid-r0;}
  mx/=g.size(); my/=g.size();
  long double sxx=0,sxy=0;
  for(const auto* s:g){
    long double dx=(s->fc_ns-fc0)-mx;
    long double dy=(s->rpi_mid-r0)-my;
    sxx+=dx*dx; sxy+=dx*dy;
  }
  if(sxx<=0) return m;
  m.a=sxy/sxx; m.fc_ref=fc0;
  m.rpi_ref=static_cast<long double>(r0)+(my-m.a*mx);
  m.drift_ppm=static_cast<double>((m.a-1.0L)*1e6L);
  m.valid=true; m.last_update=g.back()->t1;
  return m;
}

class CleanPipeline final : public VIO::MonoImuPipeline {
 public:
  explicit CleanPipeline(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallbacks() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out) return;
      const auto& st=out->W_State_Blkf_;
      const auto p=st.pose_.translation(),rpy=st.pose_.rotation().rpy();
      const auto& v=st.velocity_;
      const auto bg=st.imu_bias_.gyroscope(),ba=st.imu_bias_.accelerometer();
      VioState s;
      s.timestamp_ns=st.timestamp_; s.callback_wall_ns=monoNs(); s.keyframe=out->cur_kf_id_;
      s.px=p.x();s.py=p.y();s.pz=p.z();s.vx=v.x();s.vy=v.y();s.vz=v.z();
      s.roll=deg(rpy.x());s.pitch=deg(rpy.y());s.yaw=deg(rpy.z());
      s.bgx=bg.x();s.bgy=bg.y();s.bgz=bg.z();s.bax=ba.x();s.bay=ba.y();s.baz=ba.z();
      std::lock_guard<std::mutex> l(mu_); states_.push_back(s);latest_=s;have_latest_=true;
    });
    registerFrontendOutputCallback([this](const std::shared_ptr<VIO::FrontendOutputPacketBase>& out){
      if(!out) return;
      FrontendState d; d.timestamp_ns=out->timestamp_;d.callback_wall_ns=monoNs();
      d.is_keyframe=out->is_keyframe_?1:0;
      if(const auto* f=out->getTrackingFrame()) d.frame_id=f->id_;
      const auto& q=out->debug_tracker_info_;
      d.detected=q.nrDetectedFeatures_;d.tracked=q.nrTrackerFeatures_;
      d.inliers=q.nrMonoInliers_;d.putatives=q.nrMonoPutatives_;d.ransac_iters=q.monoRansacIters_;
      if(const auto* st=out->getTrackerStatus()){
        d.mono_status=VIO::TrackerStatusSummary::asString(st->kfTrackingStatus_mono_);
        if(st->kfTrackingStatus_mono_==VIO::TrackingStatus::VALID){
          const auto t=st->lkf_T_k_mono_.translation();
          d.mono_pose_valid=1;d.mono_tx=t.x();d.mono_ty=t.y();d.mono_tz=t.z();
        }
      }
      std::lock_guard<std::mutex> l(mu_); front_.push_back(std::move(d));
    });
  }
  bool latest(VioState* s) const {
    std::lock_guard<std::mutex> l(mu_);if(!have_latest_)return false;*s=latest_;return true;
  }
  std::vector<VioState> states()const{std::lock_guard<std::mutex>l(mu_);return states_;}
  std::vector<FrontendState> front()const{std::lock_guard<std::mutex>l(mu_);return front_;}
 private:
  mutable std::mutex mu_;
  bool have_latest_=false; VioState latest_;
  std::vector<VioState> states_; std::vector<FrontendState> front_;
};

std::string phaseName(Phase p){
  switch(p){
    case Phase::STATIC_CHECK:return"ПРОВЕРКА НЕПОДВИЖНОСТИ";
    case Phase::WARMUP:return"ПРОГРЕВ VIO";
    case Phase::READY_A:return"ГОТОВ К ТОЧКЕ A";
    case Phase::MOVING:return"ДВИЖЕНИЕ A → B";
    case Phase::HOLD_B:return"ФИКСАЦИЯ ТОЧКИ B";
    case Phase::DONE:return"ТЕСТ ЗАВЕРШЁН";
    case Phase::ABORTED:return"ТЕСТ ПРЕРВАН";
  }
  return"";
}
void actions(Phase p,std::string* now,std::string* next){
  switch(p){
    case Phase::STATIC_CHECK:*now="СЕЙЧАС: НЕ ДВИГАТЬ";*next="ДАЛЬШЕ: ПРОГРЕВ VIO 12 С";break;
    case Phase::WARMUP:*now="СЕЙЧАС: НЕ ДВИГАТЬ";*next="ДАЛЬШЕ: ТОЧКА A → ПРОБЕЛ";break;
    case Phase::READY_A:*now="СЕЙЧАС: ТОЧКА A → ПРОБЕЛ";*next="ДАЛЬШЕ: ДВИГАЙТЕ A → B";break;
    case Phase::MOVING:*now="СЕЙЧАС: ДВИГАЙТЕ A → B";*next="ДАЛЬШЕ: НА B → СТОП → ПРОБЕЛ";break;
    case Phase::HOLD_B:*now="СЕЙЧАС: НЕ ДВИГАТЬ";*next="ДАЛЬШЕ: ЖДИТЕ СОХРАНЕНИЯ";break;
    case Phase::DONE:*now="СЕЙЧАС: ДАННЫЕ СОХРАНЕНЫ";*next="ДАЛЬШЕ: ТЕСТ ЗАВЕРШЁН";break;
    case Phase::ABORTED:*now="СЕЙЧАС: ТЕСТ ПРЕРВАН";*next="ДАЛЬШЕ: ПРОВЕРЬТЕ ТЕРМИНАЛ";break;
  }
}

void drawGui(const cv::Mat& gray,Phase phase,const CleanPipeline& pipe,
             bool mapping_valid,double drift_ppm,size_t imu_fed,size_t frames_fed,
             int64_t hold_started,const StartupStaticGate& static_gate,
             const WarmupGate& warmup) {
  cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);
  cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));
  video.copyTo(canvas(cv::Rect(0,45,900,675)));
  ru(canvas,"JT-ZERO — CLEAN-01 STANDALONE",{22,31},20,{245,245,245},cv::QT_FONT_BOLD);

  cv::Mat panel=canvas(cv::Rect(900,0,380,720));
  ru(panel,"СОСТОЯНИЕ",{18,38},14,{190,190,190},cv::QT_FONT_BOLD);
  ru(panel,phaseName(phase),{18,78},17,phase==Phase::ABORTED?cv::Scalar(50,50,255):cv::Scalar(90,220,90),cv::QT_FONT_BOLD);

  std::string now,next;actions(phase,&now,&next);
  cv::rectangle(panel,{12,110,356,112},cv::Scalar(55,120,55),-1);
  ru(panel,now,{22,160},15,{255,255,255},cv::QT_FONT_BOLD);
  cv::rectangle(panel,{12,236,356,92},cv::Scalar(55,55,95),-1);
  ru(panel,next,{22,285},12,{245,245,245},cv::QT_FONT_BOLD);

  ru(panel,std::string("СИНХРОНИЗАЦИЯ: ")+(mapping_valid?"ГОТОВА":"ЖДИТЕ"),{18,370},12,mapping_valid?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
  char b[160];
  std::snprintf(b,sizeof(b),"дрейф часов: %+.2f ppm",drift_ppm);ru(panel,b,{18,402},11,{200,200,200});
  std::snprintf(b,sizeof(b),"IMU подано: %zu",imu_fed);ru(panel,b,{18,434},11,{200,200,200});
  std::snprintf(b,sizeof(b),"кадров VIO: %zu",frames_fed);ru(panel,b,{18,466},11,{200,200,200});
  if(phase==Phase::STATIC_CHECK){
    std::snprintf(b,sizeof(b),"статика: %.1f / %.0f с",static_gate.elapsedSec(),kStartupStaticSec);
    ru(panel,b,{18,498},11,{245,245,245},cv::QT_FONT_BOLD);
  } else if(phase==Phase::WARMUP){
    std::snprintf(b,sizeof(b),"прогрев: %.1f / %.0f с",warmup.elapsed(monoNs()),kWarmupSec);
    ru(panel,b,{18,498},11,{245,245,245},cv::QT_FONT_BOLD);
    std::snprintf(b,sizeof(b),"стабильных KF: %zu / %d",warmup.stable.size(),kWarmupStableStatesNeeded);
    ru(panel,b,{18,526},11,{245,245,245},cv::QT_FONT_BOLD);
  }

  VioState s;
  if(pipe.latest(&s)){
    std::snprintf(b,sizeof(b),"VIO KF: %lld",(long long)s.keyframe);ru(panel,b,{18,558},12,{220,220,220});
    std::snprintf(b,sizeof(b),"P: %.3f  %.3f  %.3f м",s.px,s.py,s.pz);ru(panel,b,{18,590},11,{220,220,220});
    std::snprintf(b,sizeof(b),"|V|: %.1f мм/с",1000.0*std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz));ru(panel,b,{18,622},11,{220,220,220});
  } else {
    ru(panel,"VIO: ЖДИТЕ ПЕРВОГО СОСТОЯНИЯ",{18,575},11,{0,210,255});
  }

  if(phase==Phase::HOLD_B&&hold_started>0){
    double left=std::max(0.0,(kFinalHoldNs-(monoNs()-hold_started))/1e9);
    std::snprintf(b,sizeof(b),"сохранение через %.1f с",left);ru(panel,b,{18,620},12,{245,245,245},cv::QT_FONT_BOLD);
  }
  ru(panel,"Q / ESC — аварийный выход",{18,688},11,{210,210,210});
  cv::imshow(kWindow,canvas);
}

void saveAll(const std::string& out,
             const std::vector<ImuRow>& imu,const std::vector<AttRow>& att,
             const std::vector<RangeRow>& range,const std::vector<SelectedFrame>& frames,
             const std::vector<EventRow>& events,const std::vector<VioState>& states,
             const std::vector<FrontendState>& front) {
  {
    std::ofstream f(out+"/imu.csv");f<<"recv_wall_ns,mapped_ns,fc_us,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,flu_ax,flu_ay,flu_az,flu_gx,flu_gy,flu_gz\n";
    f<<std::fixed<<std::setprecision(9);
    for(const auto&r:imu)f<<r.recv_wall_ns<<','<<r.mapped_ns<<','<<r.fc_us<<','<<r.raw_ax<<','<<r.raw_ay<<','<<r.raw_az<<','<<r.raw_gx<<','<<r.raw_gy<<','<<r.raw_gz<<','<<r.flu_ax<<','<<r.flu_ay<<','<<r.flu_az<<','<<r.flu_gx<<','<<r.flu_gy<<','<<r.flu_gz<<'\n';
  }
  {
    std::ofstream f(out+"/attitude.csv");f<<"recv_wall_ns,time_boot_ms,roll,pitch,yaw\n";f<<std::fixed<<std::setprecision(9);
    for(const auto&r:att)f<<r.recv_wall_ns<<','<<r.time_boot_ms<<','<<r.roll<<','<<r.pitch<<','<<r.yaw<<'\n';
  }
  {
    std::ofstream f(out+"/range.csv");f<<"recv_wall_ns,time_boot_ms,current_cm,id,orientation,type,quality\n";
    for(const auto&r:range)f<<r.recv_wall_ns<<','<<r.time_boot_ms<<','<<r.current_cm<<','<<(int)r.id<<','<<(int)r.orientation<<','<<(int)r.type<<','<<(int)r.quality<<'\n';
  }
  {
    std::ofstream mj(out+"/selected.mjpg",std::ios::binary);
    std::ofstream ix(out+"/selected_frames.csv");ix<<"sequence,timestamp_ns,offset,bytes\n";
    uint64_t off=0;
    for(const auto&r:frames){mj.write(reinterpret_cast<const char*>(r.jpeg.data()),r.jpeg.size());ix<<r.sequence<<','<<r.timestamp_ns<<','<<off<<','<<r.jpeg.size()<<'\n';off+=r.jpeg.size();}
  }
  {
    std::ofstream f(out+"/events.csv");f<<"event,wall_ns,state_timestamp_ns,keyframe,px,py,pz\n";f<<std::fixed<<std::setprecision(9);
    for(const auto&r:events)f<<r.event<<','<<r.wall_ns<<','<<r.state_timestamp_ns<<','<<r.keyframe<<','<<r.px<<','<<r.py<<','<<r.pz<<'\n';
  }
  {
    std::ofstream f(out+"/backend.csv");f<<"timestamp_ns,callback_wall_ns,keyframe,px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,bgx,bgy,bgz,bax,bay,baz\n";f<<std::fixed<<std::setprecision(9);
    for(const auto&r:states)f<<r.timestamp_ns<<','<<r.callback_wall_ns<<','<<r.keyframe<<','<<r.px<<','<<r.py<<','<<r.pz<<','<<r.vx<<','<<r.vy<<','<<r.vz<<','<<r.roll<<','<<r.pitch<<','<<r.yaw<<','<<r.bgx<<','<<r.bgy<<','<<r.bgz<<','<<r.bax<<','<<r.bay<<','<<r.baz<<'\n';
  }
  {
    std::ofstream f(out+"/frontend.csv");f<<"timestamp_ns,callback_wall_ns,frame_id,is_keyframe,detected,tracked,inliers,putatives,ransac_iters,mono_status,mono_pose_valid,mono_tx,mono_ty,mono_tz\n";f<<std::fixed<<std::setprecision(9);
    for(const auto&r:front)f<<r.timestamp_ns<<','<<r.callback_wall_ns<<','<<r.frame_id<<','<<r.is_keyframe<<','<<r.detected<<','<<r.tracked<<','<<r.inliers<<','<<r.putatives<<','<<r.ransac_iters<<','<<r.mono_status<<','<<r.mono_pose_valid<<','<<r.mono_tx<<','<<r.mono_ty<<','<<r.mono_tz<<'\n';
  }
}

} // namespace

int main(int argc,char** argv) {
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <params_dir> <camera_device> <serial_device> <output_dir>\n";
    return 2;
  }
  const std::string params=argv[1],camdev=argv[2],serialdev=argv[3],out=argv[4];
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;

  int cfd=-1,sfd=-1;bool streaming=false;std::vector<CameraBuffer> bufs;
  std::shared_ptr<CleanPipeline> pipe;std::thread pipe_thread;
  uint8_t sys=0,comp=0;bool rates=false;
  Phase phase=Phase::STATIC_CHECK;int64_t hold_started=0;
  StartupStaticGate static_gate;WarmupGate warmup;bool vio_feed_enabled=false;
  std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending=0,next_sync=0;
  std::vector<ImuRow> imurows;std::vector<AttRow> attrows;std::vector<RangeRow> rangerows;
  std::vector<SelectedFrame> selected;std::vector<EventRow> events;
  size_t imu_fed=0,frames_fed=0;int64_t last_selected=0,next_hud=0;
  cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
  VIO::FrameId frame_id=0;

  try{
    VIO::VioParams vp(params);
    if(vp.camera_params_.empty()) throw std::runtime_error("в params нет camera parameters");
    pipe=std::make_shared<CleanPipeline>(vp);pipe->installCallbacks();
    pipe_thread=std::thread([pipe](){pipe->spin();});

    sfd=openSerial(serialdev);
    mavlink_status_t mst{};mavlink_message_t msg{};
    int64_t hb_deadline=monoNs()+10000000000LL;
    while(monoNs()<hb_deadline&&!sys){
      pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;
      uint8_t b[4096];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;
      for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}
    }
    if(!sys) throw std::runtime_error("HEARTBEAT не получен");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttitudeRateHz);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,kRangeRateHz);
    rates=true;

    cfd=open(camdev.c_str(),O_RDWR|O_NONBLOCK);
    if(cfd==-1)fail("open camera");
    configureCamera(cfd);bufs=initBuffers(cfd);
    v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(cfd,VIDIOC_STREAMON,&typ)==-1)fail("VIDIOC_STREAMON");streaming=true;

    cv::setNumThreads(1);
    cv::namedWindow(kWindow,cv::WINDOW_NORMAL);
    cv::setWindowProperty(kWindow,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);

    next_sync=monoNs();next_hud=monoNs();
    while(phase!=Phase::DONE&&phase!=Phase::ABORTED){
      const int64_t now=monoNs();
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      if(pending&&now-pending>30000000LL)pending=0;

      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};
      int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(sfd,b,sizeof(b));
          if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            const int64_t recv=monoNs();
            if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){
              mavlink_timesync_t t{};mavlink_msg_timesync_decode(&msg,&t);
              if(t.tc1!=0&&pending!=0&&t.ts1==pending){
                TimeSyncSample s;s.t0=pending;s.t1=recv;s.fc_ns=t.tc1;s.rtt=recv-pending;s.rpi_mid=pending+s.rtt/2;s.good=s.rtt>0&&s.rtt<=static_cast<int64_t>(kMaxTimesyncRttMs*1e6);
                sync.push_back(s);pending=0;mapping=estimateMapping(sync);
              }
            } else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t m{};mavlink_msg_highres_imu_decode(&msg,&m);
              if(mapping.valid&&recv-mapping.last_update<kMappingStaleNs){
                ImuRow r;r.recv_wall_ns=recv;r.fc_us=m.time_usec;r.mapped_ns=mapping.map(static_cast<int64_t>(m.time_usec)*1000LL);
                r.raw_ax=m.xacc;r.raw_ay=m.yacc;r.raw_az=m.zacc;r.raw_gx=m.xgyro;r.raw_gy=m.ygyro;r.raw_gz=m.zgyro;
                r.flu_ax=m.xacc;r.flu_ay=-m.yacc;r.flu_az=-m.zacc;r.flu_gx=m.xgyro;r.flu_gy=-m.ygyro;r.flu_gz=-m.zgyro;
                imurows.push_back(r);
                if(!vio_feed_enabled){
                  if(static_gate.add(m.time_usec,r.flu_ax,r.flu_ay,r.flu_az,r.flu_gx,r.flu_gy,r.flu_gz)){
                    vio_feed_enabled=true;
                    warmup.start(monoNs());
                    phase=Phase::WARMUP;
                    std::cout<<"[CLEAN01] STATIC CHECK PASS — VIO FEED STARTED; 12 s WARMUP\n";
                  } else {
                    continue;
                  }
                }
                VIO::ImuAccGyr d;d<<r.flu_ax,r.flu_ay,r.flu_az,r.flu_gx,r.flu_gy,r.flu_gz;
                pipe->fillSingleImuQueue(VIO::ImuMeasurement(r.mapped_ns,d));++imu_fed;
              }
            } else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t m{};mavlink_msg_attitude_decode(&msg,&m);
              attrows.push_back({recv,m.time_boot_ms,m.roll,m.pitch,m.yaw});
            } else if(msg.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){
              mavlink_distance_sensor_t m{};mavlink_msg_distance_sensor_decode(&msg,&m);
              RangeRow r;r.recv_wall_ns=recv;r.time_boot_ms=m.time_boot_ms;r.current_cm=m.current_distance;r.id=m.id;r.orientation=m.orientation;r.type=m.type;r.quality=m.signal_quality;rangerows.push_back(r);
            }
          }
        }
      }

      if(pf[0].revents&POLLIN){
        for(;;){
          v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
          if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
          const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalNs(b.timestamp));
          std::vector<unsigned char> jpg(b.bytesused);std::memcpy(jpg.data(),bufs[b.index].start,b.bytesused);
          if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("VIDIOC_QBUF");
          if(vio_feed_enabled&&(last_selected==0||ts-last_selected>=kSelectedPeriodNs)){
            cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);
            if(!g.empty()&&g.cols==kWidth&&g.rows==kHeight){
              last_gray=g;last_selected=ts;
              selected.push_back({b.sequence,ts,std::move(jpg)});
              pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(frame_id++,ts,vp.camera_params_.at(0),g.clone()));
              ++frames_fed;
            }
          }
        }
      }

      VioState latest;
      const bool have=pipe->latest(&latest);
      if(phase==Phase::WARMUP&&mapping.valid&&have&&
         (monoNs()-latest.callback_wall_ns<kBackendReadyAgeNs)){
        if(warmup.update(latest,monoNs())){
          phase=Phase::READY_A;
          std::cout<<"[CLEAN01] WARMUP PASS — MOVEMENT ENABLED\n";
        }
      }

      if(phase==Phase::HOLD_B&&hold_started>0&&monoNs()-hold_started>=kFinalHoldNs){
        phase=Phase::DONE;
      }

      if(now>=next_hud){
        drawGui(last_gray,phase,*pipe,mapping.valid,mapping.drift_ppm,imu_fed,frames_fed,hold_started,static_gate,warmup);
        next_hud=now+kHudPeriodNs;
      }
      int key=cv::waitKey(1)&0xff;
      if(key=='q'||key=='Q'||key==27){phase=Phase::ABORTED;break;}
      if(key==' '){
        if(phase==Phase::READY_A){
          EventRow e;e.event="MOVE_START";e.wall_ns=monoNs();if(pipe->latest(&latest)){e.state_timestamp_ns=latest.timestamp_ns;e.keyframe=latest.keyframe;e.px=latest.px;e.py=latest.py;e.pz=latest.pz;}events.push_back(e);phase=Phase::MOVING;
        } else if(phase==Phase::MOVING){
          EventRow e;e.event="MOVE_END";e.wall_ns=monoNs();if(pipe->latest(&latest)){e.state_timestamp_ns=latest.timestamp_ns;e.keyframe=latest.keyframe;e.px=latest.px;e.py=latest.py;e.pz=latest.pz;}events.push_back(e);phase=Phase::HOLD_B;hold_started=monoNs();
        }
      }
    }

    if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;}
    if(rates){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,0);}
    pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();

    const auto states=pipe->states();const auto front=pipe->front();
    saveAll(out,imurows,attrows,rangerows,selected,events,states,front);
    cv::destroyAllWindows();
    for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);
    if(cfd!=-1)close(cfd);if(sfd!=-1)close(sfd);

    if(phase==Phase::ABORTED){std::cout<<"CLEAN01_STANDALONE CAPTURE_ABORTED\n";return 1;}
    const bool ok=events.size()==2&&events[0].event=="MOVE_START"&&events[1].event=="MOVE_END"&&!states.empty()&&!imurows.empty()&&!selected.empty();
    std::cout<<"CLEAN01_STANDALONE DATA_CAPTURE "<<(ok?"PASS":"FAIL")<<"\n";
    std::cout<<"CLEAN01_STANDALONE MEASUREMENT_VERDICT NOT_COMPUTED\n";
    return ok?0:1;
  } catch(const std::exception& e){
    std::cerr<<"[ОШИБКА] "<<e.what()<<"\n";
    if(pipe){pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();}
    if(streaming&&cfd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}
    for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);
    if(cfd!=-1)close(cfd);if(sfd!=-1)close(sfd);
    try{cv::destroyAllWindows();}catch(...){}
    return 2;
  }
}
