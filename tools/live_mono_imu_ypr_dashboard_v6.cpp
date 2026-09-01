// JT-ZERO YAW/PITCH/ROLL dashboard v6.
// Event-driven manual test: no countdown timers.
// Russian GUI with live camera, three attitude bars and automatic step transitions.
// Run with JTZERO_DIAG_IMU_ONLY=1 and params/JTZeroMonoFLUZeroLever.

#define main jtzero_hud_base_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include <opencv2/freetype.hpp>

namespace {

constexpr int kAttRateHz = 50;
constexpr int kStableSamples = 15;
constexpr double kYawTolDeg = 2.0;
constexpr double kPitchTolDeg = 2.5;
constexpr double kRollTolDeg = 2.5;
constexpr double kReturnTolDeg = 3.0;
constexpr const char* kCsv = "/home/vio/jtzero_live_ypr_dashboard_v6.csv";
constexpr const char* kWindow = "JT-ZERO: YAW PITCH ROLL v6";
constexpr const char* kFont = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";

cv::Ptr<cv::freetype::FreeType2> uiFont() {
  static cv::Ptr<cv::freetype::FreeType2> ft = [] {
    auto p = cv::freetype::createFreeType2();
    p->loadFontData(kFont, 0);
    return p;
  }();
  return ft;
}

void uiText(cv::Mat& img, const std::string& s, int x, int y,
            double scale = 0.55, cv::Scalar color = {235,235,235}, int th = 1) {
  const int height = std::max(12, static_cast<int>(32.0 * scale));
  uiFont()->putText(img, s, {x,y}, height, color, th, cv::LINE_AA, true);
}

struct Telemetry {
  mutable std::mutex mutex;
  bool fc_valid = false;
  bool have_prev_yaw = false;
  double fc_roll = 0.0;
  double fc_pitch = 0.0;
  double fc_yaw = 0.0;
  double fc_accum_yaw = 0.0;
  double prev_yaw = 0.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double gx = 0.0, gy = 0.0, gz = 0.0;
};

struct State {
  int64_t wall_ns = 0;
  int64_t timestamp_ns = 0;
  int64_t keyframe = 0;
  double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
  double roll=0,pitch=0,yaw=0;
  double bax=0,bay=0,baz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0,fc_accum_yaw=0;
  double ax=0,ay=0,az=0;
  double awx=0,awy=0,awz=0,awxy=0;
  double pred_vxy=0;
  int step = 0;
};

class Pipeline final : public VIO::MonoImuPipeline {
 public:
  Pipeline(const VIO::VioParams& params, Telemetry* telemetry)
      : VIO::MonoImuPipeline(params), telemetry_(telemetry) {}

  void setStep(int step) { step_.store(step); }

  void installCallback() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out) {
      if (!out) return;
      const auto& st = out->W_State_Blkf_;
      const auto p = st.pose_.translation();
      const auto& Rrot = st.pose_.rotation();
      const auto R = Rrot.matrix();
      const auto rpy = Rrot.rpy();
      const auto& v = st.velocity_;
      const auto ba = st.imu_bias_.accelerometer();

      State s;
      s.wall_ns = monotonicNs();
      s.timestamp_ns = st.timestamp_;
      s.keyframe = out->cur_kf_id_;
      s.px=p.x(); s.py=p.y(); s.pz=p.z();
      s.vx=v.x(); s.vy=v.y(); s.vz=v.z();
      s.roll=rpy.x()*180.0/kPi;
      s.pitch=rpy.y()*180.0/kPi;
      s.yaw=rpy.z()*180.0/kPi;
      s.bax=ba.x(); s.bay=ba.y(); s.baz=ba.z();
      s.step=step_.load();
      {
        std::lock_guard<std::mutex> l(telemetry_->mutex);
        s.fc_roll=telemetry_->fc_roll;
        s.fc_pitch=telemetry_->fc_pitch;
        s.fc_yaw=telemetry_->fc_yaw;
        s.fc_accum_yaw=telemetry_->fc_accum_yaw;
        s.ax=telemetry_->ax; s.ay=telemetry_->ay; s.az=telemetry_->az;
      }
      const gtsam::Vector3 acc_b(s.ax-s.bax,s.ay-s.bay,s.az-s.baz);
      const gtsam::Vector3 acc_w=R*acc_b+gtsam::Vector3(0.0,0.0,-9.81);
      s.awx=acc_w.x(); s.awy=acc_w.y(); s.awz=acc_w.z();
      s.awxy=std::hypot(s.awx,s.awy);
      const auto& pv=out->debug_info_.navstate_k_.velocity();
      s.pred_vxy=std::hypot(pv.x(),pv.y());
      {
        std::lock_guard<std::mutex> l(mutex_);
        latest_=s; have_latest_=true; states_.push_back(s);
      }
    });
  }

  bool latest(State* s) const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!have_latest_) return false;
    *s=latest_; return true;
  }

  std::vector<State> states() const {
    std::lock_guard<std::mutex> l(mutex_);
    return states_;
  }

 private:
  Telemetry* telemetry_;
  std::atomic<int> step_{0};
  mutable std::mutex mutex_;
  bool have_latest_=false;
  State latest_;
  std::vector<State> states_;
};

struct Ref {
  bool valid=false;
  double fc_roll=0,fc_pitch=0,fc_accum=0;
  State vio;
};

struct Att {
  double roll=0,pitch=0,yaw=0;
  double vio_roll=0,vio_pitch=0,vio_yaw=0;
};

Att relativeAtt(const State& s,const Ref& r) {
  Att a;
  if(!r.valid) return a;
  a.roll=wrapDeg(s.fc_roll-r.fc_roll);
  a.pitch=wrapDeg(s.fc_pitch-r.fc_pitch);
  a.yaw=s.fc_accum_yaw-r.fc_accum;
  a.vio_roll=wrapDeg(s.roll-r.vio.roll);
  a.vio_pitch=wrapDeg(s.pitch-r.vio.pitch);
  a.vio_yaw=wrapDeg(s.yaw-r.vio.yaw);
  return a;
}

enum class Axis { Yaw, Pitch, Roll, All };
struct StepDef {
  Axis axis;
  double target;
  double tol;
  const char* title;
  const char* instruction;
};

const std::vector<StepDef>& steps() {
  static const std::vector<StepDef> s = {
    {Axis::Yaw,   80.0, kYawTolDeg,   "YAW → +80°",   "Поверните по YAW до зелёной зоны"},
    {Axis::Pitch, 20.0, kPitchTolDeg, "PITCH → +20°", "Наклоните PITCH вверх до зелёной зоны"},
    {Axis::Pitch,-20.0, kPitchTolDeg, "PITCH → -20°", "Наклоните PITCH вниз до зелёной зоны"},
    {Axis::Roll,  20.0, kRollTolDeg,  "ROLL → +20°",  "Наклоните ROLL вправо до зелёной зоны"},
    {Axis::Roll, -20.0, kRollTolDeg,  "ROLL → -20°",  "Наклоните ROLL влево до зелёной зоны"},
    {Axis::All,    0.0, kReturnTolDeg,"ВОЗВРАТ → 0°", "Верните YAW / PITCH / ROLL к нулю"}
  };
  return s;
}

double axisValue(const Att& a, Axis axis) {
  if(axis==Axis::Yaw) return a.yaw;
  if(axis==Axis::Pitch) return a.pitch;
  if(axis==Axis::Roll) return a.roll;
  return 0.0;
}

bool inTarget(const Att& a,const StepDef& st) {
  if(st.axis==Axis::All) {
    return std::abs(a.yaw)<=st.tol && std::abs(a.pitch)<=st.tol && std::abs(a.roll)<=st.tol;
  }
  return std::abs(axisValue(a,st.axis)-st.target)<=st.tol;
}

void drawBar(cv::Mat& panel,const std::string& name,double value,double minv,double maxv,
             double target,double tol,int y,bool active,bool target_enabled=true) {
  const cv::Scalar white(235,235,235),green(80,220,80),red(50,50,170),yellow(0,220,255),gray(70,70,70);
  const int label_x=20, bar_x=142, bar_w=375, bar_h=32;
  uiText(panel,name,label_x,y+25,.53,active?yellow:white,active?2:1);
  cv::rectangle(panel,{bar_x,y},{bar_x+bar_w,y+bar_h},red,cv::FILLED);
  if(target_enabled) {
    const auto xFor=[&](double v){double t=(v-minv)/(maxv-minv);t=std::max(0.0,std::min(1.0,t));return bar_x+(int)std::lround(t*bar_w);};
    int x1=xFor(target-tol),x2=xFor(target+tol);
    cv::rectangle(panel,{x1,y},{x2,y+bar_h},green,cv::FILLED);
    cv::line(panel,{xFor(target),y-6},{xFor(target),y+bar_h+6},white,2);
    cv::line(panel,{xFor(value),y-7},{xFor(value),y+bar_h+7},yellow,4);
  } else {
    cv::rectangle(panel,{bar_x,y},{bar_x+bar_w,y+bar_h},gray,cv::FILLED);
  }
  char b[64]; std::snprintf(b,sizeof(b),"%+.1f°",value);
  uiText(panel,b,530,y+25,.53,white,2);
}

void renderHud(const cv::Mat& gray,const Pipeline& pipe,const Ref& ref,int step_idx,
               int stable_count,bool ready,bool done) {
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(155,155,155);
  cv::Mat canvas(900,1440,CV_8UC3,bg);
  cv::Mat video;
  if(gray.empty()) video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);
  video.copyTo(canvas(cv::Rect(20,80,820,615)));
  cv::rectangle(canvas,{20,80},{840,695},cv::Scalar(60,60,60),1);

  uiText(canvas,"JT-ZERO YAW PITCH ROLL TEST",20,48,.78,white,2);
  uiText(canvas,"ВИДЕО",20,74,.43,muted,1);

  State s; const bool have=pipe.latest(&s);
  Att a; if(have) a=relativeAtt(s,ref);

  if(!ready) {
    uiText(canvas,"УСТАНОВИТЕ АППАРАТ В ИСХОДНОЕ ПОЛОЖЕНИЕ",90,315,.72,white,2);
    uiText(canvas,"НЕ ДВИГАЙТЕ. Нажмите SPACE — зафиксировать ноль",100,365,.58,yellow,2);
  } else if(done) {
    uiText(canvas,"ТЕСТ ЗАВЕРШЁН",245,315,1.10,green,3);
    uiText(canvas,"Все положения зафиксированы",235,365,.62,white,2);
  } else if(step_idx>=0 && step_idx<(int)steps().size()) {
    const auto& st=steps()[step_idx];
    const bool hit=inTarget(a,st);
    if(hit) {
      uiText(canvas,"СТОП!",300,290,1.55,green,4);
      uiText(canvas,"ЗАФИКСИРУЙТЕ ПОЛОЖЕНИЕ",195,350,.75,white,2);
    } else {
      uiText(canvas,st.instruction,115,315,.68,white,2);
    }
  }

  cv::Mat side=canvas(cv::Rect(860,70,560,810));
  cv::rectangle(side,{0,0},{559,809},cv::Scalar(45,45,45),1);
  uiText(side,"ИНСТРУКЦИЯ",20,35,.57,white,2);

  if(!ready) {
    uiText(side,"ПОДГОТОВКА",20,82,.72,yellow,2);
    uiText(side,"Ноль ещё не зафиксирован",20,120,.50,white,1);
  } else if(done) {
    uiText(side,"ГОТОВО",20,82,.82,green,3);
  } else {
    const auto& st=steps()[step_idx];
    char b[128];
    std::snprintf(b,sizeof(b),"ШАГ %d/%zu: %s",step_idx+1,steps().size(),st.title);
    uiText(side,b,20,82,.68,green,2);
    uiText(side,st.instruction,20,120,.49,white,1);
    std::snprintf(b,sizeof(b),"Фиксация: %d/%d",stable_count,kStableSamples);
    uiText(side,b,20,153,.45,muted,1);
  }

  for(size_t i=0;i<steps().size();++i){
    const bool passed=ready && (done || (int)i<step_idx);
    const bool current=ready && !done && (int)i==step_idx;
    std::string line=std::to_string(i+1)+". "+steps()[i].title;
    if(passed) line="✓ "+line;
    else if(current) line="→ "+line;
    uiText(side,line,24,198+(int)i*30,.43,passed?green:(current?yellow:muted),passed||current?2:1);
  }

  uiText(side,"YAW (Z)",20,405,.52,white,2);
  double ty=0,tp=0,tr=0,yt=kYawTolDeg,pt=kPitchTolDeg,rt=kRollTolDeg;
  bool ay=false,ap=false,ar=false;
  if(ready && !done){
    const auto& st=steps()[step_idx];
    if(st.axis==Axis::Yaw){ty=st.target;yt=st.tol;ay=true;}
    if(st.axis==Axis::Pitch){tp=st.target;pt=st.tol;ap=true;}
    if(st.axis==Axis::Roll){tr=st.target;rt=st.tol;ar=true;}
    if(st.axis==Axis::All){ay=ap=ar=true;yt=pt=rt=st.tol;}
  }
  drawBar(side,"",a.yaw,-180,180,ty,yt,420,ay,ready);
  uiText(side,"PITCH (Y)",20,500,.52,white,2);
  drawBar(side,"",a.pitch,-45,45,tp,pt,515,ap,ready);
  uiText(side,"ROLL (X)",20,595,.52,white,2);
  drawBar(side,"",a.roll,-45,45,tr,rt,610,ar,ready);

  if(have){
    char b[160];
    std::snprintf(b,sizeof(b),"FC   Y %+.2f   P %+.2f   R %+.2f",a.yaw,a.pitch,a.roll); uiText(side,b,20,700,.43,white,1);
    std::snprintf(b,sizeof(b),"VIO  Y %+.2f   P %+.2f   R %+.2f",a.vio_yaw,a.vio_pitch,a.vio_roll); uiText(side,b,20,730,.43,white,1);
    std::snprintf(b,sizeof(b),"WORLD XY %.3f м/с²   Vxy %.3f м/с",s.awxy,std::hypot(s.vx,s.vy)); uiText(side,b,20,760,.40,s.awxy>.5?red:white,1);
    std::snprintf(b,sizeof(b),"PIM PredVxy %.3f м/с",s.pred_vxy); uiText(side,b,20,788,.40,white,1);
  }

  uiText(canvas,"SPACE — зафиксировать ноль   ESC / Q — прервать",25,860,.42,muted,1);
  uiText(canvas,std::string("CSV: ")+kCsv,720,860,.40,muted,1);
  cv::imshow(kWindow,canvas);
}

void saveCsv(const std::vector<State>& ss,const Ref& ref) {
  std::ofstream f(kCsv,std::ios::trunc);
  f<<"step,wall_ns,keyframe,timestamp_ns,fc_droll,fc_dpitch,fc_dyaw,vio_droll,vio_dpitch,vio_dyaw,awx,awy,awz,awxy,vxy,pred_vxy,bax,bay,baz,px,py,pz\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:ss){
    const Att a=relativeAtt(s,ref);
    f<<s.step<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','
     <<a.roll<<','<<a.pitch<<','<<a.yaw<<','<<a.vio_roll<<','<<a.vio_pitch<<','<<a.vio_yaw<<','
     <<s.awx<<','<<s.awy<<','<<s.awz<<','<<s.awxy<<','<<std::hypot(s.vx,s.vy)<<','<<s.pred_vxy<<','
     <<s.bax<<','<<s.bay<<','<<s.baz<<','<<s.px<<','<<s.py<<','<<s.pz<<'\n';
  }
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false; FLAGS_viz_type=2; FLAGS_use_lcd=false; FLAGS_log_output=false; FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";

  int cfd=-1,sfd=-1; bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0; std::vector<CameraBuffer> bufs; Telemetry telemetry; std::shared_ptr<Pipeline> pipe; std::thread pipe_thread;
  try{
    VIO::VioParams vp(params); if(vp.camera_params_.empty()) throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<Pipeline>(vp,&telemetry); pipe->installCallback(); pipe_thread=std::thread([pipe](){pipe->spin();}); pipeline_started=true;

    sfd=openSerial(); mavlink_status_t mst{}; mavlink_message_t msg{}; std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz); imu_req=true;
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz); att_req=true;

    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK); if(cfd==-1) fail("open camera"); configureCamera(cfd); bufs=initCameraBuffers(cfd);
    v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE; if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1) fail("STREAMON"); streaming=true; discardWarmup(cfd);
    cv::setNumThreads(1); cv::namedWindow(kWindow,cv::WINDOW_NORMAL); cv::resizeWindow(kWindow,1440,900);

    std::vector<TimeSyncSample> sync; ClockMapping mapping; int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,sel=0,imu_rx=0,imu_fed=0,att_rx=0; uint32_t prev_seq=0; int64_t prev_ts=0,last_sel=0; bool have_prev=false;
    VIO::FrameId fid=0; cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    Ref ref; bool ready=false,done=false; int step_idx=0,stable_count=0; int64_t next_hud=monotonicNs();
    pipe->setStep(0);

    std::cout<<"\nJT-ZERO YAW PITCH ROLL v6\n"
             <<"No timers. SPACE fixes zero. Then steps advance automatically when target is stable.\n";

    while(true){
      const int64_t now=monotonicNs();
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}}; int rc=poll(pf,2,2); if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(sfd,b,sizeof(b)); if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break; if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            const int64_t rx=monotonicNs();
            if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){
              mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);
              if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample q;q.t0_rpi_ns=pending;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending;q.rpi_mid_ns=pending+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending=0;mapping=estimateClockMapping(sync);}
            } else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              ++att_rx; mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a); const double y=a.yaw*180.0/kPi;
              {
                std::lock_guard<std::mutex>l(telemetry.mutex);
                if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);
                telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;
              }
              if(ready&&!done){
                State st; if(pipe->latest(&st)){
                  const Att rel=relativeAtt(st,ref); const auto& sd=steps()[step_idx];
                  if(inTarget(rel,sd)) ++stable_count; else stable_count=0;
                  if(stable_count>=kStableSamples){
                    std::cout<<"[STEP] fixed "<<(step_idx+1)<<"/"<<steps().size()<<": "<<sd.title<<"\n";
                    stable_count=0; ++step_idx;
                    if(step_idx>=(int)steps().size()){done=true;pipe->setStep((int)steps().size());std::cout<<"[TEST] all steps completed\n";}
                    else {pipe->setStep(step_idx);std::cout<<"[STEP] next: "<<steps()[step_idx].title<<"\n";}
                  }
                }
              }
            } else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              ++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);
              const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;
              {std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;}
              if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs)continue;
              VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;
            }
          }
        }
      }

      if(pending&&monotonicNs()-pending>20000000LL) pending=0;

      if(pf[0].revents&POLLIN){
        for(;;){
          v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
          if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}
          ++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;
          if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}
          prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;
          if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}
          if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");
        }
      }

      if(now>=next_hud){renderHud(last_gray,*pipe,ref,step_idx,stable_count,ready,done);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){aborted=true;break;}
      if(key==' '&&!ready){
        State st; bool hv=pipe->latest(&st); bool fv=false; {std::lock_guard<std::mutex>l(telemetry.mutex);fv=telemetry.fc_valid;if(hv&&fv){ref.valid=true;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.fc_accum=telemetry.fc_accum_yaw;ref.vio=st;}}
        if(ref.valid){ready=true;stable_count=0;step_idx=0;pipe->setStep(0);std::cout<<"[ZERO] reference fixed. Step 1: "<<steps()[0].title<<"\n";}
      }
      if(done){
        const int k=cv::waitKey(20)&0xff;
        if(k==27||k=='q'||k=='Q'||k==' '){break;}
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300)); pipe->shutdown(); if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();
    const auto states=pipe->states(); saveCsv(states,ref);
    if(imu_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;} if(att_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}
    if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;} for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);

    std::cout<<"\n============================================================\nJT-ZERO YPR DASHBOARD v6 RESULT\n============================================================\n"
             <<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kCsv<<"\n";
    return aborted?2:0;
  }catch(const std::exception&e){
    if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;
  }
}
