// JT-ZERO yaw diagnostic v5: compare FC attitude against Kimera R_WB during yaw.
// Russian GUI. Protocol: 10 s INIT -> yaw 80 deg -> 10 s HOLD.
// Run with JTZERO_DIAG_IMU_ONLY=1 and params/JTZeroMonoFLUZeroLever.

#define main jtzero_hud_base_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include <opencv2/freetype.hpp>

namespace {

constexpr double kInitSec = 10.0;
constexpr double kHoldSec = 10.0;
constexpr double kYawTargetDeg = 80.0;
constexpr double kDirLockDeg = 5.0;
constexpr double kYawTimeoutSec = 35.0;
constexpr int kAttRateHz = 50;
constexpr const char* kCsv = "/home/vio/jtzero_live_yaw_attitude_v5.csv";
constexpr const char* kWindow = "JT-ZERO: FC vs VIO ориентация v5";
constexpr const char* kFont = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";

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

cv::Ptr<cv::freetype::FreeType2> font() {
  static cv::Ptr<cv::freetype::FreeType2> ft = [] {
    auto p = cv::freetype::createFreeType2();
    p->loadFontData(kFont, 0);
    return p;
  }();
  return ft;
}

void text(cv::Mat& img, const std::string& s, int x, int y,
          double scale = 0.55, cv::Scalar color = {235,235,235}, int th = 1) {
  const int height = std::max(12, static_cast<int>(32.0 * scale));
  font()->putText(img, s, {x,y}, height, color, th, cv::LINE_AA, true);
}

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
  double pred_vx=0,pred_vy=0,pred_vz=0,pred_vxy=0;
};

class Pipeline final : public VIO::MonoImuPipeline {
 public:
  Pipeline(const VIO::VioParams& params, Telemetry* telemetry)
      : VIO::MonoImuPipeline(params), telemetry_(telemetry) {}

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
      const auto& pred=out->debug_info_.navstate_k_;
      const auto& pv=pred.velocity();
      s.pred_vx=pv.x(); s.pred_vy=pv.y(); s.pred_vz=pv.z();
      s.pred_vxy=std::hypot(s.pred_vx,s.pred_vy);
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
  mutable std::mutex mutex_;
  bool have_latest_=false;
  State latest_;
  std::vector<State> states_;
};

struct Ref {
  bool valid=false;
  double fc_accum=0,fc_roll=0,fc_pitch=0;
  State vio;
};

struct Deltas {
  double fc_r=0,fc_p=0,fc_y=0;
  double vio_r=0,vio_p=0,vio_y=0;
  double err_r=0,err_p=0,err_y_mag=0;
};

Deltas deltas(const State& s,const Ref& r) {
  Deltas d;
  if(!r.valid) return d;
  d.fc_r=wrapDeg(s.fc_roll-r.fc_roll);
  d.fc_p=wrapDeg(s.fc_pitch-r.fc_pitch);
  d.fc_y=s.fc_accum_yaw-r.fc_accum;
  d.vio_r=wrapDeg(s.roll-r.vio.roll);
  d.vio_p=wrapDeg(s.pitch-r.vio.pitch);
  d.vio_y=wrapDeg(s.yaw-r.vio.yaw);
  // Current observed frame mapping: roll same sign, pitch/yaw opposite sign.
  d.err_r=d.vio_r-d.fc_r;
  d.err_p=d.vio_p+d.fc_p;
  d.err_y_mag=std::abs(d.vio_y)-std::abs(d.fc_y);
  return d;
}

std::string phaseRu(const std::string& p) {
  if(p=="INIT") return "ИНИЦИАЛИЗАЦИЯ";
  if(p=="YAW") return "ВРАЩЕНИЕ YAW";
  if(p=="HOLD") return "УДЕРЖАНИЕ";
  return p;
}

void renderHud(const cv::Mat& gray,const Pipeline& pipe,const Ref& ref,
               const std::string& phase,double left,int direction,bool reversed) {
  cv::Mat bgr;
  if(gray.empty()) bgr=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,bgr,{760,570},0,0,cv::INTER_NEAREST);
  cv::Mat canvas(820,1360,CV_8UC3,cv::Scalar(12,12,12));
  bgr.copyTo(canvas(cv::Rect(0,95,760,570)));
  cv::line(canvas,{335,380},{425,380},{0,255,255},2);
  cv::line(canvas,{380,335},{380,425},{0,255,255},2);
  cv::circle(canvas,{380,380},12,{0,255,255},2);
  const cv::Scalar white(240,240,240),green(80,220,80),yellow(0,220,255),red(40,40,245);
  text(canvas,"JT-ZERO — FC против VIO: ориентация v5",24,55,.82,white,2);

  State s; const bool have=pipe.latest(&s);
  Deltas d; if(have) d=deltas(s,ref);
  cv::Mat panel=canvas(cv::Rect(760,0,600,820));
  char b[256];
  text(panel,"ФАЗА: "+phaseRu(phase),18,38,.67,phase=="YAW"?yellow:green,2);
  std::snprintf(b,sizeof(b),"Осталось: %.1f с",std::max(0.0,left)); text(panel,b,18,70,.53,white,1);

  text(panel,"ИЗМЕНЕНИЕ ОРИЕНТАЦИИ ОТ СТАРТА",18,116,.50,yellow,2);
  std::snprintf(b,sizeof(b),"FC : крен %+.2f  тангаж %+.2f  yaw %+.2f°",d.fc_r,d.fc_p,d.fc_y); text(panel,b,18,150,.47,white,1);
  std::snprintf(b,sizeof(b),"VIO: крен %+.2f  тангаж %+.2f  yaw %+.2f°",d.vio_r,d.vio_p,d.vio_y); text(panel,b,18,182,.47,white,1);
  text(panel,"ОШИБКА ПОСЛЕ СОГЛАСОВАНИЯ ЗНАКОВ",18,226,.49,yellow,2);
  const bool bad_r=std::abs(d.err_r)>2.0, bad_p=std::abs(d.err_p)>2.0, bad_y=std::abs(d.err_y_mag)>5.0;
  std::snprintf(b,sizeof(b),"Δкрен %+.2f°",d.err_r); text(panel,b,18,260,.53,bad_r?red:green,2);
  std::snprintf(b,sizeof(b),"Δтангаж %+.2f°",d.err_p); text(panel,b,18,294,.53,bad_p?red:green,2);
  std::snprintf(b,sizeof(b),"Δ|yaw| %+.2f°",d.err_y_mag); text(panel,b,18,328,.53,bad_y?red:green,2);

  if(have){
    text(panel,"IMU / PREDICTION",18,378,.50,yellow,2);
    std::snprintf(b,sizeof(b),"WORLD accel XY: %.3f м/с²",s.awxy); text(panel,b,18,412,.55,s.awxy>.5?red:green,2);
    std::snprintf(b,sizeof(b),"Vxy: %.3f м/с",std::hypot(s.vx,s.vy)); text(panel,b,18,446,.50,white,1);
    std::snprintf(b,sizeof(b),"PIM PredVxy: %.3f м/с",s.pred_vxy); text(panel,b,18,478,.50,white,1);
    std::snprintf(b,sizeof(b),"BA: %+.3f %+.3f %+.3f",s.bax,s.bay,s.baz); text(panel,b,18,510,.44,white,1);
  }

  if(ref.valid && direction!=0 && phase!="INIT") {
    const double directed=std::max(0.0,direction*d.fc_y);
    constexpr double bar_max=100.0, red_start=70.0;
    const int bx=40,by=700,bw=680,bh=34;
    const int rx=bx+(int)(bw*(red_start/bar_max));
    const int tx=bx+(int)(bw*(kYawTargetDeg/bar_max));
    cv::rectangle(canvas,{bx,by},{rx,by+bh},green,cv::FILLED);
    cv::rectangle(canvas,{rx,by},{bx+bw,by+bh},red,cv::FILLED);
    cv::rectangle(canvas,{bx,by},{bx+bw,by+bh},white,2);
    cv::line(canvas,{tx,by-10},{tx,by+bh+10},white,3);
    const int mx=bx+(int)(bw*(std::min(directed,bar_max)/bar_max));
    cv::line(canvas,{mx,by-7},{mx,by+bh+7},yellow,5);
    std::snprintf(b,sizeof(b),"YAW %.1f° / ЦЕЛЬ 80°",directed); text(canvas,b,40,690,.58,directed>=70?red:white,2);
    text(canvas,"0°",36,760,.42,white,1); text(canvas,"70°",rx-18,760,.42,white,1);
    text(canvas,"80°",tx-18,760,.42,white,2); text(canvas,"100°",bx+bw-38,760,.42,white,1);
  }

  if(phase=="INIT") text(panel,"НЕ ДВИГАТЬ",18,600,.72,green,2);
  else if(phase=="YAW") {
    const double directed=direction==0?0.0:direction*d.fc_y;
    if(direction==0) text(panel,"ВРАЩАЙТЕ YAW В ОДНУ СТОРОНУ",18,600,.55,yellow,2);
    else if(directed>=kYawTargetDeg) text(panel,"СТОП!",18,600,.82,red,3);
    else if(directed>=70.0) text(panel,"ТОРМОЗИ — ЦЕЛЬ БЛИЗКО",18,600,.65,red,3);
    else text(panel,"ПЛАВНО ВРАЩАЙТЕ ДО 80°",18,600,.58,yellow,2);
    if(reversed) text(panel,"НЕ МЕНЯЙТЕ НАПРАВЛЕНИЕ!",18,640,.58,red,2);
  } else text(panel,"СТОП. НЕ ДВИГАТЬ 10 СЕКУНД",18,610,.58,green,2);
  text(panel,"ESC / Q — прервать",18,780,.44,white,1);
  cv::imshow(kWindow,canvas);
}

void saveCsv(const std::vector<State>& ss,const Ref& ref,int64_t init_end,int64_t yaw_end) {
  std::ofstream f(kCsv,std::ios::trunc);
  f<<"phase,wall_ns,keyframe,timestamp_ns,fc_droll,fc_dpitch,fc_dyaw,vio_droll,vio_dpitch,vio_dyaw,err_roll_mapped,err_pitch_mapped,err_yaw_abs,fc_roll,fc_pitch,fc_yaw,vio_roll,vio_pitch,vio_yaw,awx,awy,awz,awxy,vxy,pred_vxy,bax,bay,baz,px,py,pz\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:ss){
    const char* ph=s.wall_ns<init_end?"INIT":(s.wall_ns<yaw_end?"YAW":"HOLD");
    const Deltas d=deltas(s,ref);
    f<<ph<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','
     <<d.fc_r<<','<<d.fc_p<<','<<d.fc_y<<','<<d.vio_r<<','<<d.vio_p<<','<<d.vio_y<<','
     <<d.err_r<<','<<d.err_p<<','<<d.err_y_mag<<','
     <<s.fc_roll<<','<<s.fc_pitch<<','<<s.fc_yaw<<','<<s.roll<<','<<s.pitch<<','<<s.yaw<<','
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
    cv::setNumThreads(1); cv::namedWindow(kWindow,cv::WINDOW_NORMAL); cv::resizeWindow(kWindow,1360,820);

    std::vector<TimeSyncSample> sync; ClockMapping mapping; int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,sel=0,imu_rx=0,imu_fed=0,att_rx=0; uint32_t prev_seq=0; int64_t prev_ts=0,last_sel=0; bool have_prev=false;
    VIO::FrameId fid=0; cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    const int64_t start=monotonicNs(),init_end=start+(int64_t)(kInitSec*1e9); int64_t yaw_end=0,hold_end=0,next_hud=start;
    Ref ref; int direction=0; bool reversed=false; double target_fc=0,target_vio=0;
    std::cout<<"\nJT-ZERO YAW ATTITUDE v5 GUI\n10 s INIT -> yaw 80 deg -> 10 s HOLD\n";

    while(true){
      const int64_t now=monotonicNs();
      if(!ref.valid&&now>=init_end){State v;bool hv=pipe->latest(&v);std::lock_guard<std::mutex>l(telemetry.mutex);if(hv&&telemetry.fc_valid){ref.valid=true;ref.fc_accum=telemetry.fc_accum_yaw;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.vio=v;std::cout<<"[YAW] reference captured\n";}}
      double fc_dyaw=0;if(ref.valid){std::lock_guard<std::mutex>l(telemetry.mutex);fc_dyaw=telemetry.fc_accum_yaw-ref.fc_accum;}
      if(ref.valid&&!yaw_end){
        if(direction==0&&std::abs(fc_dyaw)>=kDirLockDeg){direction=fc_dyaw>0?1:-1;std::cout<<"[YAW] direction locked: "<<(direction>0?"+":"-")<<"\n";}
        if(direction!=0){const double directed=direction*fc_dyaw;reversed=directed<kDirLockDeg-2.0;if(directed>=kYawTargetDeg){yaw_end=now;hold_end=now+(int64_t)(kHoldSec*1e9);target_fc=fc_dyaw;State v;if(pipe->latest(&v))target_vio=wrapDeg(v.yaw-ref.vio.yaw);std::cout<<"[YAW] target reached at FC yaw="<<std::fixed<<std::setprecision(3)<<target_fc<<" deg -> HOLD\n";}}
        if(now-init_end>(int64_t)(kYawTimeoutSec*1e9)) throw std::runtime_error("Yaw target timeout");
      }
      if(yaw_end&&now>=hold_end) break;
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}}; int rc=poll(pf,2,2); if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample q;q.t0_rpi_ns=pending;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending;q.rpi_mid_ns=pending+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);const double y=a.yaw*180.0/kPi;std::lock_guard<std::mutex>l(telemetry.mutex);if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;{std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;}if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs)continue;VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}
      }}}
      if(pending&&monotonicNs()-pending>20000000LL) pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){std::string phase=now<init_end?"INIT":(!yaw_end?"YAW":"HOLD");double left=now<init_end?(init_end-now)/1e9:(!yaw_end?std::max(0.0,kYawTimeoutSec-(now-init_end)/1e9):(hold_end-now)/1e9);renderHud(last_gray,*pipe,ref,phase,left,direction,reversed);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }

    if(!yaw_end) yaw_end=monotonicNs(); std::this_thread::sleep_for(std::chrono::milliseconds(500)); pipe->shutdown(); if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();
    const auto states=pipe->states(); saveCsv(states,ref,init_end,yaw_end);
    if(imu_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;} if(att_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}
    if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;} for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);

    double final_fc=0;{std::lock_guard<std::mutex>l(telemetry.mutex);if(ref.valid)final_fc=telemetry.fc_accum_yaw-ref.fc_accum;}
    State last{};bool hl=pipe->latest(&last);Deltas fd;if(hl)fd=deltas(last,ref);
    double dx=0,dy=0,dz=0;if(hl&&ref.valid){dx=last.px-ref.vio.px;dy=last.py-ref.vio.py;dz=last.pz-ref.vio.pz;}
    std::cout<<"\n============================================================\nJT-ZERO YAW ATTITUDE v5 RESULT\n============================================================\n"
             <<"aborted: "<<(aborted?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kCsv<<"\n"
             <<std::fixed<<std::setprecision(6)<<"TARGET FC dYaw: "<<target_fc<<" deg\nTARGET VIO dYaw: "<<target_vio<<" deg\nFINAL FC dYaw: "<<final_fc<<" deg\nFINAL VIO dYaw: "<<fd.vio_y<<" deg\nFINAL FC dRoll/dPitch: "<<fd.fc_r<<" / "<<fd.fc_p<<" deg\nFINAL VIO dRoll/dPitch: "<<fd.vio_r<<" / "<<fd.vio_p<<" deg\nFINAL mapped errors roll/pitch/yawAbs: "<<fd.err_r<<" / "<<fd.err_p<<" / "<<fd.err_y_mag<<" deg\nfinal dP: ["<<dx<<','<<dy<<','<<dz<<"] m\n";
    return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
