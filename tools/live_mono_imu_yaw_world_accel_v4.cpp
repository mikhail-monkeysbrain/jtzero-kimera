// JT-ZERO yaw diagnostic v4: world-acceleration / gravity projection.
// Russian GUI. Protocol: 10 s INIT -> lock yaw direction -> 80 deg -> 10 s HOLD.
// Run with JTZERO_DIAG_IMU_ONLY=1 and params/JTZeroMonoFLUZeroLever.

#define main jtzero_hud_base_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include <opencv2/freetype.hpp>

namespace {

constexpr double kV4InitSec = 10.0;
constexpr double kV4HoldSec = 10.0;
constexpr double kV4YawTriggerDeg = 80.0;
constexpr double kV4DirectionLockDeg = 5.0;
constexpr double kV4YawTimeoutSec = 35.0;
constexpr double kV4RpLimitDeg = 3.0;
constexpr int kV4AttitudeRateHz = 50;
constexpr const char* kV4Csv = "/home/vio/jtzero_live_yaw_world_accel_v4.csv";
constexpr const char* kV4Window = "JT-ZERO: диагностика ускорения WORLD v4";
constexpr const char* kV4Font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";

struct V3Telemetry {
  mutable std::mutex mutex;
  bool fc_valid = false;
  bool raw_valid = false;
  bool have_prev_yaw = false;
  double fc_roll = 0.0;
  double fc_pitch = 0.0;
  double fc_yaw = 0.0;
  double fc_accum_yaw = 0.0;
  double prev_yaw = 0.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double gx = 0.0, gy = 0.0, gz = 0.0;
};

cv::Ptr<cv::freetype::FreeType2> v4Font() {
  static cv::Ptr<cv::freetype::FreeType2> ft = [] {
    auto p = cv::freetype::createFreeType2();
    p->loadFontData(kV4Font, 0);
    return p;
  }();
  return ft;
}

void v3Text(cv::Mat& img, const std::string& s, int x, int y,
            double scale = 0.55, cv::Scalar color = {235,235,235}, int th = 1) {
  const int height = std::max(12, static_cast<int>(32.0 * scale));
  v4Font()->putText(img, s, {x,y}, height, color, th, cv::LINE_AA, true);
}

struct V4State {
  int64_t wall_ns = 0;
  int64_t timestamp_ns = 0;
  int64_t keyframe = 0;
  double px = 0, py = 0, pz = 0;
  double vx = 0, vy = 0, vz = 0;
  double roll = 0, pitch = 0, yaw = 0;
  double bax = 0, bay = 0, baz = 0;
  double bgx = 0, bgy = 0, bgz = 0;
  double fc_roll = 0, fc_pitch = 0, fc_yaw = 0, fc_accum_yaw = 0;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
  double acx = 0, acy = 0, acz = 0;
  double awx = 0, awy = 0, awz = 0;
  double aw_xy = 0;
  double r00 = 1, r01 = 0, r02 = 0;
  double r10 = 0, r11 = 1, r12 = 0;
  double r20 = 0, r21 = 0, r22 = 1;
  double pred_px = 0, pred_py = 0, pred_pz = 0;
  double pred_vx = 0, pred_vy = 0, pred_vz = 0;
  double pred_vxy = 0;
  double pim_droll = 0, pim_dpitch = 0, pim_dyaw = 0;
};

class V4Pipeline final : public VIO::MonoImuPipeline {
 public:
  V4Pipeline(const VIO::VioParams& params, V3Telemetry* telemetry)
      : VIO::MonoImuPipeline(params), telemetry_(telemetry) {}

  void installBackendCallback() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out) {
      if (!out) return;
      const auto& st = out->W_State_Blkf_;
      const auto p = st.pose_.translation();
      const auto& Rrot = st.pose_.rotation();
      const auto R = Rrot.matrix();
      const auto rpy = Rrot.rpy();
      const auto& v = st.velocity_;
      const auto ba = st.imu_bias_.accelerometer();
      const auto bg = st.imu_bias_.gyroscope();

      V4State s;
      s.wall_ns = monotonicNs();
      s.timestamp_ns = st.timestamp_;
      s.keyframe = out->cur_kf_id_;
      s.px = p.x(); s.py = p.y(); s.pz = p.z();
      s.vx = v.x(); s.vy = v.y(); s.vz = v.z();
      s.roll = rpy.x() * 180.0 / kPi;
      s.pitch = rpy.y() * 180.0 / kPi;
      s.yaw = rpy.z() * 180.0 / kPi;
      s.bax = ba.x(); s.bay = ba.y(); s.baz = ba.z();
      s.bgx = bg.x(); s.bgy = bg.y(); s.bgz = bg.z();

      {
        std::lock_guard<std::mutex> lock(telemetry_->mutex);
        s.fc_roll = telemetry_->fc_roll;
        s.fc_pitch = telemetry_->fc_pitch;
        s.fc_yaw = telemetry_->fc_yaw;
        s.fc_accum_yaw = telemetry_->fc_accum_yaw;
        s.ax = telemetry_->ax; s.ay = telemetry_->ay; s.az = telemetry_->az;
        s.gx = telemetry_->gx; s.gy = telemetry_->gy; s.gz = telemetry_->gz;
      }

      s.acx = s.ax - s.bax;
      s.acy = s.ay - s.bay;
      s.acz = s.az - s.baz;
      const gtsam::Vector3 a_body(s.acx, s.acy, s.acz);
      const gtsam::Vector3 gravity_w(0.0, 0.0, -9.81);
      const gtsam::Vector3 a_world = R * a_body + gravity_w;
      s.awx = a_world.x(); s.awy = a_world.y(); s.awz = a_world.z();
      s.aw_xy = std::hypot(s.awx, s.awy);

      s.r00=R(0,0); s.r01=R(0,1); s.r02=R(0,2);
      s.r10=R(1,0); s.r11=R(1,1); s.r12=R(1,2);
      s.r20=R(2,0); s.r21=R(2,1); s.r22=R(2,2);

      const auto& pred = out->debug_info_.navstate_k_;
      const auto pp = pred.pose().translation();
      const auto& pv = pred.velocity();
      s.pred_px=pp.x(); s.pred_py=pp.y(); s.pred_pz=pp.z();
      s.pred_vx=pv.x(); s.pred_vy=pv.y(); s.pred_vz=pv.z();
      s.pred_vxy=std::hypot(s.pred_vx,s.pred_vy);
      const auto pim_rpy = out->debug_info_.imuR_lkf_kf.rpy();
      s.pim_droll=pim_rpy.x()*180.0/kPi;
      s.pim_dpitch=pim_rpy.y()*180.0/kPi;
      s.pim_dyaw=pim_rpy.z()*180.0/kPi;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        states_.push_back(s);
        latest_ = s;
        have_latest_ = true;
      }

      if ((s.keyframe % 5) == 0) {
        std::cout << std::fixed << std::setprecision(5)
                  << "[V4] kf=" << s.keyframe
                  << " FCyaw=" << s.fc_accum_yaw
                  << " Araw=[" << s.ax << ',' << s.ay << ',' << s.az << ']'
                  << " Acorr=[" << s.acx << ',' << s.acy << ',' << s.acz << ']'
                  << " Aw=[" << s.awx << ',' << s.awy << ',' << s.awz << ']'
                  << " |Awxy|=" << s.aw_xy
                  << " Vxy=" << std::hypot(s.vx,s.vy)
                  << " PredVxy=" << s.pred_vxy
                  << " BA=[" << s.bax << ',' << s.bay << ',' << s.baz << "]\n";
      }
    });
  }

  bool latest(V4State* out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_latest_) return false;
    *out = latest_;
    return true;
  }

  std::vector<V4State> states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_;
  }

 private:
  V3Telemetry* telemetry_;
  mutable std::mutex mutex_;
  bool have_latest_ = false;
  V4State latest_;
  std::vector<V4State> states_;
};

struct V4Refs {
  bool valid = false;
  double fc_accum = 0;
  double fc_roll = 0;
  double fc_pitch = 0;
  V4State vio;
};

std::string v4PhaseRu(const std::string& p) {
  if (p=="INIT") return "ИНИЦИАЛИЗАЦИЯ";
  if (p=="YAW") return "ВРАЩЕНИЕ YAW";
  if (p=="HOLD") return "УДЕРЖАНИЕ";
  return p;
}

void renderV4Hud(const cv::Mat& gray,
                 const V4Pipeline& pipe,
                 const V3Telemetry& telem,
                 const V4Refs& ref,
                 const std::string& phase,
                 double seconds_left,
                 int direction,
                 bool reversed,
                 double max_xy,
                 double max_vxy,
                 double max_awxy) {
  cv::Mat bgr;
  if (gray.empty()) bgr=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,bgr,{760,570},0,0,cv::INTER_NEAREST);
  cv::Mat canvas(800,1360,CV_8UC3,cv::Scalar(12,12,12));
  bgr.copyTo(canvas(cv::Rect(0,95,760,570)));
  cv::line(canvas,{335,380},{425,380},{0,255,255},2);
  cv::line(canvas,{380,335},{380,425},{0,255,255},2);
  cv::circle(canvas,{380,380},12,{0,255,255},2);
  v3Text(canvas,"JT-ZERO — WORLD-ускорение при YAW v4",24,55,.82,{245,245,245},2);

  cv::Mat panel=canvas(cv::Rect(760,0,600,800));
  const cv::Scalar white(240,240,240), green(80,220,80), yellow(0,220,255), red(40,40,245);
  double fr=0,fp=0,facc=0;
  { std::lock_guard<std::mutex> l(telem.mutex); fr=telem.fc_roll; fp=telem.fc_pitch; facc=telem.fc_accum_yaw; }
  const double dyaw=ref.valid?facc-ref.fc_accum:0;
  const double dr=ref.valid?wrapDeg(fr-ref.fc_roll):0;
  const double dp=ref.valid?wrapDeg(fp-ref.fc_pitch):0;
  const bool rp_bad=std::abs(dr)>kV4RpLimitDeg||std::abs(dp)>kV4RpLimitDeg;

  V4State s; const bool have=pipe.latest(&s);
  double dpx=0,dpy=0,false_xy=0,vxy=0,vdyaw=0;
  if(have&&ref.valid){
    dpx=s.px-ref.vio.px;
    dpy=s.py-ref.vio.py;
    false_xy=std::hypot(dpx,dpy);
    vxy=std::hypot(s.vx,s.vy);
  }
  if(have&&ref.valid) vdyaw=wrapDeg(s.yaw-ref.vio.yaw);

  char b[256];
  v3Text(panel,"ФАЗА: "+v4PhaseRu(phase),18,38,.67,phase=="YAW"?yellow:green,2);
  std::snprintf(b,sizeof(b),"Осталось: %.1f с",std::max(0.0,seconds_left));v3Text(panel,b,18,70,.53,white,1);
  std::snprintf(b,sizeof(b),"YAW FC: %+.1f°   VIO: %+.1f°",dyaw,vdyaw);v3Text(panel,b,18,108,.57,white,2);
  std::snprintf(b,sizeof(b),"Крен %+.1f°   Тангаж %+.1f°",dr,dp);v3Text(panel,b,18,140,.52,rp_bad?red:white,2);
  std::snprintf(b,sizeof(b),"XY %.0f мм   Vxy %.0f мм/с",false_xy*1000,vxy*1000);v3Text(panel,b,18,178,.55,false_xy>.20?red:white,2);

  if(have){
    v3Text(panel,"УСКОРЕНИЕ FLU [м/с²]",18,220,.51,yellow,2);
    std::snprintf(b,sizeof(b),"RAW    %+.3f  %+.3f  %+.3f",s.ax,s.ay,s.az);v3Text(panel,b,18,250,.46,white,1);
    std::snprintf(b,sizeof(b),"RAW-BA %+.3f  %+.3f  %+.3f",s.acx,s.acy,s.acz);v3Text(panel,b,18,278,.46,white,1);
    v3Text(panel,"WORLD = R_WB*(RAW-BA)+g",18,318,.51,yellow,2);
    std::snprintf(b,sizeof(b),"X %+.3f   Y %+.3f   Z %+.3f",s.awx,s.awy,s.awz);v3Text(panel,b,18,349,.49,s.aw_xy>.5?red:white,2);
    std::snprintf(b,sizeof(b),"|WORLD XY| = %.3f м/с²",s.aw_xy);v3Text(panel,b,18,382,.57,s.aw_xy>.5?red:green,2);
    std::snprintf(b,sizeof(b),"Макс |WORLD XY| = %.3f",max_awxy);v3Text(panel,b,18,412,.47,white,1);

    v3Text(panel,"СОСТОЯНИЕ BACKEND",18,455,.51,yellow,2);
    std::snprintf(b,sizeof(b),"BA %+.3f %+.3f %+.3f",s.bax,s.bay,s.baz);v3Text(panel,b,18,484,.45,white,1);
    std::snprintf(b,sizeof(b),"Vxy opt %.3f   pred %.3f м/с",vxy,s.pred_vxy);v3Text(panel,b,18,512,.45,white,1);
    std::snprintf(b,sizeof(b),"PIM dRPY %+.2f %+.2f %+.2f°",s.pim_droll,s.pim_dpitch,s.pim_dyaw);v3Text(panel,b,18,540,.45,white,1);
    std::snprintf(b,sizeof(b),"Макс XY %.0f мм   Макс Vxy %.0f",max_xy*1000,max_vxy*1000);v3Text(panel,b,18,568,.44,white,1);
  }

  if(phase=="INIT") v3Text(panel,"НЕ ДВИГАТЬ",18,640,.72,green,2);
  else if(phase=="YAW"){
    if(direction==0) v3Text(panel,"ВРАЩАЙТЕ YAW В ОДНУ СТОРОНУ",18,640,.55,yellow,2);
    else {
      const double directed=direction*dyaw;
      std::snprintf(b,sizeof(b),"ДО ЦЕЛИ: %.1f°",std::max(0.0,kV4YawTriggerDeg-directed));v3Text(panel,b,18,640,.62,yellow,2);
      v3Text(panel,direction>0?"НАПРАВЛЕНИЕ ЗАФИКСИРОВАНО: +YAW":"НАПРАВЛЕНИЕ ЗАФИКСИРОВАНО: -YAW",18,674,.47,white,1);
    }
    if(reversed) v3Text(panel,"НЕ МЕНЯЙТЕ НАПРАВЛЕНИЕ!",18,714,.60,red,2);
    else v3Text(panel,"НА 80° СРАЗУ ОСТАНОВИТЕСЬ",18,714,.52,yellow,2);
  } else v3Text(panel,"СТОП. НЕ ДВИГАТЬ 10 СЕКУНД",18,660,.58,green,2);
  v3Text(panel,"ESC / Q — прервать",18,765,.44,white,1);
  cv::imshow(kV4Window,canvas);
}

void saveV4Csv(const std::vector<V4State>& ss,int64_t init_end,int64_t yaw_end){
  std::ofstream f(kV4Csv,std::ios::trunc);
  f<<"phase,wall_ns,keyframe,timestamp_ns,px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,bax,bay,baz,bgx,bgy,bgz,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,fc_accum_yaw_deg,ax_flu,ay_flu,az_flu,gx_flu,gy_flu,gz_flu,acx,acy,acz,awx,awy,awz,aw_xy,r00,r01,r02,r10,r11,r12,r20,r21,r22,pred_px,pred_py,pred_pz,pred_vx,pred_vy,pred_vz,pred_vxy,pim_droll,pim_dpitch,pim_dyaw\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:ss){const char*ph=s.wall_ns<init_end?"INIT":(s.wall_ns<yaw_end?"YAW":"HOLD");
    f<<ph<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll<<','<<s.pitch<<','<<s.yaw<<','<<s.bax<<','<<s.bay<<','<<s.baz<<','<<s.bgx<<','<<s.bgy<<','<<s.bgz<<','<<s.fc_roll<<','<<s.fc_pitch<<','<<s.fc_yaw<<','<<s.fc_accum_yaw<<','<<s.ax<<','<<s.ay<<','<<s.az<<','<<s.gx<<','<<s.gy<<','<<s.gz<<','<<s.acx<<','<<s.acy<<','<<s.acz<<','<<s.awx<<','<<s.awy<<','<<s.awz<<','<<s.aw_xy<<','<<s.r00<<','<<s.r01<<','<<s.r02<<','<<s.r10<<','<<s.r11<<','<<s.r12<<','<<s.r20<<','<<s.r21<<','<<s.r22<<','<<s.pred_px<<','<<s.pred_py<<','<<s.pred_pz<<','<<s.pred_vx<<','<<s.pred_vy<<','<<s.pred_vz<<','<<s.pred_vxy<<','<<s.pim_droll<<','<<s.pim_dpitch<<','<<s.pim_dyaw<<'\n';}
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";

  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;V3Telemetry telemetry;std::shared_ptr<V4Pipeline>pipe;std::thread pipe_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<V4Pipeline>(vp,&telemetry);pipe->installBackendCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kV4AttitudeRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kV4Window,cv::WINDOW_NORMAL);cv::resizeWindow(kV4Window,1360,800);

    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rej=0,sel=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    const int64_t start=monotonicNs(),init_end=start+(int64_t)(kV4InitSec*1e9);int64_t yaw_end=0,hold_end=0,next_hud=start;V4Refs ref;double max_droll=0,max_dpitch=0,max_xy=0,max_vxy=0,max_awxy=0,target_fc_yaw=0,target_vio_yaw=0;int direction=0;bool reversed=false;
    std::cout<<"\nJT-ZERO YAW WORLD-ACCEL v4 GUI\n10 s INIT -> yaw 80 deg -> 10 s HOLD\n";

    while(true){
      const int64_t now=monotonicNs();
      if(!ref.valid&&now>=init_end){V4State v;bool have_v=pipe->latest(&v);std::lock_guard<std::mutex>l(telemetry.mutex);if(have_v&&telemetry.fc_valid){ref.valid=true;ref.fc_accum=telemetry.fc_accum_yaw;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.vio=v;std::cout<<"[YAW] reference captured\n";}}
      double cur_fc_dyaw=0,cur_dr=0,cur_dp=0;if(ref.valid){std::lock_guard<std::mutex>l(telemetry.mutex);cur_fc_dyaw=telemetry.fc_accum_yaw-ref.fc_accum;cur_dr=wrapDeg(telemetry.fc_roll-ref.fc_roll);cur_dp=wrapDeg(telemetry.fc_pitch-ref.fc_pitch);max_droll=std::max(max_droll,std::abs(cur_dr));max_dpitch=std::max(max_dpitch,std::abs(cur_dp));}
      if(ref.valid&&!yaw_end){
        if(direction==0&&std::abs(cur_fc_dyaw)>=kV4DirectionLockDeg){direction=cur_fc_dyaw>0?1:-1;std::cout<<"[YAW] direction locked: "<<(direction>0?"+":"-")<<"\n";}
        if(direction!=0){const double directed=direction*cur_fc_dyaw;reversed=directed<kV4DirectionLockDeg-2.0;if(directed>=kV4YawTriggerDeg){yaw_end=now;hold_end=now+(int64_t)(kV4HoldSec*1e9);target_fc_yaw=cur_fc_dyaw;V4State v;if(pipe->latest(&v))target_vio_yaw=wrapDeg(v.yaw-ref.vio.yaw);std::cout<<"[YAW] target reached at FC yaw="<<std::fixed<<std::setprecision(3)<<target_fc_yaw<<" deg -> HOLD\n";}}
        if(now-init_end>(int64_t)(kV4YawTimeoutSec*1e9))throw std::runtime_error("Yaw target timeout");
      }
      if(yaw_end&&now>=hold_end)break;
      if(ref.valid){V4State v;if(pipe->latest(&v)){max_xy=std::max(max_xy,std::hypot(v.px-ref.vio.px,v.py-ref.vio.py));max_vxy=std::max(max_vxy,std::hypot(v.vx,v.vy));max_awxy=std::max(max_awxy,v.aw_xy);}}
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=rx;s.fc_ns=ts.tc1;s.rtt_ns=rx-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);const double y=a.yaw*180.0/kPi;std::lock_guard<std::mutex>l(telemetry.mutex);if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;{std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;telemetry.raw_valid=true;}if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rej;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){std::string phase=now<init_end?"INIT":(!yaw_end?"YAW":"HOLD");double left=now<init_end?(init_end-now)/1e9:(!yaw_end?std::max(0.0,kV4YawTimeoutSec-(now-init_end)/1e9):(hold_end-now)/1e9);renderV4Hud(last_gray,*pipe,telemetry,ref,phase,left,direction,reversed,max_xy,max_vxy,max_awxy);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }

    if(!yaw_end)yaw_end=monotonicNs();std::this_thread::sleep_for(std::chrono::milliseconds(500));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipe->states();saveV4Csv(states,init_end,yaw_end);
    if(imu_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);

    double final_fc=0;{std::lock_guard<std::mutex>l(telemetry.mutex);if(ref.valid)final_fc=telemetry.fc_accum_yaw-ref.fc_accum;}V4State last{};bool have_last=pipe->latest(&last);double final_xy=0,dx=0,dy=0,dz=0,final_vio_yaw=0;if(ref.valid&&have_last){dx=last.px-ref.vio.px;dy=last.py-ref.vio.py;dz=last.pz-ref.vio.pz;final_xy=std::hypot(dx,dy);final_vio_yaw=wrapDeg(last.yaw-ref.vio.yaw);}std::cout<<"\n============================================================\nJT-ZERO YAW WORLD-ACCEL v4 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kV4Csv<<"\n"<<std::fixed<<std::setprecision(6)<<"TARGET FC dYaw: "<<target_fc_yaw<<" deg\nTARGET VIO dYaw: "<<target_vio_yaw<<" deg\nFINAL FC dYaw: "<<final_fc<<" deg\nFINAL VIO dYaw: "<<final_vio_yaw<<" deg\nmax FC |dRoll|: "<<max_droll<<" deg\nmax FC |dPitch|: "<<max_dpitch<<" deg\nmax false XY: "<<max_xy*1000.0<<" mm\nfinal false XY: "<<final_xy*1000.0<<" mm\nmax Vxy: "<<max_vxy*1000.0<<" mm/s\nmax |WORLD accel XY|: "<<max_awxy<<" m/s^2\nfinal dP: ["<<dx<<','<<dy<<','<<dz<<"] m\n";if(!states.empty()){const auto&a=states.front(),&b=states.back();std::cout<<"FIRST BA=["<<a.bax<<','<<a.bay<<','<<a.baz<<"] Aw=["<<a.awx<<','<<a.awy<<','<<a.awz<<"]\nLAST  BA=["<<b.bax<<','<<b.bay<<','<<b.baz<<"] Aw=["<<b.awx<<','<<b.awy<<','<<b.awz<<"]\n";}return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
