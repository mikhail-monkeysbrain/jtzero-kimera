// JT-ZERO yaw IMU-only diagnostic v3 with full GUI telemetry.
// Protocol: 10 s INIT -> yaw until accumulated FC yaw reaches 88 deg -> 10 s HOLD.
// Run with JTZERO_DIAG_IMU_ONLY=1 and params/JTZeroMonoFLUZeroLever.

#define main jtzero_hud_base_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include <future>

namespace {

constexpr int kV3AttitudeRateHz = 50;
constexpr double kV3InitSec = 10.0;
constexpr double kV3HoldSec = 10.0;
constexpr double kV3YawTriggerDeg = 88.0;
constexpr double kV3YawTimeoutSec = 35.0;
constexpr double kV3RpLimitDeg = 2.0;
constexpr const char* kV3Csv = "/home/vio/jtzero_live_yaw_imu_only_v3.csv";
constexpr const char* kV3Window = "JT-ZERO YAW IMU-ONLY v3";

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

struct V3State {
  int64_t wall_ns = 0;
  int64_t timestamp_ns = 0;
  int64_t keyframe = 0;
  double px = 0.0, py = 0.0, pz = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  double bax = 0.0, bay = 0.0, baz = 0.0;
  double bgx = 0.0, bgy = 0.0, bgz = 0.0;
  double fc_roll = 0.0, fc_pitch = 0.0, fc_yaw = 0.0, fc_accum_yaw = 0.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double gx = 0.0, gy = 0.0, gz = 0.0;
};

class V3Pipeline final : public VIO::MonoImuPipeline {
 public:
  V3Pipeline(const VIO::VioParams& params, V3Telemetry* telemetry)
      : VIO::MonoImuPipeline(params), telemetry_(telemetry) {}

  void installBackendCallback() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out) {
      if (!out) return;
      const auto& st = out->W_State_Blkf_;
      const auto p = st.pose_.translation();
      const auto rpy = st.pose_.rotation().rpy();
      const auto& v = st.velocity_;
      const auto ba = st.imu_bias_.accelerometer();
      const auto bg = st.imu_bias_.gyroscope();

      V3State s;
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

      {
        std::lock_guard<std::mutex> lock(mutex_);
        states_.push_back(s);
        latest_ = s;
        have_latest_ = true;
      }

      if ((s.keyframe % 5) == 0) {
        std::cout << std::fixed << std::setprecision(5)
                  << "[V3] kf=" << s.keyframe
                  << " FCaccYaw=" << s.fc_accum_yaw
                  << " P=[" << s.px << ',' << s.py << ',' << s.pz << ']'
                  << " V=[" << s.vx << ',' << s.vy << ',' << s.vz << ']'
                  << " BA=[" << s.bax << ',' << s.bay << ',' << s.baz << ']'
                  << " BG=[" << s.bgx << ',' << s.bgy << ',' << s.bgz << ']'
                  << " Aflu=[" << s.ax << ',' << s.ay << ',' << s.az << ']'
                  << " Gflu=[" << s.gx << ',' << s.gy << ',' << s.gz << "]\n";
      }
    });
  }

  bool latest(V3State* out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_latest_) return false;
    *out = latest_;
    return true;
  }

  std::vector<V3State> states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_;
  }

 private:
  V3Telemetry* telemetry_;
  mutable std::mutex mutex_;
  bool have_latest_ = false;
  V3State latest_;
  std::vector<V3State> states_;
};

struct V3Refs {
  bool valid = false;
  double fc_accum = 0.0;
  double fc_roll = 0.0;
  double fc_pitch = 0.0;
  V3State vio;
};

void v3Text(cv::Mat& img, const std::string& s, int x, int y,
            double scale = 0.55, cv::Scalar color = {235,235,235}, int th = 1) {
  txt(img, s, {x,y}, scale, color, th);
}

void renderV3Hud(const cv::Mat& gray,
                 const V3Pipeline& pipeline,
                 const V3Telemetry& telemetry,
                 const V3Refs& ref,
                 const std::string& phase,
                 double seconds_left,
                 double max_false_xy,
                 double max_vxy) {
  cv::Mat bgr;
  if (gray.empty()) bgr = cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,bgr,{800,600},0,0,cv::INTER_NEAREST);

  cv::Mat canvas(760,1280,CV_8UC3,cv::Scalar(12,12,12));
  bgr.copyTo(canvas(cv::Rect(0,80,800,600)));
  cv::line(canvas,{355,380},{445,380},{0,255,255},2);
  cv::line(canvas,{400,335},{400,425},{0,255,255},2);
  cv::circle(canvas,{400,380},12,{0,255,255},2);
  txt(canvas,"JT-ZERO YAW IMU-ONLY v3",{24,48},1.0,{245,245,245},3);

  cv::Mat panel = canvas(cv::Rect(800,0,480,760));
  const cv::Scalar white(240,240,240), green(80,220,80), yellow(0,220,255), red(40,40,245);

  double fr=0,fp=0,fy=0,facc=0,ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  {
    std::lock_guard<std::mutex> lock(telemetry.mutex);
    fr=telemetry.fc_roll; fp=telemetry.fc_pitch; fy=telemetry.fc_yaw;
    facc=telemetry.fc_accum_yaw;
    ax=telemetry.ax; ay=telemetry.ay; az=telemetry.az;
    gx=telemetry.gx; gy=telemetry.gy; gz=telemetry.gz;
  }

  const double fdyaw = ref.valid ? facc-ref.fc_accum : 0.0;
  const double fdroll = ref.valid ? wrapDeg(fr-ref.fc_roll) : 0.0;
  const double fdpitch = ref.valid ? wrapDeg(fp-ref.fc_pitch) : 0.0;
  const bool rp_bad = std::abs(fdroll)>kV3RpLimitDeg || std::abs(fdpitch)>kV3RpLimitDeg;

  V3State s;
  const bool have = pipeline.latest(&s);
  double dx=0,dy=0,dz=0,false_xy=0,vxy=0,vio_dyaw=0;
  if (have && ref.valid) {
    dx=s.px-ref.vio.px; dy=s.py-ref.vio.py; dz=s.pz-ref.vio.pz;
    false_xy=std::hypot(dx,dy);
    vxy=std::hypot(s.vx,s.vy);
    vio_dyaw=wrapDeg(s.yaw-ref.vio.yaw);
  }

  txt(panel,"PHASE: "+phase,{18,38},.82,phase=="YAW"?yellow:green,2);
  char b[192];
  std::snprintf(b,sizeof(b),"%.1f s",std::max(0.0,seconds_left)); v3Text(panel,b,18,68,.58,white,1);
  std::snprintf(b,sizeof(b),"FC ACC YAW  %+.1f deg",fdyaw); v3Text(panel,b,18,105,.67,white,2);
  std::snprintf(b,sizeof(b),"VIO YAW     %+.1f deg",vio_dyaw); v3Text(panel,b,18,138,.62,white,2);
  std::snprintf(b,sizeof(b),"dROLL %+.1f   dPITCH %+.1f",fdroll,fdpitch); v3Text(panel,b,18,171,.58,rp_bad?red:white,2);
  std::snprintf(b,sizeof(b),"FALSE XY %.1f mm",false_xy*1000.0); v3Text(panel,b,18,210,.67,false_xy>0.06?red:white,2);
  std::snprintf(b,sizeof(b),"Vxy %.1f mm/s",vxy*1000.0); v3Text(panel,b,18,242,.60,vxy>0.2?red:white,2);
  std::snprintf(b,sizeof(b),"MAX XY %.1f  MAX Vxy %.1f",max_false_xy*1000.0,max_vxy*1000.0); v3Text(panel,b,18,272,.50,white,1);

  if (have) {
    v3Text(panel,"ACC BIAS [m/s2]",18,312,.58,yellow,2);
    std::snprintf(b,sizeof(b),"X %+.5f  Y %+.5f  Z %+.5f",s.bax,s.bay,s.baz); v3Text(panel,b,18,342,.48,white,1);
    v3Text(panel,"GYRO BIAS [rad/s]",18,382,.58,yellow,2);
    std::snprintf(b,sizeof(b),"X %+.6f",s.bgx); v3Text(panel,b,18,410,.51,white,1);
    std::snprintf(b,sizeof(b),"Y %+.6f",s.bgy); v3Text(panel,b,18,435,.51,white,1);
    std::snprintf(b,sizeof(b),"Z %+.6f",s.bgz); v3Text(panel,b,18,460,.51,white,1);
  }

  v3Text(panel,"RAW FLU ACC",18,500,.58,yellow,2);
  std::snprintf(b,sizeof(b),"%+.3f  %+.3f  %+.3f",ax,ay,az); v3Text(panel,b,18,530,.50,white,1);
  v3Text(panel,"RAW FLU GYRO",18,570,.58,yellow,2);
  std::snprintf(b,sizeof(b),"%+.4f  %+.4f  %+.4f",gx,gy,gz); v3Text(panel,b,18,600,.50,white,1);

  if (phase=="INIT") v3Text(panel,"HOLD STILL",18,655,.72,green,2);
  else if (phase=="YAW") {
    std::snprintf(b,sizeof(b),"ROTATE YAW  %.1f deg left",std::max(0.0,kV3YawTriggerDeg-std::abs(fdyaw)));
    v3Text(panel,b,18,655,.62,rp_bad?red:yellow,2);
    v3Text(panel,"STOP IMMEDIATELY AT TARGET",18,690,.58,yellow,2);
  } else v3Text(panel,"HOLD STILL - DO NOT MOVE",18,655,.62,green,2);
  v3Text(panel,"ESC / Q = abort",18,730,.48,white,1);

  cv::imshow(kV3Window,canvas);
}

void saveV3Csv(const std::vector<V3State>& states,int64_t init_end,int64_t yaw_end) {
  std::ofstream f(kV3Csv,std::ios::trunc);
  f<<"phase,wall_ns,keyframe,timestamp_ns,px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,bax,bay,baz,bgx,bgy,bgz,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,fc_accum_yaw_deg,ax_flu,ay_flu,az_flu,gx_flu,gy_flu,gz_flu\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states){const char* ph=s.wall_ns<init_end?"INIT":(s.wall_ns<yaw_end?"YAW":"HOLD");
    f<<ph<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll<<','<<s.pitch<<','<<s.yaw<<','<<s.bax<<','<<s.bay<<','<<s.baz<<','<<s.bgx<<','<<s.bgy<<','<<s.bgz<<','<<s.fc_roll<<','<<s.fc_pitch<<','<<s.fc_yaw<<','<<s.fc_accum_yaw<<','<<s.ax<<','<<s.ay<<','<<s.az<<','<<s.gx<<','<<s.gy<<','<<s.gz<<'\n';}
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";

  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;V3Telemetry telemetry;std::shared_ptr<V3Pipeline>pipe;std::thread pipe_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<V3Pipeline>(vp,&telemetry);pipe->installBackendCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kV3AttitudeRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kV3Window,cv::WINDOW_NORMAL);cv::resizeWindow(kV3Window,1280,760);

    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rej=0,sel=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    const int64_t start=monotonicNs(),init_end=start+(int64_t)(kV3InitSec*1e9);int64_t yaw_end=0,hold_end=0,next_hud=start;V3Refs ref;double max_droll=0,max_dpitch=0,max_xy=0,max_vxy=0,target_fc_yaw=0,target_vio_yaw=0;
    std::cout<<"\nJT-ZERO YAW IMU-ONLY v3 GUI\n10 s INIT -> yaw 88 deg -> 10 s HOLD\n";

    while(true){
      const int64_t now=monotonicNs();
      if(!ref.valid&&now>=init_end){V3State v;bool have_v=pipe->latest(&v);std::lock_guard<std::mutex>l(telemetry.mutex);if(have_v&&telemetry.fc_valid){ref.valid=true;ref.fc_accum=telemetry.fc_accum_yaw;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.vio=v;std::cout<<"[YAW] reference captured\n";}}
      double cur_fc_dyaw=0,cur_dr=0,cur_dp=0;if(ref.valid){std::lock_guard<std::mutex>l(telemetry.mutex);cur_fc_dyaw=telemetry.fc_accum_yaw-ref.fc_accum;cur_dr=wrapDeg(telemetry.fc_roll-ref.fc_roll);cur_dp=wrapDeg(telemetry.fc_pitch-ref.fc_pitch);max_droll=std::max(max_droll,std::abs(cur_dr));max_dpitch=std::max(max_dpitch,std::abs(cur_dp));}
      if(ref.valid&&!yaw_end){if(std::abs(cur_fc_dyaw)>=kV3YawTriggerDeg){yaw_end=now;hold_end=now+(int64_t)(kV3HoldSec*1e9);target_fc_yaw=cur_fc_dyaw;V3State v;if(pipe->latest(&v))target_vio_yaw=wrapDeg(v.yaw-ref.vio.yaw);std::cout<<"[YAW] target reached at accumulated FC yaw="<<std::fixed<<std::setprecision(3)<<target_fc_yaw<<" deg -> HOLD\n";}if(now-init_end>(int64_t)(kV3YawTimeoutSec*1e9))throw std::runtime_error("Yaw target timeout");}
      if(yaw_end&&now>=hold_end)break;
      if(ref.valid){V3State v;if(pipe->latest(&v)){max_xy=std::max(max_xy,std::hypot(v.px-ref.vio.px,v.py-ref.vio.py));max_vxy=std::max(max_vxy,std::hypot(v.vx,v.vy));}}
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=rx;s.fc_ns=ts.tc1;s.rtt_ns=rx-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);const double y=a.yaw*180.0/kPi;std::lock_guard<std::mutex>l(telemetry.mutex);if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;{std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;telemetry.raw_valid=true;}if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rej;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){std::string phase=now<init_end?"INIT":(!yaw_end?"YAW":"HOLD");double left=now<init_end?(init_end-now)/1e9:(!yaw_end?std::max(0.0,kV3YawTimeoutSec-(now-init_end)/1e9):(hold_end-now)/1e9);renderV3Hud(last_gray,*pipe,telemetry,ref,phase,left,max_xy,max_vxy);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }

    if(!yaw_end)yaw_end=monotonicNs();std::this_thread::sleep_for(std::chrono::milliseconds(500));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipe->states();saveV3Csv(states,init_end,yaw_end);
    if(imu_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);

    double final_fc=0;{std::lock_guard<std::mutex>l(telemetry.mutex);if(ref.valid)final_fc=telemetry.fc_accum_yaw-ref.fc_accum;}V3State last{};bool have_last=pipe->latest(&last);double final_xy=0,dx=0,dy=0,dz=0,final_vio_yaw=0;if(ref.valid&&have_last){dx=last.px-ref.vio.px;dy=last.py-ref.vio.py;dz=last.pz-ref.vio.pz;final_xy=std::hypot(dx,dy);final_vio_yaw=wrapDeg(last.yaw-ref.vio.yaw);}std::cout<<"\n============================================================\nJT-ZERO YAW IMU-ONLY v3 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nrejected raw pairs: "<<rej<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nIMU skipped mapping: "<<imu_skip<<"\nATTITUDE received: "<<att_rx<<"\nTIMESYNC samples: "<<sync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm: "<<mapping.drift_ppm<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kV3Csv<<"\n"<<std::fixed<<std::setprecision(6)<<"TARGET FC accumulated dYaw: "<<target_fc_yaw<<" deg\nTARGET VIO dYaw: "<<target_vio_yaw<<" deg\nTARGET yaw scale: "<<(std::abs(target_fc_yaw)>1e-6?target_vio_yaw/target_fc_yaw:0)<<"\nFINAL FC accumulated dYaw: "<<final_fc<<" deg\nFINAL VIO dYaw: "<<final_vio_yaw<<" deg\nmax FC |dRoll|: "<<max_droll<<" deg\nmax FC |dPitch|: "<<max_dpitch<<" deg\nmax false XY: "<<max_xy*1000.0<<" mm\nfinal false XY: "<<final_xy*1000.0<<" mm\nmax Vxy: "<<max_vxy*1000.0<<" mm/s\nfinal dP: ["<<dx<<','<<dy<<','<<dz<<"] m\n";if(!states.empty()){const auto&a=states.front(),&b=states.back();std::cout<<"FIRST BA=["<<a.bax<<','<<a.bay<<','<<a.baz<<"] BG=["<<a.bgx<<','<<a.bgy<<','<<a.bgz<<"]\nLAST  BA=["<<b.bax<<','<<b.bay<<','<<b.baz<<"] BG=["<<b.bgx<<','<<b.bgy<<','<<b.bgz<<"]\n";}return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
