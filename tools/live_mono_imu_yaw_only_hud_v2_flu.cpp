// JT-ZERO live Mono+IMU yaw-only HUD v2 FLU diagnostic.
// Protocol: 10 s INIT -> rotate yaw to ~90 deg -> automatic 10 s HOLD.
// HIGHRES_IMU input is converted from ArduPilot FRD to Kimera FLU:
//   x'=x, y'=-y, z'=-z for both accelerometer and gyroscope.
#define main jtzero_hud_v2_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

namespace {

constexpr int kYawFcAttitudeRateHz = 50;
constexpr const char* kYawV2VioCsv = "/home/vio/jtzero_live_yaw_only_v2_flu_vio.csv";
constexpr const char* kYawV2FcCsv  = "/home/vio/jtzero_live_yaw_only_v2_flu_fc.csv";
constexpr const char* kYawV2Window = "JT-ZERO YAW-ONLY HUD v2 FLU";
constexpr double kInitSec = 10.0;
constexpr double kHoldSec = 10.0;
constexpr double kYawTargetDeg = 90.0;
constexpr double kYawTriggerDeg = 88.0;
constexpr double kYawMinValidDeg = 85.0;
constexpr double kYawMaxValidDeg = 95.0;
constexpr double kRpLimitDeg = 2.0;
constexpr double kMaxYawPhaseSec = 30.0;

struct FcSample {
  int64_t wall_ns = 0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
};

struct RefState {
  bool valid = false;
  FcSample fc0;
  VioState vio0;
  int64_t wall_ns = 0;
};

struct TrialStats {
  double max_abs_roll = 0.0;
  double max_abs_pitch = 0.0;
  double max_false_xy_m = 0.0;
  double max_vxy_mps = 0.0;
};

static double wrap180(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

static void drawSignedBar(cv::Mat& img, const std::string& label,
                          double value, double full, const std::string& unit,
                          int y, bool limit_bad = false) {
  const cv::Scalar white(240,240,240), gray(110,110,110);
  const cv::Scalar green(90,210,90), red(30,80,245);
  txt(img,label,{24,y},.65,white,2);
  char b[96]; std::snprintf(b,sizeof(b),"%+.1f %s",value,unit.c_str());
  txt(img,b,{275,y},.65,limit_bad?red:white,2);
  const int x0=24,w=420,h=22,yy=y+15;
  cv::rectangle(img,{x0,yy,w,h},gray,2);
  cv::line(img,{x0+w/2,yy},{x0+w/2,yy+h},{150,150,150},1);
  const double cl=std::max(-full,std::min(full,value));
  const int px=static_cast<int>((cl/full)*(w/2));
  const int xa=px>=0?x0+w/2:x0+w/2+px;
  cv::rectangle(img,{xa,yy,std::max(1,std::abs(px)),h},limit_bad?red:green,-1);
}

static void saveVioCsv(const std::vector<VioState>& states,
                       int64_t init_end, int64_t yaw_end) {
  std::ofstream f(kYawV2VioCsv,std::ios::trunc);
  f<<"phase,keyframe,timestamp_ns,callback_wall_ns,px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states){
    const char* ph=s.callback_wall_ns<init_end?"INIT":(s.callback_wall_ns<yaw_end?"YAW":"HOLD");
    f<<ph<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.callback_wall_ns<<','
     <<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','
     <<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
  }
}

static void saveFcCsv(const std::vector<FcSample>& samples,
                      int64_t init_end, int64_t yaw_end) {
  std::ofstream f(kYawV2FcCsv,std::ios::trunc);
  f<<"phase,wall_ns,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:samples){
    const char* ph=s.wall_ns<init_end?"INIT":(s.wall_ns<yaw_end?"YAW":"HOLD");
    f<<ph<<','<<s.wall_ns<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
  }
}

static void renderHud(const cv::Mat& gray,
                      const HudPipeline& pipeline,
                      const FcSample& fc, bool fc_valid,
                      const RefState& ref,
                      const TrialStats& stats,
                      const std::string& phase,
                      double seconds_left,
                      bool yaw_reached) {
  cv::Mat bgr;
  if(gray.empty()) bgr=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,bgr,cv::Size(800,600),0,0,cv::INTER_NEAREST);

  cv::Mat canvas(760,1280,CV_8UC3,cv::Scalar(15,15,15));
  bgr.copyTo(canvas(cv::Rect(0,80,800,600)));
  cv::line(canvas,{345,380},{455,380},{0,255,255},3);
  cv::line(canvas,{400,325},{400,435},{0,255,255},3);
  cv::circle(canvas,{400,380},14,{0,255,255},2);
  txt(canvas,"JT-ZERO YAW-ONLY VALIDATION v2 FLU",{24,48},1.0,{245,245,245},3);

  cv::Mat panel=canvas(cv::Rect(800,0,480,760));
  const cv::Scalar green(90,220,90), yellow(0,220,255), red(30,80,245), white(245,245,245);

  double fc_dyaw=0,fc_droll=0,fc_dpitch=0;
  if(ref.valid&&fc_valid){
    fc_dyaw=wrap180(fc.yaw_deg-ref.fc0.yaw_deg);
    fc_droll=wrap180(fc.roll_deg-ref.fc0.roll_deg);
    fc_dpitch=wrap180(fc.pitch_deg-ref.fc0.pitch_deg);
  }
  VioState v{}; const bool have_vio=pipeline.latest(&v);
  double vio_dyaw=0,dx=0,dy=0,dz=0,false_xy=0,vxy=0;
  if(ref.valid&&have_vio){
    vio_dyaw=wrap180(v.yaw_deg-ref.vio0.yaw_deg);
    dx=v.px-ref.vio0.px;dy=v.py-ref.vio0.py;dz=v.pz-ref.vio0.pz;
    false_xy=std::hypot(dx,dy);vxy=std::hypot(v.vx,v.vy);
  }

  const bool rp_bad=std::abs(fc_droll)>kRpLimitDeg||std::abs(fc_dpitch)>kRpLimitDeg;
  if(rp_bad&&phase=="YAW") cv::rectangle(panel,{0,0,480,760},red,10);

  cv::Scalar phase_color=green;
  std::string instruction="HOLD STILL";
  if(phase=="INIT") instruction="HOLD STILL - ZEROING";
  else if(phase=="YAW"){instruction=rp_bad?"LEVEL THE RIG":"ROTATE YAW ONLY";phase_color=rp_bad?red:yellow;}
  else if(phase=="HOLD") instruction="TARGET REACHED - HOLD STILL";
  else instruction="DONE";

  txt(panel,"PHASE: "+phase,{20,45},.90,phase_color,3);
  txt(panel,instruction,{20,86},.68,phase_color,2);
  char buf[160];std::snprintf(buf,sizeof(buf),"%.1f s",std::max(0.0,seconds_left));
  txt(panel,buf,{20,120},.70,white,2);

  drawSignedBar(panel,"FC YAW",fc_dyaw,100.0,"deg",160,false);
  drawSignedBar(panel,"VIO YAW",vio_dyaw,100.0,"deg",235,false);
  drawSignedBar(panel,"dROLL",fc_droll,4.0,"deg",310,std::abs(fc_droll)>kRpLimitDeg);
  drawSignedBar(panel,"dPITCH",fc_dpitch,4.0,"deg",385,std::abs(fc_dpitch)>kRpLimitDeg);

  std::snprintf(buf,sizeof(buf),"FALSE XY: %.1f mm",false_xy*1000.0);
  txt(panel,buf,{20,500},.80,false_xy<0.03?green:(false_xy<0.06?yellow:red),3);
  std::snprintf(buf,sizeof(buf),"MAX XY: %.1f mm",stats.max_false_xy_m*1000.0);
  txt(panel,buf,{20,535},.62,white,2);
  std::snprintf(buf,sizeof(buf),"Vxy: %.1f  MAX %.1f mm/s",vxy*1000.0,stats.max_vxy_mps*1000.0);
  txt(panel,buf,{20,568},.58,white,2);
  std::snprintf(buf,sizeof(buf),"dP=[%.1f %.1f %.1f] mm",dx*1000.0,dy*1000.0,dz*1000.0);
  txt(panel,buf,{20,600},.54,white,1);

  if(phase=="YAW"){
    if(!rp_bad){
      if(std::abs(fc_dyaw)<kYawTriggerDeg){
        std::snprintf(buf,sizeof(buf),"ROTATE MORE  %.1f deg left",kYawTargetDeg-std::abs(fc_dyaw));
        txt(panel,buf,{20,655},.69,yellow,2);
      } else txt(panel,"90 DEG REACHED",{20,655},.80,green,3);
    } else {
      txt(panel,"ROLL/PITCH > 2 DEG",{20,655},.75,red,3);
    }
  } else if(phase=="HOLD") {
    txt(panel,yaw_reached?"DO NOT MOVE":"HOLD",{20,655},.82,green,3);
  }
  std::snprintf(buf,sizeof(buf),"max dR %.1f  max dP %.1f deg",stats.max_abs_roll,stats.max_abs_pitch);
  txt(panel,buf,{20,704},.54,(stats.max_abs_roll>kRpLimitDeg||stats.max_abs_pitch>kRpLimitDeg)?red:white,1);

  cv::imshow(kYawV2Window,canvas);
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";

  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<HudPipeline>pipe;std::thread pipe_thread;

  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<HudPipeline>(vp);pipe->installBackendCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;

    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kYawFcAttitudeRateHz);att_req=true;

    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kYawV2Window,cv::WINDOW_NORMAL);cv::resizeWindow(kYawV2Window,1280,760);

    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,rej=0,sel=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;
    std::vector<FcSample>fc_samples;fc_samples.reserve(2500);FcSample fc;bool fc_valid=false;RefState ref;TrialStats stats;
    cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));

    const int64_t start=monotonicNs();const int64_t init_end=start+(int64_t)(kInitSec*1e9);
    int64_t yaw_start=init_end,yaw_end=0,hold_end=0;bool yaw_reached=false,ref_captured=false;int64_t next_hud=start;
    std::cout<<"\nYAW-ONLY HUD v2 FLU: 10s INIT -> rotate to 90deg -> auto 10s HOLD\n";

    while(true){
      const int64_t now=monotonicNs();
      if(now>=init_end&&!ref_captured&&fc_valid){VioState v;if(pipe->latest(&v)){ref.valid=true;ref.fc0=fc;ref.vio0=v;ref.wall_ns=now;ref_captured=true;std::cout<<"[YAW] reference captured\n";}}
      if(ref.valid&&!yaw_reached&&now>=init_end){
        const double dyaw=wrap180(fc.yaw_deg-ref.fc0.yaw_deg);
        const double dr=wrap180(fc.roll_deg-ref.fc0.roll_deg),dp=wrap180(fc.pitch_deg-ref.fc0.pitch_deg);
        stats.max_abs_roll=std::max(stats.max_abs_roll,std::abs(dr));stats.max_abs_pitch=std::max(stats.max_abs_pitch,std::abs(dp));
        if(std::abs(dyaw)>=kYawTriggerDeg){yaw_reached=true;yaw_end=now;hold_end=now+(int64_t)(kHoldSec*1e9);std::cout<<"[YAW] target reached -> HOLD\n";}
        if(now-yaw_start>(int64_t)(kMaxYawPhaseSec*1e9))throw std::runtime_error("Yaw target timeout");
      }
      if(yaw_reached&&now>=hold_end)break;

      std::string phase="INIT";double left=(init_end-now)/1e9;
      if(now>=init_end&&!yaw_reached){phase="YAW";left=std::max(0.0,kMaxYawPhaseSec-(now-yaw_start)/1e9);}
      else if(yaw_reached){phase="HOLD";left=(hold_end-now)/1e9;}

      if(ref.valid){VioState v;if(pipe->latest(&v)){const double dx=v.px-ref.vio0.px,dy=v.py-ref.vio0.py;stats.max_false_xy_m=std::max(stats.max_false_xy_m,std::hypot(dx,dy));stats.max_vxy_mps=std::max(stats.max_vxy_mps,std::hypot(v.vx,v.vy));}}

      if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=rx;s.fc_ns=ts.tc1;s.rtt_ns=rx-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}VIO::ImuAccGyr d;d<<h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);++att_rx;fc_valid=true;fc.wall_ns=rx;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;fc_samples.push_back(fc);}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;

      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rej;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}

      if(now>=next_hud){renderHud(last_gray,*pipe,fc,fc_valid,ref,stats,phase,left,yaw_reached);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }

    if(!yaw_end)yaw_end=monotonicNs();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();
    const auto states=pipe->states();saveVioCsv(states,init_end,yaw_end);saveFcCsv(fc_samples,init_end,yaw_end);

    if(imu_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);

    double fc_dyaw=0,fc_droll=0,fc_dpitch=0,vio_dyaw=0,final_xy=0,dx=0,dy=0,dz=0;
    if(ref.valid&&fc_valid){fc_dyaw=wrap180(fc.yaw_deg-ref.fc0.yaw_deg);fc_droll=wrap180(fc.roll_deg-ref.fc0.roll_deg);fc_dpitch=wrap180(fc.pitch_deg-ref.fc0.pitch_deg);}
    VioState vlast{};bool have_vlast=pipe->latest(&vlast);if(ref.valid&&have_vlast){vio_dyaw=wrap180(vlast.yaw_deg-ref.vio0.yaw_deg);dx=vlast.px-ref.vio0.px;dy=vlast.py-ref.vio0.py;dz=vlast.pz-ref.vio0.pz;final_xy=std::hypot(dx,dy);}
    const double yaw_abs=std::abs(fc_dyaw);const bool valid=!aborted&&ref.valid&&yaw_reached&&yaw_abs>=kYawMinValidDeg&&yaw_abs<=kYawMaxValidDeg&&stats.max_abs_roll<=kRpLimitDeg&&stats.max_abs_pitch<=kRpLimitDeg;
    const double yaw_scale=(std::abs(fc_dyaw)>1e-6)?vio_dyaw/fc_dyaw:0.0;

    std::cout<<"\n============================================================\nJT-ZERO YAW-ONLY HUD v2 FLU RESULT\n============================================================\n"
             <<"VALIDITY: "<<(valid?"VALID":"INVALID")<<"\n"
             <<"aborted: "<<(aborted?"yes":"no")<<"\n"
             <<"raw camera frames: "<<raw<<"\nrejected raw pairs: "<<rej<<"\nselected frames: "<<sel<<"\n"
             <<"IMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nIMU skipped mapping: "<<imu_skip<<"\nATTITUDE received: "<<att_rx<<"\n"
             <<"TIMESYNC samples: "<<sync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm: "<<mapping.drift_ppm<<"\nbackend states: "<<states.size()<<"\n"
             <<"VIO CSV: "<<kYawV2VioCsv<<"\nFC CSV: "<<kYawV2FcCsv<<"\n"
             <<std::fixed<<std::setprecision(6)
             <<"FC dYaw: "<<fc_dyaw<<" deg\nVIO dYaw: "<<vio_dyaw<<" deg\nyaw scale VIO/FC: "<<yaw_scale<<"\n"
             <<"max FC |dRoll|: "<<stats.max_abs_roll<<" deg\nmax FC |dPitch|: "<<stats.max_abs_pitch<<" deg\n"
             <<"final FC dRoll/dPitch: ["<<fc_droll<<','<<fc_dpitch<<"] deg\n"
             <<"max false XY: "<<stats.max_false_xy_m*1000.0<<" mm\nfinal false XY: "<<final_xy*1000.0<<" mm\nmax Vxy: "<<stats.max_vxy_mps*1000.0<<" mm/s\n"
             <<"final dP: ["<<dx<<','<<dy<<','<<dz<<"] m\n";
    return valid?0:3;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
