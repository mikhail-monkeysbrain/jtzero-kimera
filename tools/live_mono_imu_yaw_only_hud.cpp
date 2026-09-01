// JT-ZERO live Mono+IMU yaw-only HUD diagnostic.
// Protocol: 10 s STILL -> 15 s YAW (~90 deg in place) -> 10 s SETTLE.
#define main jtzero_fc_hud_v3_unused_main
#include "live_mono_imu_300mm_fc_hud_v3.cpp"
#undef main

namespace {

constexpr const char* kYawHudCsv = "/home/vio/jtzero_live_yaw_only_hud.csv";
constexpr const char* kYawHudWindow = "JT-ZERO YAW-ONLY HUD";
constexpr double kYawInitSec = 10.0;
constexpr double kYawMoveSec = 15.0;
constexpr double kYawSettleSec = 10.0;
constexpr double kYawTargetDeg = 90.0;
constexpr double kYawToleranceDeg = 5.0;
constexpr double kRollPitchLimitDeg = 2.0;

struct YawHudRef {
  bool fc_ref_valid = false;
  double fc_roll0 = 0.0;
  double fc_pitch0 = 0.0;
  double fc_yaw0 = 0.0;
  bool vio_ref_valid = false;
  VioState vio0;
};

static double yawWrap(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

static const char* yawPhase(int64_t now, int64_t t1, int64_t t2, int64_t t3) {
  if (now < t1) return "INIT";
  if (now < t2) return "YAW";
  if (now < t3) return "SETTLE";
  return "DONE";
}

static void drawBar(cv::Mat& img, const std::string& label, double value, double full,
                    const std::string& unit, int y) {
  txt(img, label, {28,y}, .70, {230,230,230}, 2);
  char b[96];
  std::snprintf(b,sizeof(b),"%+.1f %s",value,unit.c_str());
  txt(img,b,{260,y},.70,{245,245,245},2);
  const int x0=28,w=520,h=26, yy=y+18;
  cv::rectangle(img,{x0,yy,w,h},{110,110,110},2);
  cv::line(img,{x0+w/2,yy},{x0+w/2,yy+h},{150,150,150},1);
  const double cl=std::max(-full,std::min(full,value));
  const int px=static_cast<int>((cl/full)*(w/2));
  const int xa=px>=0?x0+w/2:x0+w/2+px;
  cv::rectangle(img,{xa,yy,std::max(1,std::abs(px)),h},{90,210,90},-1);
}

static void renderYawHud(const cv::Mat& gray,
                         const HudPipeline& pipeline,
                         const FcAttitude& fc,
                         YawHudRef* ref,
                         const std::string& ph,
                         double seconds_left) {
  cv::Mat bgr;
  if (gray.empty()) bgr=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));
  else cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,bgr,cv::Size(800,600),0,0,cv::INTER_NEAREST);

  cv::Mat canvas(760,1280,CV_8UC3,cv::Scalar(15,15,15));
  bgr.copyTo(canvas(cv::Rect(0,80,800,600)));
  cv::line(canvas,{400-55,380},{400+55,380},{0,255,255},3);
  cv::line(canvas,{400,380-55},{400,380+55},{0,255,255},3);
  cv::circle(canvas,{400,380},14,{0,255,255},2);

  txt(canvas,"JT-ZERO YAW-ONLY TEST",{24,48},1.05,{245,245,245},3);

  cv::Mat panel=canvas(cv::Rect(800,0,480,760));
  const cv::Scalar ok(90,220,90), warn(0,220,255), bad(30,80,245), white(245,245,245);

  std::string instruction;
  cv::Scalar phaseColor=ok;
  if(ph=="INIT") instruction="НЕ ДВИГАТЬ";
  else if(ph=="YAW") { instruction="ПОВОРАЧИВАТЬ ПО YAW"; phaseColor=warn; }
  else if(ph=="SETTLE") instruction="СТОП. НЕ ДВИГАТЬ";
  else instruction="ГОТОВО";

  txt(panel,"PHASE: "+ph,{20,48},.92,phaseColor,3);
  txt(panel,instruction,{20,92},.78,phaseColor,3);
  char buf[160];
  std::snprintf(buf,sizeof(buf),"%.1f s",std::max(0.0,seconds_left));
  txt(panel,buf,{20,128},.76,white,2);

  if(fc.valid && !ref->fc_ref_valid){
    ref->fc_ref_valid=true;
    ref->fc_roll0=fc.roll_deg;
    ref->fc_pitch0=fc.pitch_deg;
    ref->fc_yaw0=fc.yaw_deg;
  }

  VioState s; const bool have_vio=pipeline.latest(&s);
  if(have_vio && !ref->vio_ref_valid){ref->vio_ref_valid=true;ref->vio0=s;}

  double fcdy=0,fcdr=0,fcdp=0;
  if(fc.valid&&ref->fc_ref_valid){
    fcdy=yawWrap(fc.yaw_deg-ref->fc_yaw0);
    fcdr=yawWrap(fc.roll_deg-ref->fc_roll0);
    fcdp=yawWrap(fc.pitch_deg-ref->fc_pitch0);
  }
  double viody=0,dx=0,dy=0,dz=0,h=0,vxy=0;
  if(have_vio&&ref->vio_ref_valid){
    viody=yawWrap(s.yaw_deg-ref->vio0.yaw_deg);
    dx=s.px-ref->vio0.px;dy=s.py-ref->vio0.py;dz=s.pz-ref->vio0.pz;
    h=std::hypot(dx,dy);vxy=std::hypot(s.vx,s.vy);
  }

  drawBar(panel,"FC YAW",fcdy,kYawTargetDeg,"deg",175);
  drawBar(panel,"VIO YAW",viody,kYawTargetDeg,"deg",255);
  drawBar(panel,"ROLL",fcdr,kRollPitchLimitDeg,"deg",335);
  drawBar(panel,"PITCH",fcdp,kRollPitchLimitDeg,"deg",415);

  std::snprintf(buf,sizeof(buf),"FALSE XY: %.1f mm",h*1000.0);
  txt(panel,buf,{20,535},.82,h<0.03?ok:(h<0.06?warn:bad),3);
  std::snprintf(buf,sizeof(buf),"Vxy: %.1f mm/s",vxy*1000.0);
  txt(panel,buf,{20,572},.68,white,2);
  std::snprintf(buf,sizeof(buf),"dP=[%.1f %.1f %.1f] mm",dx*1000.0,dy*1000.0,dz*1000.0);
  txt(panel,buf,{20,606},.58,white,2);

  if(ph=="YAW"){
    const double rem=kYawTargetDeg-std::abs(fcdy);
    if(std::abs(fcdy)<kYawTargetDeg-kYawToleranceDeg)
      txt(panel,"КРУТИ ЕЩЁ",{20,660},.86,warn,3);
    else if(std::abs(fcdy)<=kYawTargetDeg+kYawToleranceDeg)
      txt(panel,"90° ДОСТИГНУТО",{20,660},.86,ok,3);
    else
      txt(panel,"ПЕРЕКРУТ",{20,660},.86,bad,3);
    std::snprintf(buf,sizeof(buf),"до цели: %.1f deg",rem);
    txt(panel,buf,{20,700},.62,white,2);
  }

  cv::imshow(kYawHudWindow,canvas);
}

static void writeYawHudCsv(const std::vector<VioState>& states,
                           int64_t t0,int64_t t1,int64_t t2,int64_t t3) {
  std::ofstream f(kYawHudCsv,std::ios::trunc);
  f<<"phase,keyframe,timestamp_ns,px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states){
    const char*ph=s.callback_wall_ns<t1?"INIT":(s.callback_wall_ns<t2?"YAW":"SETTLE");
    f<<ph<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','
     <<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
  }
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMono";

  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t target_system=0,target_component=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;

  try{
    VIO::VioParams vio_params(params);if(vio_params.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipeline=std::make_shared<HudPipeline>(vio_params);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t mav_status{};mavlink_message_t mav_msg{};
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!target_system){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t bytes[2048];ssize_t n=read(serial_fd,bytes,sizeof(bytes));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&mav_msg,&mav_status)&&mav_msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){target_system=mav_msg.sysid;target_component=mav_msg.compid;break;}}
    if(!target_system)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;

    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);
    cv::setNumThreads(1);cv::namedWindow(kYawHudWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kYawHudWindow,1280,760);

    std::vector<TimeSyncSample>timesync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_selected=0;bool have_prev=false;VIO::FrameId frame_id=0;
    const int64_t t0=monotonicNs(),t1=t0+(int64_t)(kYawInitSec*1e9),t2=t1+(int64_t)(kYawMoveSec*1e9),t3=t2+(int64_t)(kYawSettleSec*1e9);int64_t next_hud=t0;FcAttitude fc;YawHudRef ref;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));

    std::cout<<"\nYAW-ONLY HUD: 10s STILL -> 15s rotate ~90 deg -> 10s STILL\n";
    while(monotonicNs()<t3){
      const int64_t now=monotonicNs();const std::string ph=yawPhase(now,t1,t2,t3);const int64_t pend=now<t1?t1:(now<t2?t2:t3);const double left=(pend-now)/1e9;
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,target_system,target_component);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t bytes[8192];for(;;){ssize_t n=read(serial_fd,bytes,sizeof(bytes));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&mav_msg,&mav_status))continue;const int64_t recv=monotonicNs();if(mav_msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&mav_msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;timesync.push_back(s);pending=0;mapping=estimateClockMapping(timesync);}}else if(mav_msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t imu{};mavlink_msg_highres_imu_decode(&mav_msg,&imu);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}VIO::ImuAccGyr data;data<<imu.xacc,imu.yacc,imu.zacc,imu.xgyro,imu.ygyro,imu.zgyro;pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)imu.time_usec*1000LL),data));++imu_fed;}else if(mav_msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mav_msg,&a);++att_rx;fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;const bool due=last_selected==0||corrected-last_selected>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),buffers[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){++decoded;last_gray=gray;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(frame_id++,corrected,vio_params.camera_params_.at(0),gray.clone()));last_selected=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderYawHud(last_gray,*pipeline,fc,&ref,ph,left);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();writeYawHudCsv(states,t0,t1,t2,t3);
    if(imu_req){requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);

    std::cout<<"\n============================================================\nJT-ZERO YAW-ONLY HUD RESULT\n============================================================\n"
             <<"aborted: "<<(aborted?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nrejected raw pairs: "<<rejected<<"\nselected frames: "<<selected<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nIMU skipped mapping: "<<imu_skip<<"\nATTITUDE received: "<<att_rx<<"\nTIMESYNC samples: "<<timesync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm: "<<mapping.drift_ppm<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kYawHudCsv<<"\n";
    if(states.size()>=2){const auto&a=states.front();const auto&b=states.back();double dx=b.px-a.px,dy=b.py-a.py,dz=b.pz-a.pz;std::cout<<std::fixed<<std::setprecision(6)<<"VIO dYaw: "<<yawWrap(b.yaw_deg-a.yaw_deg)<<" deg\nfinal false XY: "<<std::hypot(dx,dy)*1000.0<<" mm\nfinal dP: ["<<dx<<','<<dy<<','<<dz<<"] m\n";}
    return aborted?2:0;
  }catch(const std::exception&e){if(pipeline){pipeline->shutdown();}if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();if(streaming&&camera_fd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd>=0)close(camera_fd);if(serial_fd>=0)close(serial_fd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
