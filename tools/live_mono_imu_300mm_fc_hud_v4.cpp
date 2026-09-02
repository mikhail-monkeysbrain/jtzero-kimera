// JT-ZERO 300 mm production-candidate test v4.
// Standalone v4 runtime built directly on verified v2 helpers.
// IMU path: ArduPilot HIGHRES_IMU FRD -> FLU -> validated ZXY -> gravity feedback -> Kimera.

#define main jtzero_hud_v2_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

namespace {

constexpr double kExpectedDistanceM = 0.300;
constexpr double kDirectionLearnM = 0.030;
constexpr double kFcRollLimitDeg = 1.5;
constexpr double kFcPitchLimitDeg = 1.5;
constexpr double kFcYawLimitDeg = 2.0;
constexpr int kFcAttitudeRateHz = 50;
constexpr double kGyroCx = 0.014570;
constexpr double kGyroCy = 0.082383;
constexpr double kGravityKp = 1.0;
constexpr double kGravityG = 9.81;
constexpr double kGravityAccTol = 0.35;
constexpr const char* kCsvPath = "/home/vio/jtzero_live_300mm_fc_hud_v4.csv";
constexpr const char* kWindowName = "JT-ZERO 300 mm FC HUD v4 - corrected IMU";

struct FcAttitude {
  bool valid=false;
  int64_t wall_ns=0;
  double roll_deg=0,pitch_deg=0,yaw_deg=0;
};

struct FcReference {
  bool valid=false;
  double roll_deg=0,pitch_deg=0,yaw_deg=0;
  double max_abs_roll_deg=0,max_abs_pitch_deg=0,max_abs_yaw_deg=0;
};

struct GravityStabilizer {
  bool initialized=false;
  uint64_t last_us=0;
  Eigen::Vector3d gravity_body=Eigen::Vector3d(0,0,1);

  Eigen::Vector3d correct(uint64_t us,const Eigen::Vector3d& accel_flu,const Eigen::Vector3d& gyro_zxy){
    double dt=0.0;
    if(last_us&&us>last_us)dt=double(us-last_us)*1e-6;
    last_us=us;
    if(dt<=0.0||dt>0.03)return gyro_zxy;
    const double an=accel_flu.norm();
    const bool gravity_valid=an>1e-6&&std::abs(an-kGravityG)<kGravityAccTol;
    if(!initialized){
      if(gravity_valid){gravity_body=accel_flu.normalized();initialized=true;}
      return gyro_zxy;
    }
    Eigen::Vector3d w=gyro_zxy;
    if(gravity_valid){
      const Eigen::Vector3d measured=accel_flu.normalized();
      const Eigen::Vector3d error=gravity_body.cross(measured);
      w-=kGravityKp*error;
    }
    const Eigen::Vector3d theta=-w*dt;
    const double a=theta.norm();
    if(a>1e-12){gravity_body=Eigen::AngleAxisd(a,theta/a)*gravity_body;gravity_body.normalize();}
    return w;
  }
};

void writeCsv(const std::vector<VioState>& states,int64_t start,int64_t move,int64_t end,int64_t stop){
  std::ofstream f(kCsvPath,std::ios::trunc);if(!f)return;
  f<<"phase,keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states)f<<phaseName(s.timestamp_ns,start,move,end,stop)<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
}

void printMeasurement(const MeanState&a,const MeanState&b){
  if(!a.valid||!b.valid){std::cout<<"MEASUREMENT RESULT: FAIL (insufficient START/END states)\n";return;}
  const double dx=b.px-a.px,dy=b.py-a.py,dz=b.pz-a.pz;
  const double horizontal=std::sqrt(dx*dx+dy*dy);
  const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
  const double error=distance-kExpectedDistanceM;
  const double end_v=std::sqrt(b.vx*b.vx+b.vy*b.vy+b.vz*b.vz);
  std::cout<<std::fixed<<std::setprecision(6)
           <<"START avg states:       "<<a.count<<"\n"
           <<"END avg states:         "<<b.count<<"\n"
           <<"measured dP:            ["<<dx<<','<<dy<<','<<dz<<"] m\n"
           <<"horizontal distance:    "<<horizontal*1000.0<<" mm\n"
           <<"3D measured distance:   "<<distance*1000.0<<" mm\n"
           <<"expected distance:      300.000000 mm\n"
           <<"absolute error:         "<<error*1000.0<<" mm\n"
           <<"relative error:         "<<error/kExpectedDistanceM*100.0<<" %\n"
           <<"scale measured/true:    "<<distance/kExpectedDistanceM<<"\n"
           <<"VIO dRoll:              "<<wrapDeg(b.roll_deg-a.roll_deg)<<" deg\n"
           <<"VIO dPitch:             "<<wrapDeg(b.pitch_deg-a.pitch_deg)<<" deg\n"
           <<"VIO dYaw:               "<<wrapDeg(b.yaw_deg-a.yaw_deg)<<" deg\n"
           <<"END mean |V|:           "<<end_v*1000.0<<" mm/s\n";
}

void drawFcGauge(cv::Mat&panel,const std::string&name,double value,double limit,int y){gauge(panel,name,value,limit,"deg",y);}

void renderHud(const cv::Mat&gray,const HudPipeline&pipeline,HudReference*vio_ref,const FcAttitude&fc,FcReference*fc_ref,const std::string&ph,int64_t now,double left){
  cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,cv::Size(900,675),0,0,cv::INTER_NEAREST);
  const int cx=video.cols/2,cy=video.rows/2;cv::line(video,{cx-55,cy},{cx+55,cy},{0,255,255},3);cv::line(video,{cx,cy-55},{cx,cy+55},{0,255,255},3);cv::circle(video,{cx,cy},13,{0,255,255},2);
  cv::Mat panel(900,380,CV_8UC3,cv::Scalar(24,24,24)),canvas(900,1280,CV_8UC3,cv::Scalar(8,8,8));video.copyTo(canvas(cv::Rect(0,105,900,675)));
  txt(canvas,"JT-ZERO LIVE 300 mm v4",{28,52},1.12,{245,245,245},3);txt(canvas,"Исправленный IMU: FLU + ZXY + gravity feedback",{28,84},.55,{190,190,190},1);
  txt(panel,"PHASE "+ph,{18,42},.88,ph=="MOVE"?cv::Scalar(0,230,255):cv::Scalar(90,220,90),3);char buf[192];std::snprintf(buf,sizeof(buf),"%.1f s left",std::max(0.0,left));txt(panel,buf,{18,74},.62,{190,190,190},1);
  if(fc.valid&&!fc_ref->valid){fc_ref->valid=true;fc_ref->roll_deg=fc.roll_deg;fc_ref->pitch_deg=fc.pitch_deg;fc_ref->yaw_deg=fc.yaw_deg;}
  double dr=0,dp=0,dy=0;if(fc.valid&&fc_ref->valid){if(ph=="START"){fc_ref->roll_deg=fc.roll_deg;fc_ref->pitch_deg=fc.pitch_deg;fc_ref->yaw_deg=fc.yaw_deg;}dr=wrapDeg(fc.roll_deg-fc_ref->roll_deg);dp=wrapDeg(fc.pitch_deg-fc_ref->pitch_deg);dy=wrapDeg(fc.yaw_deg-fc_ref->yaw_deg);if(ph=="MOVE"){fc_ref->max_abs_roll_deg=std::max(fc_ref->max_abs_roll_deg,std::abs(dr));fc_ref->max_abs_pitch_deg=std::max(fc_ref->max_abs_pitch_deg,std::abs(dp));fc_ref->max_abs_yaw_deg=std::max(fc_ref->max_abs_yaw_deg,std::abs(dy));}txt(panel,"FC ATTITUDE",{18,112},.72,{220,220,220},2);drawFcGauge(panel,"ROLL ",dr,kFcRollLimitDeg,155);drawFcGauge(panel,"PITCH",dp,kFcPitchLimitDeg,235);drawFcGauge(panel,"YAW  ",dy,kFcYawLimitDeg,315);}else txt(panel,"Waiting FC ATTITUDE...",{18,160},.70,{245,245,245},2);
  VioState s;if(pipeline.latest(&s)){if(!vio_ref->have_baseline){vio_ref->baseline=s;vio_ref->have_baseline=true;}const double dx=s.px-vio_ref->baseline.px,dy2=s.py-vio_ref->baseline.py,dz=s.pz-vio_ref->baseline.pz,h=std::sqrt(dx*dx+dy2*dy2);if(!vio_ref->have_direction&&ph=="MOVE"&&h>=kDirectionLearnM){vio_ref->dir_x=dx/h;vio_ref->dir_y=dy2/h;vio_ref->have_direction=true;std::cout<<"[HUD] 300 mm motion direction learned\n";}const double travel=vio_ref->have_direction?dx*vio_ref->dir_x+dy2*vio_ref->dir_y:h;const double cross=vio_ref->have_direction?vio_ref->dir_x*dy2-vio_ref->dir_y*dx:0.0;const double speed=std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);gauge(panel,"Z    ",dz*1000.0,30.0,"mm",410);gauge(panel,"CROSS",cross*1000.0,30.0,"mm",490);std::snprintf(buf,sizeof(buf),"TRAVEL %.0f / 300 mm",travel*1000.0);txt(panel,buf,{18,580},.76,{245,245,245},2);const double prog=std::max(0.0,std::min(1.0,travel/kExpectedDistanceM));cv::rectangle(panel,{22,602,334,28},{100,100,100},2);cv::rectangle(panel,{25,605,static_cast<int>(328*prog),22},{90,210,90},-1);std::snprintf(buf,sizeof(buf),"|V| %.1f mm/s    KF %lld",speed*1000.0,(long long)s.keyframe);txt(panel,buf,{18,660},.68,{225,225,225},2);const int64_t age=now-s.callback_wall_ns;if(age>kBackendStaleNs){cv::rectangle(panel,{8,708,364,104},{20,20,220},-1);txt(panel,"BACKEND STALE",{20,750},.82,{255,255,255},3);std::snprintf(buf,sizeof(buf),"no state %.1f s",age/1e9);txt(panel,buf,{20,787},.64,{255,255,255},2);}else{const bool bad=std::abs(dr)>kFcRollLimitDeg||std::abs(dp)>kFcPitchLimitDeg||std::abs(dy)>kFcYawLimitDeg;txt(panel,bad?"ATTITUDE LIMIT - correct rig":"ATTITUDE OK",{18,738},.67,bad?cv::Scalar(30,80,245):cv::Scalar(80,220,80),2);txt(panel,"Остановись на механической отметке 300 мм",{18,780},.46,{185,185,185},1);txt(panel,"ESC / Q = abort",{18,820},.58,{215,215,215},1);}}
  panel.copyTo(canvas(cv::Rect(900,0,380,900)));cv::imshow(kWindowName,canvas);
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";const double total_sec=kStartPhaseSec+kMovePhaseSec+kEndPhaseSec;
  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");pipeline=std::make_shared<HudPipeline>(vp);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;
    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);cv::setNumThreads(1);cv::namedWindow(kWindowName,cv::WINDOW_NORMAL);cv::resizeWindow(kWindowName,1280,900);
    std::cout<<"\nJT-ZERO 300 MM v4 - CORRECTED IMU\nSTART 10 s: не двигать. MOVE 15 s: переместить ровно на 300 мм по механической отметке. END 10 s: не двигать.\n";
    std::vector<TimeSyncSample>sync;sync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;const int64_t start=monotonicNs(),move=start+(int64_t)(kStartPhaseSec*1e9),end=move+(int64_t)(kMovePhaseSec*1e9),stop=end+(int64_t)(kEndPhaseSec*1e9);int64_t next_hud=start;bool move_msg=false,end_msg=false;HudReference vr;FcAttitude fc;FcReference fr;cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));GravityStabilizer gravity;
    while(monotonicNs()<stop){const int64_t now=monotonicNs();const std::string ph=phase(now,move,end,stop);const int64_t pe=now<move?move:(now<end?end:stop);const double left=(pe-now)/1e9;if(!move_msg&&now>=move){move_msg=true;std::cout<<"\n>>> MOVE: ровно 300 мм по механической отметке <<<\n";}if(!end_msg&&now>=end){end_msg=true;std::cout<<"\n>>> STOP: не двигать <<<\n";}if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(serial_fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t recv=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}const Eigen::Vector3d acc(h.xacc,-h.yacc,-h.zacc);const double gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;const Eigen::Vector3d zxy(gx+kGyroCx*gz,gy+kGyroCy*gz,gz);const Eigen::Vector3d w=gravity.correct(h.time_usec,acc,zxy);VIO::ImuAccGyr data;data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),data));++imu_fed;}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);++att_rx;fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;const bool due=last_sel==0||corrected-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){++decoded;gray=g;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));last_sel=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderHud(gray,*pipeline,&vr,fc,&fr,ph,now,left);next_hud=now+kHudPeriodNs;}const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();writeCsv(states,start,move,end,stop);const MeanState a=meanWindow(states,move-(int64_t)(kAverageWindowSec*1e9),move),z=meanWindow(states,stop-(int64_t)(kAverageWindowSec*1e9),stop);
    if(imu_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    const bool alignment_pass=fr.valid&&fr.max_abs_roll_deg<=kFcRollLimitDeg&&fr.max_abs_pitch_deg<=kFcPitchLimitDeg&&fr.max_abs_yaw_deg<=kFcYawLimitDeg;
    std::cout<<"\n============================================================\nJT-ZERO LIVE MONO+IMU 300 MM FC HUD V4 RESULT\n============================================================\n"<<"aborted:               "<<(aborted?"yes":"no")<<"\n"<<"raw camera frames:     "<<raw<<"\n"<<"rejected raw pairs:    "<<rejected<<"\n"<<"selected frames:       "<<selected<<"\n"<<"decoded frames:        "<<decoded<<"\n"<<"IMU received:          "<<imu_rx<<"\n"<<"IMU fed to Kimera:     "<<imu_fed<<"\n"<<"ATTITUDE received:     "<<att_rx<<"\n"<<"TIMESYNC samples:      "<<sync.size()<<"\n"<<"mapping valid:         "<<(mapping.valid?"yes":"no")<<"\n"<<"mapping drift ppm:     "<<mapping.drift_ppm<<"\n"<<"backend states:        "<<states.size()<<"\n";printMeasurement(a,z);std::cout<<std::fixed<<std::setprecision(3)<<"FC max |dRoll| MOVE:   "<<fr.max_abs_roll_deg<<" deg\n"<<"FC max |dPitch| MOVE:  "<<fr.max_abs_pitch_deg<<" deg\n"<<"FC max |dYaw| MOVE:    "<<fr.max_abs_yaw_deg<<" deg\n"<<"ALIGNMENT RESULT:      "<<(alignment_pass?"PASS":"FAIL")<<"\n"<<"CSV:                   "<<kCsvPath<<"\n";const bool pass=!aborted&&mapping.valid&&att_rx>100&&selected>=total_sec*20&&imu_fed>=total_sec*150&&states.size()>=80&&a.valid&&z.valid;std::cout<<"PIPELINE RESULT:       "<<(pass?"PASS":"FAIL")<<"\n";return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
