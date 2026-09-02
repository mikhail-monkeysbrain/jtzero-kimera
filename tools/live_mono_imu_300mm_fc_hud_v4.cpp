// JT-ZERO 300 mm production-candidate test v4.
// Fixes the IMU path used by the 300 mm test:
//   ArduPilot HIGHRES_IMU FRD -> FLU
//   validated Z->XY gyro correction (v41/v42)
//   accelerometer gravity-reference feedback for roll/pitch drift
// The existing v3 HUD/measurement helpers are reused unchanged.

#define main jtzero_300mm_v3_unused_main
#include "live_mono_imu_300mm_fc_hud_v3.cpp"
#undef main

namespace {
constexpr double kV4GyroCx = 0.014570;
constexpr double kV4GyroCy = 0.082383;
constexpr double kV4GravityKp = 1.0;
constexpr double kV4GravityG = 9.81;
constexpr double kV4GravityAccTol = 0.35;
constexpr const char* kV4CsvPath = "/home/vio/jtzero_live_300mm_fc_hud_v4.csv";
constexpr const char* kV4WindowName = "JT-ZERO 300 mm FC HUD v4 - corrected IMU";

struct GravityStabilizerV4 {
  bool initialized = false;
  uint64_t last_us = 0;
  Eigen::Vector3d gravity_body = Eigen::Vector3d(0,0,1);

  Eigen::Vector3d correct(uint64_t us,
                          const Eigen::Vector3d& accel_flu,
                          const Eigen::Vector3d& gyro_zxy) {
    double dt = 0.0;
    if (last_us && us > last_us) dt = double(us-last_us)*1e-6;
    last_us = us;
    if (dt <= 0.0 || dt > 0.03) return gyro_zxy;

    const double an = accel_flu.norm();
    const bool gravity_valid = an > 1e-6 && std::abs(an-kV4GravityG) < kV4GravityAccTol;
    if (!initialized) {
      if (gravity_valid) {
        gravity_body = accel_flu.normalized();
        initialized = true;
      }
      return gyro_zxy;
    }

    Eigen::Vector3d w = gyro_zxy;
    if (gravity_valid) {
      const Eigen::Vector3d measured = accel_flu.normalized();
      const Eigen::Vector3d error = gravity_body.cross(measured);
      w -= kV4GravityKp * error;
    }

    const Eigen::Vector3d theta = -w*dt;
    const double a = theta.norm();
    if (a > 1e-12) {
      gravity_body = Eigen::AngleAxisd(a,theta/a)*gravity_body;
      gravity_body.normalize();
    }
    return w;
  }
};

void writeV4Csv(const std::vector<VioState>& states,
                int64_t start,int64_t move,int64_t end,int64_t stop) {
  std::ofstream f(kV4CsvPath,std::ios::trunc);
  if(!f)return;
  f<<"phase,keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states) f<<phaseName(s.timestamp_ns,start,move,end,stop)<<','<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
}
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";
  const double total_sec=kStartPhaseSec+kMovePhaseSec+kEndPhaseSec;
  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;uint8_t sys=0,comp=0;
  std::vector<CameraBuffer> buffers;std::shared_ptr<HudPipeline> pipeline;std::thread pipeline_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipeline=std::make_shared<HudPipeline>(vp);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;
    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);
    cv::setNumThreads(1);cv::namedWindow(kV4WindowName,cv::WINDOW_NORMAL);cv::resizeWindow(kV4WindowName,1280,900);
    std::cout<<"\nJT-ZERO 300 MM v4 - CORRECTED IMU\nSTART: do not move. MOVE: exactly 300 mm by mechanical mark. END: hold still.\nIMU: FRD->FLU + ZXY + gravity feedback.\n";
    std::vector<TimeSyncSample> sync;sync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;
    const int64_t start=monotonicNs(),move=start+(int64_t)(kStartPhaseSec*1e9),end=move+(int64_t)(kMovePhaseSec*1e9),stop=end+(int64_t)(kEndPhaseSec*1e9);int64_t next_hud=start;bool move_msg=false,end_msg=false;HudReference vr;FcAttitude fc;FcReference fr;cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));GravityStabilizerV4 gravity;
    while(monotonicNs()<stop){const int64_t now=monotonicNs();const std::string ph=phase(now,move,end,stop);const int64_t pe=now<move?move:(now<end?end:stop);const double left=(pe-now)/1e9;if(!move_msg&&now>=move){move_msg=true;std::cout<<"\n>>> MOVE: exactly 300 mm by mechanical mark <<<\n";}if(!end_msg&&now>=end){end_msg=true;std::cout<<"\n>>> STOP: hold still <<<\n";}if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(serial_fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t recv=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}const Eigen::Vector3d acc(h.xacc,-h.yacc,-h.zacc);const double gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;const Eigen::Vector3d zxy(gx+kV4GyroCx*gz,gy+kV4GyroCy*gz,gz);const Eigen::Vector3d w=gravity.correct(h.time_usec,acc,zxy);VIO::ImuAccGyr data;data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),data));++imu_fed;}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);++att_rx;fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;}
      }}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;const bool due=last_sel==0||corrected-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){++decoded;gray=g;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));last_sel=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderV3Hud(gray,*pipeline,&vr,fc,&fr,ph,now,left);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();writeV4Csv(states,start,move,end,stop);const MeanState a=meanWindow(states,move-(int64_t)(kAverageWindowSec*1e9),move),z=meanWindow(states,stop-(int64_t)(kAverageWindowSec*1e9),stop);
    if(imu_req)requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    std::cout<<"\n============================================================\nJT-ZERO 300 MM v4 RESULT - CORRECTED IMU\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<selected<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nTIMESYNC samples: "<<sync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nbackend states: "<<states.size()<<"\n";printV3Measurement(a,z);std::cout<<"CSV: "<<kV4CsvPath<<"\n";const bool pass=!aborted&&mapping.valid&&att_rx>100&&selected>=total_sec*20&&imu_fed>=total_sec*150&&states.size()>=80&&a.valid&&z.valid;std::cout<<"PIPELINE RESULT: "<<(pass?"PASS":"FAIL")<<"\n";return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
