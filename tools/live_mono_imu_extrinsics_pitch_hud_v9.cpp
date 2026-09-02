// JT-ZERO Stage 11.5: live pitch lever-arm validation.
// Reuses the validated v7 camera/timing/IMU correction path.
// Sequence: ZERO -> +45 deg -> ZERO -> -45 deg -> ZERO.

#define JTZERO_V7_EMBEDDED
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef JTZERO_V7_EMBEDDED
#undef main

namespace {
constexpr const char* kV9Window = "JT-ZERO 11.5 — проверка lever arm";
constexpr const char* kV9Csv = "/home/vio/jtzero_extrinsics_pitch_v9.csv";
constexpr int kStableStates = 4;
constexpr double kStableSpeed = 0.020;
constexpr double kStableSpan = 0.008;
constexpr double kTargetTolDeg = 2.0;
constexpr double kRollTolDeg = 5.0;
constexpr double kYawTolDeg = 8.0;
constexpr double kExpectedPitchDeg[] = {0.0, 45.0, 0.0, -45.0, 0.0};
constexpr int kPointCount = 5;

struct Stable9 {
  std::vector<VioState> q; int64_t last_kf=-1;
  void reset(){q.clear();last_kf=-1;}
  static double speed(const VioState&s){return std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);}
  bool update(const VioState&s,VioState*out){
    if(s.keyframe==last_kf)return false; last_kf=s.keyframe;
    if(speed(s)>kStableSpeed){q.clear();return false;}
    if(!q.empty()){const auto&a=q.front();const double dx=s.px-a.px,dy=s.py-a.py,dz=s.pz-a.pz;if(std::sqrt(dx*dx+dy*dy+dz*dz)>kStableSpan)q.clear();}
    q.push_back(s);if((int)q.size()<kStableStates)return false;
    *out=q.back();out->px=out->py=out->pz=out->vx=out->vy=out->vz=0;double sr=0,sp=0,sy=0;const auto&r=q.front();
    for(const auto&x:q){out->px+=x.px;out->py+=x.py;out->pz+=x.pz;out->vx+=x.vx;out->vy+=x.vy;out->vz+=x.vz;sr+=r.roll_deg+wrapDeg(x.roll_deg-r.roll_deg);sp+=r.pitch_deg+wrapDeg(x.pitch_deg-r.pitch_deg);sy+=r.yaw_deg+wrapDeg(x.yaw_deg-r.yaw_deg);}
    const double n=q.size();out->px/=n;out->py/=n;out->pz/=n;out->vx/=n;out->vy/=n;out->vz/=n;out->roll_deg=sr/n;out->pitch_deg=sp/n;out->yaw_deg=sy/n;reset();return true;
  }
  int count()const{return (int)q.size();}
};

Eigen::Matrix3d rpyDeg(double r,double p,double y){const double d=kPi/180.0;return(Eigen::AngleAxisd(y*d,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(p*d,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(r*d,Eigen::Vector3d::UnitX())).toRotationMatrix();}
double dist9(const VioState&a,const VioState&b){return std::sqrt((b.px-a.px)*(b.px-a.px)+(b.py-a.py)*(b.py-a.py)+(b.pz-a.pz)*(b.pz-a.pz));}

void saveV9(const std::vector<VioState>&p,const Eigen::Vector3d&t){
  std::ofstream f(kV9Csv,std::ios::trunc);if(!f)return;f<<std::fixed<<std::setprecision(9);f<<"point,target_pitch_deg,keyframe,px_m,py_m,pz_m,roll_deg,pitch_deg,yaw_deg,body_from_zero_m,lever_from_zero_m,camera_from_zero_m\n";if(p.empty())return;
  const Eigen::Vector3d p0(p[0].px,p[0].py,p[0].pz),l0=rpyDeg(p[0].roll_deg,p[0].pitch_deg,p[0].yaw_deg)*t;
  for(size_t i=0;i<p.size();++i){const auto&s=p[i];Eigen::Vector3d pb(s.px,s.py,s.pz),l=rpyDeg(s.roll_deg,s.pitch_deg,s.yaw_deg)*t,db=pb-p0,dl=l-l0,dc=db+dl;f<<i<<','<<kExpectedPitchDeg[i]<<','<<s.keyframe<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<','<<db.norm()<<','<<dl.norm()<<','<<dc.norm()<<'\n';}
}

void hudV9(const cv::Mat&gray,const HudPipeline&pipe,const FcAttitude&fc,bool armed,bool settling,int idx,const Stable9&stable,const std::vector<VioState>&pts,double ref_r,double ref_p,double ref_y,const Eigen::Vector3d&t){
  cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);cv::Mat panel(900,380,CV_8UC3,cv::Scalar(24,24,24)),c(900,1280,CV_8UC3,cv::Scalar(8,8,8));video.copyTo(c(cv::Rect(0,105,900,675)));
  ru(c,"JT-ZERO — ЭКСПЕРИМЕНТАЛЬНАЯ ПРОВЕРКА EXTRINSICS 11.5",{24,50},23,{245,245,245},cv::QT_FONT_BOLD);ru(c,"Pitch вокруг центра IMU: 0 → +45 → 0 → -45 → 0",{24,82},17,{190,190,190});char b[256];std::snprintf(b,sizeof(b),"t_BC = [%.1f %.1f %.1f] мм",t.x()*1000,t.y()*1000,t.z()*1000);ru(panel,b,{18,55},17,{220,220,220},cv::QT_FONT_BOLD);
  double rr=0,pp=0,yy=0;if(fc.valid){rr=wrapDeg(fc.roll_deg-ref_r);pp=wrapDeg(fc.pitch_deg-ref_p);yy=wrapDeg(fc.yaw_deg-ref_y);std::snprintf(b,sizeof(b),"FC ΔR %.1f  ΔP %.1f  ΔY %.1f°",rr,pp,yy);ru(panel,b,{18,100},16,{220,220,220});}VioState s;if(pipe.latest(&s)){std::snprintf(b,sizeof(b),"P [%.3f %.3f %.3f] м",s.px,s.py,s.pz);ru(panel,b,{18,145},15,{210,210,210});std::snprintf(b,sizeof(b),"|V| %.1f мм/с  KF %lld",Stable9::speed(s)*1000,(long long)s.keyframe);ru(panel,b,{18,178},14,{180,180,180});}
  if(!armed){ru(panel,"УСТАНОВИТЕ АППАРАТ В 0°",{18,245},20,{0,220,255},cv::QT_FONT_BOLD);ru(panel,"НЕ ДВИГАТЬ",{18,285},18,{245,245,245});ru(panel,"SPACE / ENTER — НАЧАТЬ",{18,340},18,{90,220,90},cv::QT_FONT_BOLD);}else if(idx>=kPointCount){ru(panel,"ТЕСТ ЗАВЕРШЁН",{18,245},23,{90,220,90},cv::QT_FONT_BOLD);ru(panel,"SPACE / ENTER — ВЫХОД",{18,300},16,{245,245,245});}else{std::snprintf(b,sizeof(b),"ТОЧКА %d/5   ЦЕЛЬ PITCH %+.0f°",idx+1,kExpectedPitchDeg[idx]);ru(panel,b,{18,245},18,{245,245,245},cv::QT_FONT_BOLD);if(settling){ru(panel,"ЦЕЛЬ ДОСТИГНУТА — НЕ ДВИГАТЬ",{18,290},16,{90,220,90},cv::QT_FONT_BOLD);std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable.count(),kStableStates);ru(panel,b,{18,330},15,{220,220,220});}else ru(panel,"ПЛАВНО ПОВЕРНИТЕ ПО PITCH",{18,290},17,{0,220,255},cv::QT_FONT_BOLD);}
  std::snprintf(b,sizeof(b),"Зафиксировано точек: %zu/5",pts.size());ru(panel,b,{18,420},16,{220,220,220});ru(panel,"Ось вращения должна проходить через IMU/FC",{18,480},13,{190,190,190});ru(panel,"Не переносить аппарат между точками",{18,510},13,{190,190,190});ru(panel,"Q / ESC — прервать",{18,750},14,{210,210,210});panel.copyTo(c(cv::Rect(900,0,380,900)));cv::imshow(kV9Window,c);
}
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";
  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,started=false,aborted=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<HudPipeline>pipe;std::thread th;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");const auto tt=vp.camera_params_.at(0).body_Pose_cam_.translation();const Eigen::Vector3d tbc(tt.x(),tt.y(),tt.z());pipe=std::make_shared<HudPipeline>(vp);pipe->installBackendCallback();th=std::thread([pipe](){pipe->spin();});started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kV9Window,cv::WINDOW_NORMAL);cv::resizeWindow(kV9Window,1280,900);
    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs(),next_hud=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));FcAttitude fc;jtzero::ImuCorrection corr;bool armed=false,settling=false;int idx=0;double ref_r=0,ref_p=0,ref_y=0;Stable9 stable;std::vector<VioState>pts;
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t recv=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample q;q.t0_rpi_ns=pending;q.t1_rpi_ns=recv;q.fc_ns=ts.tc1;q.rtt_ns=recv-pending;q.rpi_mid_ns=pending+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs)continue;const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(h.xacc,h.yacc,h.zacc),g=jtzero::ImuCorrection::gyroFrdToFlu(h.xgyro,h.ygyro,h.zgyro);const bool allow=(!armed)||settling||(idx>=kPointCount);const Eigen::Vector3d w=corr.correctGyro(h.time_usec,acc,g,allow);VIO::ImuAccGyr d;d<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),bufs[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){gray=g;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),g.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(armed&&idx<kPointCount&&fc.valid){const double rr=wrapDeg(fc.roll_deg-ref_r),pp=wrapDeg(fc.pitch_deg-ref_p),yy=wrapDeg(fc.yaw_deg-ref_y);const bool at=std::abs(pp-kExpectedPitchDeg[idx])<=kTargetTolDeg&&std::abs(rr)<=kRollTolDeg&&std::abs(yy)<=kYawTolDeg;if(at){settling=true;VioState s;if(pipe->latest(&s)){VioState cap;if(stable.update(s,&cap)){pts.push_back(cap);std::cout<<"[POINT "<<idx<<"] target="<<kExpectedPitchDeg[idx]<<" FCpitch="<<pp<<" KF="<<cap.keyframe<<" P=["<<cap.px<<","<<cap.py<<","<<cap.pz<<"]\n";++idx;settling=false;stable.reset();}}}else{settling=false;stable.reset();}}
      if(now>=next_hud){hudV9(gray,*pipe,fc,armed,settling,idx,stable,pts,ref_r,ref_p,ref_y,tbc);next_hud=now+kHudPeriodNs;}const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}if(key==32||key==13||key==10){if(!armed){if(!fc.valid)std::cout<<"[WAIT] FC ATTITUDE not ready\n";else{ref_r=fc.roll_deg;ref_p=fc.pitch_deg;ref_y=fc.yaw_deg;armed=true;idx=0;stable.reset();std::cout<<"[START] reference FC RPY=["<<ref_r<<","<<ref_p<<","<<ref_y<<"] t_BC=["<<tbc.transpose()<<"]\n";}}else if(idx>=kPointCount)break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipe->shutdown();if(th.joinable())th.join();started=false;cv::destroyAllWindows();saveV9(pts,tbc);if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&x:bufs)if(x.start&&x.start!=MAP_FAILED)munmap(x.start,x.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n================ STAGE 11.5 PITCH EXTRINSICS ================\nparams: "<<params<<"\nt_BC mm: ["<<tbc.transpose()*1000.0<<"]\npoints: "<<pts.size()<<"/5\n";if(pts.size()==5){const auto&p0=pts[0];for(int i=1;i<5;++i){const double body=dist9(p0,pts[i]);const Eigen::Vector3d l0=rpyDeg(p0.roll_deg,p0.pitch_deg,p0.yaw_deg)*tbc,li=rpyDeg(pts[i].roll_deg,pts[i].pitch_deg,pts[i].yaw_deg)*tbc;std::cout<<"POINT "<<i<<" target "<<kExpectedPitchDeg[i]<<" deg: body_from_zero="<<body*1000.0<<" mm lever_prediction="<<(li-l0).norm()*1000.0<<" mm\n";}std::cout<<"zero return #1="<<dist9(pts[0],pts[2])*1000.0<<" mm, #2="<<dist9(pts[0],pts[4])*1000.0<<" mm\n";}std::cout<<"CSV: "<<kV9Csv<<"\nOpen CSV:\n  code "<<kV9Csv<<"\n";return(!aborted&&pts.size()==5)?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipe)pipe->shutdown();if(started&&th.joinable())th.join();try{cv::destroyAllWindows();}catch(...){}if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&x:bufs)if(x.start&&x.start!=MAP_FAILED)munmap(x.start,x.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);return 1;}
}
