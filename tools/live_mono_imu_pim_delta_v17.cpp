// JT-ZERO v17: локализация ошибки ориентации PIM.
// Ручной тест: YAW 0 -> +80 -> 0, roll/pitch не ограничиваются.
// Логирует R_start, deltaR PIM, R_pred = R_start*deltaR и сравнивает gravity direction с FC.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v17 {
using namespace jtzero_v10;

constexpr const char* kCsv17 = "/home/vio/jtzero_live_pim_delta_v17.csv";
constexpr const char* kWindow17 = "JT-ZERO: PIM START + DELTAR v17";
constexpr int kStable17 = 20;
constexpr double kYawTarget17 = 80.0;
constexpr double kYawTol17 = 3.0;
constexpr double kGyroStill17 = 0.08;
constexpr double kAccTol17 = 0.35;

static double angleDeg17(const Eigen::Vector3d& a,const Eigen::Vector3d& b){
  double d=a.normalized().dot(b.normalized()); d=std::max(-1.0,std::min(1.0,d)); return std::acos(d)*180.0/kPi;
}
static double rotAngleDeg17(const Eigen::Matrix3d& R){
  double c=(R.trace()-1.0)*0.5; c=std::max(-1.0,std::min(1.0,c)); return std::acos(c)*180.0/kPi;
}

struct Backend17 {
  bool valid=false;
  int64_t timestamp_ns=0,keyframe=0;
  Eigen::Matrix3d Ropt=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rpim=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d dR=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rstart=Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero(),bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d vpim=Eigen::Vector3d::Zero();
};

class Pipeline17 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline17(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out) return;
      Backend17 b; const auto& st=out->W_State_Blkf_;
      b.valid=true; b.timestamp_ns=st.timestamp_; b.keyframe=out->cur_kf_id_;
      b.Ropt=st.pose_.rotation().matrix();
      b.Rpim=out->debug_info_.navstate_k_.pose().rotation().matrix();
      b.dR=out->debug_info_.imuR_lkf_kf.matrix();
      b.Rstart=b.Rpim*b.dR.transpose();
      b.ba=st.imu_bias_.accelerometer(); b.bg=st.imu_bias_.gyroscope();
      b.vpim=out->debug_info_.navstate_k_.velocity();
      std::lock_guard<std::mutex> l(m_); latest_=b;
    });
  }
  bool latest(Backend17* b) const { std::lock_guard<std::mutex> l(m_); if(!latest_.valid) return false; *b=latest_; return true; }
 private:
  mutable std::mutex m_; Backend17 latest_;
};

struct Row17 {
  uint64_t imu_us=0; int64_t imu_rpi_ns=0,keyframe=0,backend_ns=0; double dt=0,backend_age_ms=0;
  double gx=0,gy=0,gz=0,gyro_norm=0;
  double fc_grav_err_start=0,fc_grav_err_pred=0,fc_grav_err_opt=0;
  double deltaR_deg=0,recompose_err_deg=0;
  double start_to_opt_deg=0,pred_to_opt_deg=0;
  double pim_vxy=0;
};

struct Log17 {
  bool armed=false; uint64_t last_us=0; Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity();
  std::vector<Row17> rows;
  double max_start_g=0,max_pred_g=0,max_opt_g=0,max_recompose=0,max_deltaR=0;
};

void process17(Log17& log,const ImuSample10& im,const Eigen::Matrix3d& Rfc,const Pipeline17& pipe,const ClockMapping& mapping){
  if(!log.armed) return;
  Backend17 b; if(!pipe.latest(&b)) return;
  double dt=0; if(log.last_us && im.us>log.last_us) dt=(im.us-log.last_us)*1e-6; log.last_us=im.us; if(dt<0||dt>0.03)dt=0;
  const Eigen::Matrix3d RfcK=log.world_map*Rfc;
  const Eigen::Vector3d z(0,0,1);
  Row17 r; r.imu_us=im.us; r.dt=dt; r.keyframe=b.keyframe; r.backend_ns=b.timestamp_ns;
  r.imu_rpi_ns=mapping.valid?mapping.map((int64_t)im.us*1000LL):0;
  r.backend_age_ms=r.imu_rpi_ns?nsToMs(r.imu_rpi_ns-b.timestamp_ns):0;
  r.gx=im.gx;r.gy=im.gy;r.gz=im.gz;r.gyro_norm=std::sqrt(im.gx*im.gx+im.gy*im.gy+im.gz*im.gz);
  r.fc_grav_err_start=angleDeg17(RfcK.transpose()*z,b.Rstart.transpose()*z);
  r.fc_grav_err_pred=angleDeg17(RfcK.transpose()*z,b.Rpim.transpose()*z);
  r.fc_grav_err_opt=angleDeg17(RfcK.transpose()*z,b.Ropt.transpose()*z);
  r.deltaR_deg=rotAngleDeg17(b.dR);
  r.recompose_err_deg=rotAngleDeg17((b.Rstart*b.dR).transpose()*b.Rpim);
  r.start_to_opt_deg=rotAngleDeg17(b.Rstart.transpose()*b.Ropt);
  r.pred_to_opt_deg=rotAngleDeg17(b.Rpim.transpose()*b.Ropt);
  r.pim_vxy=std::hypot(b.vpim.x(),b.vpim.y());
  log.rows.push_back(r);
  log.max_start_g=std::max(log.max_start_g,r.fc_grav_err_start); log.max_pred_g=std::max(log.max_pred_g,r.fc_grav_err_pred);
  log.max_opt_g=std::max(log.max_opt_g,r.fc_grav_err_opt); log.max_recompose=std::max(log.max_recompose,r.recompose_err_deg); log.max_deltaR=std::max(log.max_deltaR,r.deltaR_deg);
}

void save17(const Log17& log){
  std::ofstream f(kCsv17,std::ios::trunc); f<<std::fixed<<std::setprecision(9);
  f<<"imu_us,imu_rpi_ns,dt,keyframe,backend_ns,backend_age_ms,gx,gy,gz,gyro_norm,gravity_err_start_deg,gravity_err_pred_deg,gravity_err_opt_deg,deltaR_deg,recompose_err_deg,start_to_opt_deg,pred_to_opt_deg,pim_vxy\n";
  for(const auto&r:log.rows) f<<r.imu_us<<','<<r.imu_rpi_ns<<','<<r.dt<<','<<r.keyframe<<','<<r.backend_ns<<','<<r.backend_age_ms<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.gyro_norm<<','<<r.fc_grav_err_start<<','<<r.fc_grav_err_pred<<','<<r.fc_grav_err_opt<<','<<r.deltaR_deg<<','<<r.recompose_err_deg<<','<<r.start_to_opt_deg<<','<<r.pred_to_opt_deg<<','<<r.pim_vxy<<'\n';
}

void hud17(const cv::Mat& gray,const Telemetry& tel,bool ready,bool returning,bool done,int stable,const Log17& log,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video; if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST); video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: PIM START + DELTAR v17",20,48,.72,white,2);
  double yy=0,gn=0,an=0; {std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ НЕПОДВИЖНО",150,285,.85,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",190,355,.70,yellow,2);}else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);}else if(returning){uiText(c,"ВЕРНИТЕ YAW К НУЛЮ И ОСТАНОВИТЕСЬ",105,315,.68,white,2);}else{uiText(c,"ПОВЕРНИТЕ YAW К +80°",205,285,.82,yellow,3);uiText(c,"ROLL/PITCH НЕ ОГРАНИЧЕНЫ",205,355,.55,white,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850)); cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1); drawBar(s,"YAW",yy,-180,180,returning?0:kYawTarget17,kYawTol17,145);
  char b[256]; std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable17);uiText(s,b,18,230,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,270,.44,gn<=kGyroStill17?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,305,.44,std::abs(an-9.81)<=kAccTol17?green:red,1);
  if(!log.rows.empty()){const auto&r=log.rows.back();std::snprintf(b,sizeof(b),"Gravity START: %.2f°",r.fc_grav_err_start);uiText(s,b,18,380,.50,r.fc_grav_err_start>2?red:green,2);std::snprintf(b,sizeof(b),"Gravity PRED: %.2f°",r.fc_grav_err_pred);uiText(s,b,18,425,.50,r.fc_grav_err_pred>2?red:green,2);std::snprintf(b,sizeof(b),"Gravity OPT: %.2f°",r.fc_grav_err_opt);uiText(s,b,18,470,.50,r.fc_grav_err_opt>2?red:green,2);std::snprintf(b,sizeof(b),"deltaR: %.2f°",r.deltaR_deg);uiText(s,b,18,530,.48,yellow,2);std::snprintf(b,sizeof(b),"recompose err: %.6f°",r.recompose_err_deg);uiText(s,b,18,570,.44,white,1);std::snprintf(b,sizeof(b),"PIM Vxy: %.3f м/с",r.pim_vxy);uiText(s,b,18,625,.48,white,1);std::snprintf(b,sizeof(b),"KF: %lld",(long long)r.keyframe);uiText(s,b,18,670,.44,muted,1);}
  uiText(s,"Цель: где рождается ошибка gravity",18,745,.40,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv17,720,910,.40,muted,1);cv::imshow(kWindow17,c);
}

} // namespace jtzero_v17

int main(int argc,char**argv){
  using namespace jtzero_v17; using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]); FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1; bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false; uint8_t sys=0,comp=0; std::vector<CameraBuffer>bufs; std::shared_ptr<Pipeline17>pipe; std::thread pipe_thread; Telemetry tel;
  try{
    VIO::VioParams vp(params); pipe=std::make_shared<Pipeline17>(vp); pipe->installCallback(); pipe_thread=std::thread([pipe](){pipe->spin();}); pipeline_started=true;
    sfd=openSerial(); mavlink_status_t mst{}; mavlink_message_t msg{}; std::cout<<"[MAV] waiting for HEARTBEAT...\n"; int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout"); requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true; requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow17,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow17,1440,940);
    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Log17 log;bool ready=false,returning=false,done=false;int stable=0;double ref_accum=0;
    std::cout<<"\nJT-ZERO PIM START + DELTAR v17\nSPACE fixes zero. Test: YAW +80 then return to zero. Roll/pitch may move.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us)process17(log,im,interpR(prev_att,cur_att,im.us),*pipe,mapping);iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double ty=returning?0:kYawTarget17;bool ok=std::abs(yy-ty)<=kYawTol17&&gn<=kGyroStill17&&std::abs(an-9.81)<=kAccTol17;if(ok)++stable;else stable=0;if(stable>=kStable17){stable=0;if(!returning){returning=true;std::cout<<"[STEP] +80 fixed. Return to zero.\n";}else{done=true;std::cout<<"[TEST] completed at zero.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud17(last_gray,tel,ready,returning,done,stable,log,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend17 b;bool bv=pipe->latest(&b),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_accum=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){log.world_map=b.Ropt*Rfc0.transpose();log.last_us=0;log.armed=true;ready=true;std::cout<<"[ZERO] KF "<<b.keyframe<<" BA ["<<b.ba.x()<<", "<<b.ba.y()<<", "<<b.ba.z()<<"] BG ["<<b.bg.x()<<", "<<b.bg.y()<<", "<<b.bg.z()<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save17(log);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO PIM START + DELTAR v17 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<log.rows.size()<<"\nmax gravity error START: "<<log.max_start_g<<" deg\nmax gravity error PRED: "<<log.max_pred_g<<" deg\nmax gravity error OPT: "<<log.max_opt_g<<" deg\nmax deltaR angle: "<<log.max_deltaR<<" deg\nmax recompose error: "<<log.max_recompose<<" deg\nCSV: "<<kCsv17<<"\nOpen CSV:\n  code "<<kCsv17<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
