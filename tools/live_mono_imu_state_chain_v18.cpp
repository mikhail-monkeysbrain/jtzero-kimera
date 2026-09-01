// JT-ZERO v18: проверка цепочки OPT(k-1) -> START(k) -> PRED(k) -> OPT(k).
// Ручной тест: YAW 0 -> +80 -> 0. Roll/pitch не ограничиваются.
// Главная метрика: совпадает ли START(k) с OPT(k-1), и на каком переходе растёт ошибка gravity.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v18 {
using namespace jtzero_v10;

constexpr const char* kCsv18 = "/home/vio/jtzero_live_state_chain_v18.csv";
constexpr const char* kWindow18 = "JT-ZERO: ЦЕПОЧКА СОСТОЯНИЙ v18";
constexpr int kStable18 = 20;
constexpr double kYawTarget18 = 80.0;
constexpr double kYawTol18 = 3.0;
constexpr double kGyroStill18 = 0.08;
constexpr double kAccTol18 = 0.35;

static double clamp18(double x){return std::max(-1.0,std::min(1.0,x));}
static double rotAngle18(const Eigen::Matrix3d& R){return std::acos(clamp18((R.trace()-1.0)*0.5))*180.0/kPi;}
static double gravAngle18(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  const Eigen::Vector3d z(0,0,1); return std::acos(clamp18((A.transpose()*z).normalized().dot((B.transpose()*z).normalized())))*180.0/kPi;
}

struct Backend18 {
  bool valid=false,have_prev=false;
  int64_t timestamp_ns=0,keyframe=0,prev_keyframe=0;
  Eigen::Matrix3d RprevOpt=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rstart=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rpred=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Ropt=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d dR=Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero(),bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d vpred=Eigen::Vector3d::Zero();
  double prevopt_to_start_rot_deg=0;
  double prevopt_to_start_grav_deg=0;
  double start_to_pred_grav_deg=0;
  double pred_to_opt_grav_deg=0;
  double start_to_opt_grav_deg=0;
  double recompose_err_deg=0;
};

class Pipeline18 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline18(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      Backend18 b; const auto& st=out->W_State_Blkf_;
      b.valid=true;b.timestamp_ns=st.timestamp_;b.keyframe=out->cur_kf_id_;
      b.Ropt=st.pose_.rotation().matrix();
      b.Rpred=out->debug_info_.navstate_k_.pose().rotation().matrix();
      b.dR=out->debug_info_.imuR_lkf_kf.matrix();
      b.Rstart=b.Rpred*b.dR.transpose();
      b.ba=st.imu_bias_.accelerometer();b.bg=st.imu_bias_.gyroscope();
      b.vpred=out->debug_info_.navstate_k_.velocity();
      {
        std::lock_guard<std::mutex> l(m_);
        if(have_prev_){
          b.have_prev=true;b.prev_keyframe=prev_kf_;b.RprevOpt=prev_opt_;
          b.prevopt_to_start_rot_deg=rotAngle18(prev_opt_.transpose()*b.Rstart);
          b.prevopt_to_start_grav_deg=gravAngle18(prev_opt_,b.Rstart);
        }
        b.start_to_pred_grav_deg=gravAngle18(b.Rstart,b.Rpred);
        b.pred_to_opt_grav_deg=gravAngle18(b.Rpred,b.Ropt);
        b.start_to_opt_grav_deg=gravAngle18(b.Rstart,b.Ropt);
        b.recompose_err_deg=rotAngle18((b.Rstart*b.dR).transpose()*b.Rpred);
        latest_=b; prev_opt_=b.Ropt; prev_kf_=b.keyframe; have_prev_=true;
      }
    });
  }
  bool latest(Backend18* b)const{std::lock_guard<std::mutex>l(m_);if(!latest_.valid)return false;*b=latest_;return true;}
 private:
  mutable std::mutex m_;Backend18 latest_;bool have_prev_=false;int64_t prev_kf_=0;Eigen::Matrix3d prev_opt_=Eigen::Matrix3d::Identity();
};

struct Row18 {
  uint64_t imu_us=0;int64_t imu_rpi_ns=0,keyframe=0,prev_keyframe=0,backend_ns=0;double dt=0,backend_age_ms=0,gyro_norm=0;
  double fc_err_prevopt=0,fc_err_start=0,fc_err_pred=0,fc_err_opt=0;
  double prevopt_to_start_rot=0,prevopt_to_start_grav=0,start_to_pred_grav=0,pred_to_opt_grav=0,start_to_opt_grav=0,recompose_err=0;
  double deltaR_deg=0,pim_vxy=0;
};
struct Log18 {
  bool armed=false;uint64_t last_us=0;Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity();std::vector<Row18>rows;
  double max_prev_start_rot=0,max_prev_start_grav=0,max_pred_opt_grav=0,max_start_pred_grav=0,max_fc_start=0,max_fc_pred=0,max_fc_opt=0,max_recompose=0;
};

void process18(Log18& log,const ImuSample10& im,const Eigen::Matrix3d& Rfc,const Pipeline18& pipe,const ClockMapping& mapping){
  if(!log.armed)return;Backend18 b;if(!pipe.latest(&b))return;
  double dt=0;if(log.last_us&&im.us>log.last_us)dt=(im.us-log.last_us)*1e-6;log.last_us=im.us;if(dt<0||dt>0.03)dt=0;
  const Eigen::Matrix3d RfcK=log.world_map*Rfc;
  Row18 r;r.imu_us=im.us;r.dt=dt;r.keyframe=b.keyframe;r.prev_keyframe=b.prev_keyframe;r.backend_ns=b.timestamp_ns;
  r.imu_rpi_ns=mapping.valid?mapping.map((int64_t)im.us*1000LL):0;r.backend_age_ms=r.imu_rpi_ns?nsToMs(r.imu_rpi_ns-b.timestamp_ns):0;
  r.gyro_norm=std::sqrt(im.gx*im.gx+im.gy*im.gy+im.gz*im.gz);
  if(b.have_prev)r.fc_err_prevopt=gravAngle18(RfcK,b.RprevOpt);
  r.fc_err_start=gravAngle18(RfcK,b.Rstart);r.fc_err_pred=gravAngle18(RfcK,b.Rpred);r.fc_err_opt=gravAngle18(RfcK,b.Ropt);
  r.prevopt_to_start_rot=b.prevopt_to_start_rot_deg;r.prevopt_to_start_grav=b.prevopt_to_start_grav_deg;r.start_to_pred_grav=b.start_to_pred_grav_deg;
  r.pred_to_opt_grav=b.pred_to_opt_grav_deg;r.start_to_opt_grav=b.start_to_opt_grav_deg;r.recompose_err=b.recompose_err_deg;
  r.deltaR_deg=rotAngle18(b.dR);r.pim_vxy=std::hypot(b.vpred.x(),b.vpred.y());log.rows.push_back(r);
  log.max_prev_start_rot=std::max(log.max_prev_start_rot,r.prevopt_to_start_rot);log.max_prev_start_grav=std::max(log.max_prev_start_grav,r.prevopt_to_start_grav);
  log.max_pred_opt_grav=std::max(log.max_pred_opt_grav,r.pred_to_opt_grav);log.max_start_pred_grav=std::max(log.max_start_pred_grav,r.start_to_pred_grav);
  log.max_fc_start=std::max(log.max_fc_start,r.fc_err_start);log.max_fc_pred=std::max(log.max_fc_pred,r.fc_err_pred);log.max_fc_opt=std::max(log.max_fc_opt,r.fc_err_opt);log.max_recompose=std::max(log.max_recompose,r.recompose_err);
}

void save18(const Log18&log){
  std::ofstream f(kCsv18,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"imu_us,imu_rpi_ns,dt,keyframe,prev_keyframe,backend_ns,backend_age_ms,gyro_norm,fc_gravity_err_prevopt_deg,fc_gravity_err_start_deg,fc_gravity_err_pred_deg,fc_gravity_err_opt_deg,prevopt_to_start_rot_deg,prevopt_to_start_gravity_deg,start_to_pred_gravity_deg,pred_to_opt_gravity_deg,start_to_opt_gravity_deg,recompose_err_deg,deltaR_deg,pim_vxy\n";
  for(const auto&r:log.rows)f<<r.imu_us<<','<<r.imu_rpi_ns<<','<<r.dt<<','<<r.keyframe<<','<<r.prev_keyframe<<','<<r.backend_ns<<','<<r.backend_age_ms<<','<<r.gyro_norm<<','<<r.fc_err_prevopt<<','<<r.fc_err_start<<','<<r.fc_err_pred<<','<<r.fc_err_opt<<','<<r.prevopt_to_start_rot<<','<<r.prevopt_to_start_grav<<','<<r.start_to_pred_grav<<','<<r.pred_to_opt_grav<<','<<r.start_to_opt_grav<<','<<r.recompose_err<<','<<r.deltaR_deg<<','<<r.pim_vxy<<'\n';
}

void hud18(const cv::Mat&gray,const Telemetry&tel,bool ready,bool returning,bool done,int stable,const Log18&log,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ЦЕПОЧКА OPT→START→PRED→OPT v18",20,48,.68,white,2);
  double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ НЕПОДВИЖНО",150,285,.85,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",190,355,.70,yellow,2);}else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);}else if(returning){uiText(c,"ВЕРНИТЕ YAW К НУЛЮ И ОСТАНОВИТЕСЬ",105,315,.68,white,2);}else{uiText(c,"ПОВЕРНИТЕ YAW К +80°",205,285,.82,yellow,3);uiText(c,"ROLL/PITCH НЕ ОГРАНИЧЕНЫ",205,355,.55,white,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"YAW",yy,-180,180,returning?0:kYawTarget18,kYawTol18,145);
  char b[256];std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable18);uiText(s,b,18,230,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,270,.44,gn<=kGyroStill18?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,305,.44,std::abs(an-9.81)<=kAccTol18?green:red,1);
  if(!log.rows.empty()){const auto&r=log.rows.back();std::snprintf(b,sizeof(b),"OPT(k-1)→START grav: %.3f°",r.prevopt_to_start_grav);uiText(s,b,18,370,.45,r.prevopt_to_start_grav>.5?red:green,2);std::snprintf(b,sizeof(b),"START→PRED grav: %.3f°",r.start_to_pred_grav);uiText(s,b,18,412,.45,yellow,2);std::snprintf(b,sizeof(b),"PRED→OPT grav: %.3f°",r.pred_to_opt_grav);uiText(s,b,18,454,.45,r.pred_to_opt_grav>.5?red:green,2);std::snprintf(b,sizeof(b),"FC err START/PRED/OPT: %.2f / %.2f / %.2f°",r.fc_err_start,r.fc_err_pred,r.fc_err_opt);uiText(s,b,18,512,.38,white,1);std::snprintf(b,sizeof(b),"OPTprev→START rot: %.4f°",r.prevopt_to_start_rot);uiText(s,b,18,555,.42,white,1);std::snprintf(b,sizeof(b),"recompose: %.7f°",r.recompose_err);uiText(s,b,18,595,.42,white,1);std::snprintf(b,sizeof(b),"PIM Vxy: %.3f м/с",r.pim_vxy);uiText(s,b,18,640,.46,white,1);std::snprintf(b,sizeof(b),"KF: %lld ← %lld",(long long)r.keyframe,(long long)r.prev_keyframe);uiText(s,b,18,682,.42,muted,1);}
  uiText(s,"Цель: найти переход, портящий gravity",18,755,.38,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv18,720,910,.40,muted,1);cv::imshow(kWindow18,c);
}
}

int main(int argc,char**argv){
  using namespace jtzero_v18;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline18>pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline18>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow18,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow18,1440,940);
    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Log18 log;bool ready=false,returning=false,done=false;int stable=0;double ref_accum=0;
    std::cout<<"\nJT-ZERO STATE CHAIN v18\nSPACE fixes zero. Test: YAW +80 then return to zero. Roll/pitch may move.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us)process18(log,im,interpR(prev_att,cur_att,im.us),*pipe,mapping);iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double ty=returning?0:kYawTarget18;bool ok=std::abs(yy-ty)<=kYawTol18&&gn<=kGyroStill18&&std::abs(an-9.81)<=kAccTol18;if(ok)++stable;else stable=0;if(stable>=kStable18){stable=0;if(!returning){returning=true;std::cout<<"[STEP] +80 fixed. Return to zero.\n";}else{done=true;std::cout<<"[TEST] completed at zero.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud18(last_gray,tel,ready,returning,done,stable,log,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend18 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_accum=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){log.world_map=bb.Ropt*Rfc0.transpose();log.last_us=0;log.armed=true;ready=true;std::cout<<"[ZERO] KF "<<bb.keyframe<<" BA ["<<bb.ba.x()<<", "<<bb.ba.y()<<", "<<bb.ba.z()<<"] BG ["<<bb.bg.x()<<", "<<bb.bg.y()<<", "<<bb.bg.z()<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save18(log);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO STATE CHAIN v18 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<log.rows.size()<<"\nmax OPT(k-1)->START rotation: "<<log.max_prev_start_rot<<" deg\nmax OPT(k-1)->START gravity: "<<log.max_prev_start_grav<<" deg\nmax START->PRED gravity: "<<log.max_start_pred_grav<<" deg\nmax PRED->OPT gravity: "<<log.max_pred_opt_grav<<" deg\nmax FC gravity error START: "<<log.max_fc_start<<" deg\nmax FC gravity error PRED: "<<log.max_fc_pred<<" deg\nmax FC gravity error OPT: "<<log.max_fc_opt<<" deg\nmax recompose error: "<<log.max_recompose<<" deg\nCSV: "<<kCsv18<<"\nOpen CSV:\n  code "<<kCsv18<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
