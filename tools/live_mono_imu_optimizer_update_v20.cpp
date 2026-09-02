// JT-ZERO v20: точная диагностика шага PRED -> OPT.
// Стендовый тест: YAW 0 -> +80 -> 0. Roll/pitch не ограничиваются программно.
// Сравнивает PIM prediction с оптимизированным состоянием и изменение bias между KF.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

namespace jtzero_v20 {
using namespace jtzero_v10;

constexpr const char* kCsv20 = "/home/vio/jtzero_live_optimizer_update_v20.csv";
constexpr const char* kWindow20 = "JT-ZERO: ОПТИМИЗАТОР v20";
constexpr int kStable20 = 20;
constexpr double kYawTarget20 = 80.0;
constexpr double kYawTol20 = 3.0;
constexpr double kGyroStill20 = 0.08;
constexpr double kAccTol20 = 0.35;

static double clamp20(double x){return std::max(-1.0,std::min(1.0,x));}
static double rotAngle20(const Eigen::Matrix3d& R){return std::acos(clamp20((R.trace()-1.0)*0.5))*180.0/kPi;}
static double gravAngle20(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  const Eigen::Vector3d z(0,0,1);
  return std::acos(clamp20((A.transpose()*z).normalized().dot((B.transpose()*z).normalized())))*180.0/kPi;
}

struct Backend20 {
  bool valid=false,have_prev=false;
  int64_t timestamp_ns=0,keyframe=0,prev_keyframe=0;
  Eigen::Matrix3d Rpred=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Ropt=Eigen::Matrix3d::Identity();
  Eigen::Vector3d Vpred=Eigen::Vector3d::Zero();
  Eigen::Vector3d Vopt=Eigen::Vector3d::Zero();
  Eigen::Vector3d BAprev=Eigen::Vector3d::Zero();
  Eigen::Vector3d BGprev=Eigen::Vector3d::Zero();
  Eigen::Vector3d BAopt=Eigen::Vector3d::Zero();
  Eigen::Vector3d BGopt=Eigen::Vector3d::Zero();
  double pred_opt_rot_deg=0,pred_opt_grav_deg=0,dv=0,dba=0,dbg=0,pim_vxy=0,opt_vxy=0;
};

class Pipeline20 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline20(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      Backend20 b; const auto& st=out->W_State_Blkf_;
      b.valid=true; b.timestamp_ns=st.timestamp_; b.keyframe=out->cur_kf_id_;
      b.Rpred=out->debug_info_.navstate_k_.pose().rotation().matrix();
      b.Vpred=out->debug_info_.navstate_k_.velocity();
      b.Ropt=st.pose_.rotation().matrix(); b.Vopt=st.velocity_;
      b.BAopt=st.imu_bias_.accelerometer(); b.BGopt=st.imu_bias_.gyroscope();
      b.pred_opt_rot_deg=rotAngle20(b.Rpred.transpose()*b.Ropt);
      b.pred_opt_grav_deg=gravAngle20(b.Rpred,b.Ropt);
      b.dv=(b.Vopt-b.Vpred).norm(); b.pim_vxy=std::hypot(b.Vpred.x(),b.Vpred.y()); b.opt_vxy=std::hypot(b.Vopt.x(),b.Vopt.y());
      std::lock_guard<std::mutex> l(m_);
      if(have_prev_){
        b.have_prev=true; b.prev_keyframe=prev_kf_; b.BAprev=prev_ba_; b.BGprev=prev_bg_;
        b.dba=(b.BAopt-b.BAprev).norm(); b.dbg=(b.BGopt-b.BGprev).norm();
      }
      latest_=b; prev_kf_=b.keyframe; prev_ba_=b.BAopt; prev_bg_=b.BGopt; have_prev_=true;
    });
  }
  bool latest(Backend20* b)const{std::lock_guard<std::mutex>l(m_);if(!latest_.valid)return false;*b=latest_;return true;}
 private:
  mutable std::mutex m_; Backend20 latest_; bool have_prev_=false; int64_t prev_kf_=0;
  Eigen::Vector3d prev_ba_=Eigen::Vector3d::Zero(),prev_bg_=Eigen::Vector3d::Zero();
};

struct Row20 {
  uint64_t imu_us=0; int64_t imu_rpi_ns=0,keyframe=0,prev_keyframe=0,backend_ns=0; double backend_age_ms=0,gyro_norm=0;
  double fc_pred_grav=0,fc_opt_grav=0,pred_opt_rot=0,pred_opt_grav=0,dv=0,dba=0,dbg=0,pim_vxy=0,opt_vxy=0;
  Eigen::Vector3d Vpred=Eigen::Vector3d::Zero(),Vopt=Eigen::Vector3d::Zero();
  Eigen::Vector3d BAprev=Eigen::Vector3d::Zero(),BAopt=Eigen::Vector3d::Zero(),BGprev=Eigen::Vector3d::Zero(),BGopt=Eigen::Vector3d::Zero();
};

struct Log20 {
  bool armed=false; Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity(); std::vector<Row20> rows;
  int64_t last_logged_kf=-1;
  double max_grav=0,max_rot=0,max_dv=0,max_dba=0,max_dbg=0,max_fc_pred=0,max_fc_opt=0;
  Row20 at_max_grav;
};

void process20(Log20& log,const ImuSample10& im,const Eigen::Matrix3d& Rfc,const Pipeline20& pipe,const ClockMapping& mapping){
  if(!log.armed)return; Backend20 b; if(!pipe.latest(&b)||!b.have_prev)return;
  if(b.keyframe==log.last_logged_kf)return; // одна строка на keyframe
  log.last_logged_kf=b.keyframe;
  const Eigen::Matrix3d RfcK=log.world_map*Rfc;
  Row20 r; r.imu_us=im.us; r.keyframe=b.keyframe; r.prev_keyframe=b.prev_keyframe; r.backend_ns=b.timestamp_ns;
  r.imu_rpi_ns=mapping.valid?mapping.map((int64_t)im.us*1000LL):0; r.backend_age_ms=r.imu_rpi_ns?nsToMs(r.imu_rpi_ns-b.timestamp_ns):0;
  r.gyro_norm=std::sqrt(im.gx*im.gx+im.gy*im.gy+im.gz*im.gz);
  r.fc_pred_grav=gravAngle20(RfcK,b.Rpred); r.fc_opt_grav=gravAngle20(RfcK,b.Ropt);
  r.pred_opt_rot=b.pred_opt_rot_deg; r.pred_opt_grav=b.pred_opt_grav_deg; r.dv=b.dv; r.dba=b.dba; r.dbg=b.dbg; r.pim_vxy=b.pim_vxy; r.opt_vxy=b.opt_vxy;
  r.Vpred=b.Vpred; r.Vopt=b.Vopt; r.BAprev=b.BAprev; r.BAopt=b.BAopt; r.BGprev=b.BGprev; r.BGopt=b.BGopt;
  log.rows.push_back(r);
  log.max_rot=std::max(log.max_rot,r.pred_opt_rot); log.max_dv=std::max(log.max_dv,r.dv); log.max_dba=std::max(log.max_dba,r.dba); log.max_dbg=std::max(log.max_dbg,r.dbg);
  log.max_fc_pred=std::max(log.max_fc_pred,r.fc_pred_grav); log.max_fc_opt=std::max(log.max_fc_opt,r.fc_opt_grav);
  if(r.pred_opt_grav>=log.max_grav){log.max_grav=r.pred_opt_grav;log.at_max_grav=r;}
}

void save20(const Log20&log){
  std::ofstream f(kCsv20,std::ios::trunc); f<<std::fixed<<std::setprecision(9);
  f<<"imu_us,imu_rpi_ns,keyframe,prev_keyframe,backend_ns,backend_age_ms,gyro_norm,fc_pred_gravity_deg,fc_opt_gravity_deg,pred_opt_rotation_deg,pred_opt_gravity_deg,dV_mps,dBA_mps2,dBG_rps,pim_vxy,opt_vxy,";
  f<<"vpred_x,vpred_y,vpred_z,vopt_x,vopt_y,vopt_z,baprev_x,baprev_y,baprev_z,baopt_x,baopt_y,baopt_z,bgprev_x,bgprev_y,bgprev_z,bgopt_x,bgopt_y,bgopt_z\n";
  for(const auto&r:log.rows){
    f<<r.imu_us<<','<<r.imu_rpi_ns<<','<<r.keyframe<<','<<r.prev_keyframe<<','<<r.backend_ns<<','<<r.backend_age_ms<<','<<r.gyro_norm<<','<<r.fc_pred_grav<<','<<r.fc_opt_grav<<','<<r.pred_opt_rot<<','<<r.pred_opt_grav<<','<<r.dv<<','<<r.dba<<','<<r.dbg<<','<<r.pim_vxy<<','<<r.opt_vxy<<',';
    f<<r.Vpred.x()<<','<<r.Vpred.y()<<','<<r.Vpred.z()<<','<<r.Vopt.x()<<','<<r.Vopt.y()<<','<<r.Vopt.z()<<',';
    f<<r.BAprev.x()<<','<<r.BAprev.y()<<','<<r.BAprev.z()<<','<<r.BAopt.x()<<','<<r.BAopt.y()<<','<<r.BAopt.z()<<',';
    f<<r.BGprev.x()<<','<<r.BGprev.y()<<','<<r.BGprev.z()<<','<<r.BGopt.x()<<','<<r.BGopt.y()<<','<<r.BGopt.z()<<'\n';
  }
}

void hud20(const cv::Mat&gray,const Telemetry&tel,bool ready,bool returning,bool done,int stable,const Log20&log,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ШАГ ОПТИМИЗАТОРА v20",20,48,.72,white,2);
  double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",190,355,.70,yellow,2);}else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);}else if(returning){uiText(c,"ВЕРНИТЕ YAW К НУЛЮ И ОСТАНОВИТЕСЬ",105,315,.68,white,2);}else{uiText(c,"ПОВЕРНИТЕ YAW К +80°",205,285,.82,yellow,3);uiText(c,"ДВИГАЙТЕ СТЕНД ПЛАВНО",205,355,.55,white,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"YAW",yy,-180,180,returning?0:kYawTarget20,kYawTol20,145);
  char b[256];std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable20);uiText(s,b,18,230,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,270,.44,gn<=kGyroStill20?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,305,.44,std::abs(an-9.81)<=kAccTol20?green:red,1);
  if(!log.rows.empty()){const auto&r=log.rows.back();std::snprintf(b,sizeof(b),"PRED→OPT gravity: %.3f°",r.pred_opt_grav);uiText(s,b,18,375,.50,r.pred_opt_grav>.5?red:green,2);std::snprintf(b,sizeof(b),"ΔV: %.3f м/с",r.dv);uiText(s,b,18,420,.48,r.dv>.2?red:green,2);std::snprintf(b,sizeof(b),"ΔBA: %.4f м/с²",r.dba);uiText(s,b,18,465,.48,r.dba>.05?red:green,2);std::snprintf(b,sizeof(b),"ΔBG: %.5f рад/с",r.dbg);uiText(s,b,18,510,.48,r.dbg>.005?red:green,2);std::snprintf(b,sizeof(b),"Vxy PIM/OPT: %.3f / %.3f",r.pim_vxy,r.opt_vxy);uiText(s,b,18,565,.44,white,1);std::snprintf(b,sizeof(b),"FC err PRED/OPT: %.2f / %.2f°",r.fc_pred_grav,r.fc_opt_grav);uiText(s,b,18,610,.42,white,1);std::snprintf(b,sizeof(b),"KF: %lld",(long long)r.keyframe);uiText(s,b,18,655,.44,muted,1);}
  uiText(s,"Цель: связать ΔR с ΔV / ΔBA / ΔBG",18,745,.38,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv20,720,910,.40,muted,1);cv::imshow(kWindow20,c);
}

} // namespace jtzero_v20

#ifndef JTZERO_V20_NO_MAIN
int main(int argc,char**argv){
  using namespace jtzero_v20; using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]); FLAGS_no_incremental_pose=true; FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline20>pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline20>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow20,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow20,1440,940);
    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Log20 log;bool ready=false,returning=false,done=false;int stable=0;double ref_accum=0;
    std::cout<<"\nJT-ZERO OPTIMIZER UPDATE v20\nSPACE fixes zero. Test: YAW +80 then return to zero.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us)process20(log,im,interpR(prev_att,cur_att,im.us),*pipe,mapping);iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double ty=returning?0:kYawTarget20;bool ok=std::abs(yy-ty)<=kYawTol20&&gn<=kGyroStill20&&std::abs(an-9.81)<=kAccTol20;if(ok)++stable;else stable=0;if(stable>=kStable20){stable=0;if(!returning){returning=true;std::cout<<"[STEP] +80 fixed. Return to zero.\n";}else{done=true;std::cout<<"[TEST] completed at zero.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud20(last_gray,tel,ready,returning,done,stable,log,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend20 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_accum=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){log.world_map=bb.Ropt*Rfc0.transpose();log.armed=true;log.last_logged_kf=-1;ready=true;std::cout<<"[ZERO] KF "<<bb.keyframe<<" BA ["<<bb.BAopt.x()<<", "<<bb.BAopt.y()<<", "<<bb.BAopt.z()<<"] BG ["<<bb.BGopt.x()<<", "<<bb.BGopt.y()<<", "<<bb.BGopt.z()<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save20(log);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    const auto&m=log.at_max_grav;
    std::cout<<"\n============================================================\nJT-ZERO OPTIMIZER UPDATE v20 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nkeyframes logged: "<<log.rows.size()<<"\nmax PRED->OPT gravity: "<<log.max_grav<<" deg\nmax PRED->OPT rotation: "<<log.max_rot<<" deg\nmax |Vopt-Vpred|: "<<log.max_dv<<" m/s\nmax |BAopt-BAprev|: "<<log.max_dba<<" m/s^2\nmax |BGopt-BGprev|: "<<log.max_dbg<<" rad/s\nmax FC gravity error PRED: "<<log.max_fc_pred<<" deg\nmax FC gravity error OPT: "<<log.max_fc_opt<<" deg\n";
    if(!log.rows.empty())std::cout<<"at max gravity: KF "<<m.keyframe<<" dV="<<m.dv<<" dBA="<<m.dba<<" dBG="<<m.dbg<<" PIM_Vxy="<<m.pim_vxy<<" OPT_Vxy="<<m.opt_vxy<<"\n";
    std::cout<<"CSV: "<<kCsv20<<"\nOpen CSV:\n  code "<<kCsv20<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
#endif
