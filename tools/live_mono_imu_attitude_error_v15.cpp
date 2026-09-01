// JT-ZERO v15: FC-vs-Kimera attitude/gravity compensation diagnostic.
// Manual test: YAW 0 -> +80 -> 0. Roll/pitch are NOT constrained during motion.
// Compares timestamp-aligned FC ATTITUDE against latest optimized/PIM Kimera rotation
// in a common world frame established at SPACE.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v15 {
using namespace jtzero_v10;

constexpr const char* kCsv15 = "/home/vio/jtzero_live_attitude_error_v15.csv";
constexpr const char* kWindow15 = "JT-ZERO: ОШИБКА ОРИЕНТАЦИИ v15";
constexpr int kStable15 = 20;
constexpr double kYawTarget15 = 80.0;
constexpr double kYawTol15 = 3.0;
constexpr double kGyroStill15 = 0.08;
constexpr double kAccTol15 = 0.35;

struct Backend15 {
  bool valid=false;
  int64_t timestamp_ns=0;
  int64_t keyframe=0;
  Eigen::Matrix3d Ropt=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rpim=Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero();
  Eigen::Vector3d bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d vopt=Eigen::Vector3d::Zero();
  Eigen::Vector3d vpim=Eigen::Vector3d::Zero();
};

class Pipeline15 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline15(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      Backend15 b;
      const auto& st=out->W_State_Blkf_;
      b.valid=true;b.timestamp_ns=st.timestamp_;b.keyframe=out->cur_kf_id_;
      b.Ropt=st.pose_.rotation().matrix();
      b.Rpim=out->debug_info_.navstate_k_.pose().rotation().matrix();
      b.ba=st.imu_bias_.accelerometer();b.bg=st.imu_bias_.gyroscope();
      b.vopt=st.velocity_;b.vpim=out->debug_info_.navstate_k_.velocity();
      std::lock_guard<std::mutex>l(m_);latest_=b;
    });
  }
  bool latest(Backend15* b)const{std::lock_guard<std::mutex>l(m_);if(!latest_.valid)return false;*b=latest_;return true;}
 private:
  mutable std::mutex m_;Backend15 latest_;
};

struct Row15 {
  uint64_t imu_us=0;int64_t imu_rpi_ns=0;double dt=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
  int64_t backend_ns=0,keyframe=0;double backend_age_ms=0;
  double gravity_err_opt_deg=0,gravity_err_pim_deg=0;
  double rot_err_opt_deg=0,rot_err_pim_deg=0;
  double afc_x=0,afc_y=0,afc_z=0,afc_xy=0;
  double aopt_x=0,aopt_y=0,aopt_z=0,aopt_xy=0;
  double apim_x=0,apim_y=0,apim_z=0,apim_xy=0;
  double derr_opt_x=0,derr_opt_y=0,derr_opt_xy=0;
  double derr_pim_x=0,derr_pim_y=0,derr_pim_xy=0;
  double verr_opt_x=0,verr_opt_y=0,verr_opt_xy=0;
  double verr_pim_x=0,verr_pim_y=0,verr_pim_xy=0;
  double backend_vxy=0,pim_vxy=0;
};

struct Integrator15 {
  bool armed=false,align_valid=false;uint64_t last_us=0;
  Eigen::Vector3d fixed_ba=Eigen::Vector3d::Zero();
  Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity();
  Eigen::Vector3d verr_opt=Eigen::Vector3d::Zero(),verr_pim=Eigen::Vector3d::Zero();
  std::vector<Row15> rows;
  double max_gravity_err_opt=0,max_gravity_err_pim=0;
  double max_rot_err_opt=0,max_rot_err_pim=0;
  double max_verr_opt=0,max_verr_pim=0;
};

static double angleDeg(const Eigen::Vector3d&a,const Eigen::Vector3d&b){
  double d=a.normalized().dot(b.normalized());d=std::max(-1.0,std::min(1.0,d));return std::acos(d)*180.0/kPi;
}
static double rotationAngleDeg(const Eigen::Matrix3d&R){
  double c=(R.trace()-1.0)*0.5;c=std::max(-1.0,std::min(1.0,c));return std::acos(c)*180.0/kPi;
}

void process15(Integrator15& in,const ImuSample10&s,const Eigen::Matrix3d&Rfc,const Pipeline15&pipe,const ClockMapping&mapping){
  if(!in.armed)return;
  double dt=0;if(in.last_us&&s.us>in.last_us)dt=(s.us-in.last_us)*1e-6;in.last_us=s.us;if(dt<0||dt>0.03)dt=0;
  Backend15 b;if(!pipe.latest(&b))return;
  const Eigen::Vector3d ac=Eigen::Vector3d(s.ax,s.ay,s.az)-in.fixed_ba;
  const Eigen::Vector3d gW(0,0,-9.81);
  const Eigen::Matrix3d RfcK=in.world_map*Rfc;
  const Eigen::Vector3d afc=RfcK*ac+gW;
  const Eigen::Vector3d aopt=b.Ropt*ac+gW;
  const Eigen::Vector3d apim=b.Rpim*ac+gW;
  const Eigen::Vector3d deOpt=aopt-afc,dePim=apim-afc;
  if(dt>0){in.verr_opt+=deOpt*dt;in.verr_pim+=dePim*dt;}
  const Eigen::Vector3d z(0,0,1);
  const double geOpt=angleDeg(RfcK.transpose()*z,b.Ropt.transpose()*z);
  const double gePim=angleDeg(RfcK.transpose()*z,b.Rpim.transpose()*z);
  const double reOpt=rotationAngleDeg(RfcK.transpose()*b.Ropt);
  const double rePim=rotationAngleDeg(RfcK.transpose()*b.Rpim);
  Row15 r;r.imu_us=s.us;r.imu_rpi_ns=mapping.valid?mapping.map((int64_t)s.us*1000LL):0;r.dt=dt;
  r.ax=s.ax;r.ay=s.ay;r.az=s.az;r.gx=s.gx;r.gy=s.gy;r.gz=s.gz;
  Eigen::Vector3d e=Rfc.eulerAngles(0,1,2)*180.0/kPi;r.fc_roll=e.x();r.fc_pitch=e.y();r.fc_yaw=e.z();
  r.backend_ns=b.timestamp_ns;r.keyframe=b.keyframe;r.backend_age_ms=r.imu_rpi_ns?nsToMs(r.imu_rpi_ns-b.timestamp_ns):0;
  r.gravity_err_opt_deg=geOpt;r.gravity_err_pim_deg=gePim;r.rot_err_opt_deg=reOpt;r.rot_err_pim_deg=rePim;
  r.afc_x=afc.x();r.afc_y=afc.y();r.afc_z=afc.z();r.afc_xy=std::hypot(afc.x(),afc.y());
  r.aopt_x=aopt.x();r.aopt_y=aopt.y();r.aopt_z=aopt.z();r.aopt_xy=std::hypot(aopt.x(),aopt.y());
  r.apim_x=apim.x();r.apim_y=apim.y();r.apim_z=apim.z();r.apim_xy=std::hypot(apim.x(),apim.y());
  r.derr_opt_x=deOpt.x();r.derr_opt_y=deOpt.y();r.derr_opt_xy=std::hypot(deOpt.x(),deOpt.y());
  r.derr_pim_x=dePim.x();r.derr_pim_y=dePim.y();r.derr_pim_xy=std::hypot(dePim.x(),dePim.y());
  r.verr_opt_x=in.verr_opt.x();r.verr_opt_y=in.verr_opt.y();r.verr_opt_xy=std::hypot(in.verr_opt.x(),in.verr_opt.y());
  r.verr_pim_x=in.verr_pim.x();r.verr_pim_y=in.verr_pim.y();r.verr_pim_xy=std::hypot(in.verr_pim.x(),in.verr_pim.y());
  r.backend_vxy=std::hypot(b.vopt.x(),b.vopt.y());r.pim_vxy=std::hypot(b.vpim.x(),b.vpim.y());
  in.rows.push_back(r);
  in.max_gravity_err_opt=std::max(in.max_gravity_err_opt,geOpt);in.max_gravity_err_pim=std::max(in.max_gravity_err_pim,gePim);
  in.max_rot_err_opt=std::max(in.max_rot_err_opt,reOpt);in.max_rot_err_pim=std::max(in.max_rot_err_pim,rePim);
  in.max_verr_opt=std::max(in.max_verr_opt,r.verr_opt_xy);in.max_verr_pim=std::max(in.max_verr_pim,r.verr_pim_xy);
}

void saveCsv15(const Integrator15&in){
  std::ofstream f(kCsv15,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"imu_us,imu_rpi_ns,dt,ax,ay,az,gx,gy,gz,fcR_roll,fcR_pitch,fcR_yaw,backend_ns,keyframe,backend_age_ms,gravity_err_opt_deg,gravity_err_pim_deg,rot_err_opt_deg,rot_err_pim_deg,a_fc_x,a_fc_y,a_fc_z,a_fc_xy,a_opt_x,a_opt_y,a_opt_z,a_opt_xy,a_pim_x,a_pim_y,a_pim_z,a_pim_xy,derr_opt_x,derr_opt_y,derr_opt_xy,derr_pim_x,derr_pim_y,derr_pim_xy,verr_opt_x,verr_opt_y,verr_opt_xy,verr_pim_x,verr_pim_y,verr_pim_xy,backend_vxy,pim_vxy\n";
  for(const auto&r:in.rows)f<<r.imu_us<<','<<r.imu_rpi_ns<<','<<r.dt<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<','<<r.backend_ns<<','<<r.keyframe<<','<<r.backend_age_ms<<','<<r.gravity_err_opt_deg<<','<<r.gravity_err_pim_deg<<','<<r.rot_err_opt_deg<<','<<r.rot_err_pim_deg<<','<<r.afc_x<<','<<r.afc_y<<','<<r.afc_z<<','<<r.afc_xy<<','<<r.aopt_x<<','<<r.aopt_y<<','<<r.aopt_z<<','<<r.aopt_xy<<','<<r.apim_x<<','<<r.apim_y<<','<<r.apim_z<<','<<r.apim_xy<<','<<r.derr_opt_x<<','<<r.derr_opt_y<<','<<r.derr_opt_xy<<','<<r.derr_pim_x<<','<<r.derr_pim_y<<','<<r.derr_pim_xy<<','<<r.verr_opt_x<<','<<r.verr_opt_y<<','<<r.verr_opt_xy<<','<<r.verr_pim_x<<','<<r.verr_pim_y<<','<<r.verr_pim_xy<<','<<r.backend_vxy<<','<<r.pim_vxy<<'\n';
}

void drawHud15(const cv::Mat&gray,const Telemetry&tel,bool ready,bool returning,bool done,int stable,const Integrator15&in,double ref_roll,double ref_pitch,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ОШИБКА ОРИЕНТАЦИИ v15",20,48,.72,white,2);
  double rr=0,pp=0,yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double ty=returning?0:kYawTarget15;
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ НЕПОДВИЖНО",150,285,.85,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);uiText(c,"SPACE — ВЫХОД",305,365,.68,white,2);}
  else if(returning)uiText(c,"ВЕРНИТЕ YAW К НУЛЮ И ОСТАНОВИТЕСЬ",105,315,.68,white,2);
  else {uiText(c,"ПОВЕРНИТЕ YAW К +80°",205,285,.82,yellow,3);uiText(c,"ROLL/PITCH МОГУТ МЕНЯТЬСЯ — ЭТО НОРМАЛЬНО",95,355,.55,white,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"YAW",yy,-180,180,ty,kYawTol15,145);drawBar(s,"PITCH",pp,-45,45,0,10,230);drawBar(s,"ROLL",rr,-45,45,0,10,315);
  char b[256];std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable15);uiText(s,b,18,380,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,420,.44,gn<=kGyroStill15?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,455,.44,std::abs(an-9.81)<=kAccTol15?green:red,1);
  if(!in.rows.empty()){const auto&r=in.rows.back();std::snprintf(b,sizeof(b),"Ошибка gravity OPT: %.2f°",r.gravity_err_opt_deg);uiText(s,b,18,515,.46,r.gravity_err_opt_deg>2?red:green,2);std::snprintf(b,sizeof(b),"Ошибка gravity PIM: %.2f°",r.gravity_err_pim_deg);uiText(s,b,18,555,.46,r.gravity_err_pim_deg>2?red:green,2);std::snprintf(b,sizeof(b),"∫Δa OPT Vxy: %.3f м/с",r.verr_opt_xy);uiText(s,b,18,610,.45,yellow,1);std::snprintf(b,sizeof(b),"∫Δa PIM Vxy: %.3f м/с",r.verr_pim_xy);uiText(s,b,18,648,.45,red,2);std::snprintf(b,sizeof(b),"Backend/PIM: %.3f / %.3f м/с",r.backend_vxy,r.pim_vxy);uiText(s,b,18,690,.42,white,1);std::snprintf(b,sizeof(b),"Возраст backend: %.1f мс",r.backend_age_ms);uiText(s,b,18,730,.40,muted,1);}
  uiText(s,"ROLL/PITCH во время движения не ограничены",18,790,.36,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv15,720,910,.40,muted,1);cv::imshow(kWindow15,c);
}

} // namespace jtzero_v15

int main(int argc,char**argv){
  using namespace jtzero_v15;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline15>pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline15>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow15,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow15,1440,940);
    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator15 integ;bool ready=false,returning=false,done=false;int stable=0;double ref_roll=0,ref_pitch=0,ref_accum=0;
    std::cout<<"\nJT-ZERO ATTITUDE ERROR v15\nSPACE fixes zero/alignment. Test: YAW +80 then return to zero. Roll/pitch may move.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us)process15(integ,im,interpR(prev_att,cur_att,im.us),*pipe,mapping);iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double ty=returning?0:kYawTarget15;bool ok=std::abs(yy-ty)<=kYawTol15&&gn<=kGyroStill15&&std::abs(an-9.81)<=kAccTol15;if(ok)++stable;else stable=0;if(stable>=kStable15){stable=0;if(!returning){returning=true;std::cout<<"[STEP] +80 fixed. Return to zero.\n";}else{done=true;std::cout<<"[TEST] completed at zero.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){drawHud15(last_gray,tel,ready,returning,done,stable,integ,ref_roll,ref_pitch,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend15 b;bool bv=pipe->latest(&b),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_roll=tel.fc_roll;ref_pitch=tel.fc_pitch;ref_accum=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){integ.fixed_ba=b.ba;integ.world_map=b.Ropt*Rfc0.transpose();integ.last_us=0;integ.verr_opt.setZero();integ.verr_pim.setZero();integ.armed=true;integ.align_valid=true;ready=true;std::cout<<"[ZERO] BA ["<<b.ba.x()<<", "<<b.ba.y()<<", "<<b.ba.z()<<"] BG ["<<b.bg.x()<<", "<<b.bg.y()<<", "<<b.bg.z()<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;saveCsv15(integ);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO ATTITUDE ERROR v15 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<integ.rows.size()<<"\nmax gravity error OPT: "<<integ.max_gravity_err_opt<<" deg\nmax gravity error PIM: "<<integ.max_gravity_err_pim<<" deg\nmax rotation error OPT: "<<integ.max_rot_err_opt<<" deg\nmax rotation error PIM: "<<integ.max_rot_err_pim<<" deg\nmax integral dV error OPT: "<<integ.max_verr_opt<<" m/s\nmax integral dV error PIM: "<<integ.max_verr_pim<<" m/s\nCSV: "<<kCsv15<<"\nOpen CSV:\n  code "<<kCsv15<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
