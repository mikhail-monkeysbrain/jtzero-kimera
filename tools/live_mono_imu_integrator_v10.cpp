// JT-ZERO v10: per-sample IMU A/B integrator diagnostic.
// Compares GTSAM PIM against two independent integrations of the same HIGHRES_IMU stream:
//   A) orientation integrated from gyro_FLU with fixed startup bias
//   B) FC ATTITUDE interpolated in FC boot-time domain

#define JTZERO_V8_NO_MAIN
#include "live_mono_imu_ypr_dashboard_v8.cpp"
#undef JTZERO_V8_NO_MAIN

#include <deque>
#include <Eigen/Geometry>

namespace jtzero_v10 {

constexpr const char* kCsv10 = "/home/vio/jtzero_live_imu_integrator_v10.csv";
constexpr const char* kWindow10 = "JT-ZERO: IMU INTEGRATOR v10";
constexpr int kStable10 = 20;
constexpr double kGyroStill10 = 0.08;
constexpr double kAccTol10 = 0.35;

struct Backend10 {
  bool valid=false;
  double bax=0,bay=0,baz=0,bgx=0,bgy=0,bgz=0;
  double vx=0,vy=0,vz=0,pim_vxy=0;
};

class Pipeline10 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline10(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      Backend10 b;
      const auto& st=out->W_State_Blkf_;
      const auto ba=st.imu_bias_.accelerometer();
      const auto bg=st.imu_bias_.gyroscope();
      b.valid=true;b.bax=ba.x();b.bay=ba.y();b.baz=ba.z();b.bgx=bg.x();b.bgy=bg.y();b.bgz=bg.z();
      b.vx=st.velocity_.x();b.vy=st.velocity_.y();b.vz=st.velocity_.z();
      const auto& pv=out->debug_info_.navstate_k_.velocity();
      b.pim_vxy=std::hypot(pv.x(),pv.y());
      std::lock_guard<std::mutex> l(m_);latest_=b;
    });
  }
  bool latest(Backend10* b)const{std::lock_guard<std::mutex>l(m_);if(!latest_.valid)return false;*b=latest_;return true;}
 private:
  mutable std::mutex m_;
  Backend10 latest_;
};

struct AttSample10 {uint64_t us=0;double roll=0,pitch=0,yaw=0;};
struct ImuSample10 {uint64_t us=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;};

struct Row10 {
  uint64_t imu_us=0;double dt=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
  double gyro_roll=0,gyro_pitch=0,gyro_yaw=0;
  double afx=0,afy=0,afz=0,agx=0,agy=0,agz=0;
  double vfx=0,vfy=0,vfz=0,vgx=0,vgy=0,vgz=0;
  double backend_vxy=0,pim_vxy=0;
};

struct Integrator10 {
  bool armed=false;
  uint64_t last_us=0;
  Eigen::Matrix3d Rgyro=Eigen::Matrix3d::Identity();
  Eigen::Vector3d fixed_ba=Eigen::Vector3d::Zero();
  Eigen::Vector3d fixed_bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d vfc=Eigen::Vector3d::Zero();
  Eigen::Vector3d vgyro=Eigen::Vector3d::Zero();
  std::vector<Row10> rows;
  double max_fc_vxy=0,max_gyro_vxy=0,max_backend_vxy=0,max_pim_vxy=0;
};

Eigen::Matrix3d frdToFlu(){Eigen::Matrix3d D=Eigen::Matrix3d::Identity();D(1,1)=-1;D(2,2)=-1;return D;}

Eigen::Matrix3d fcRnedFlu(double roll_deg,double pitch_deg,double yaw_deg){
  const double r=roll_deg*kPi/180.0,p=pitch_deg*kPi/180.0,y=yaw_deg*kPi/180.0;
  const Eigen::Matrix3d Rz=Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d Ry=Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d Rx=Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX()).toRotationMatrix();
  return Rz*Ry*Rx*frdToFlu();
}

Eigen::Vector3d rpyDeg(const Eigen::Matrix3d& R){
  Eigen::Vector3d e=R.eulerAngles(0,1,2);
  return e*180.0/kPi;
}

Eigen::Matrix3d interpR(const AttSample10& a,const AttSample10& b,uint64_t us){
  if(b.us<=a.us)return fcRnedFlu(b.roll,b.pitch,b.yaw);
  double t=(double)(us-a.us)/(double)(b.us-a.us);t=std::max(0.0,std::min(1.0,t));
  Eigen::Quaterniond qa(fcRnedFlu(a.roll,a.pitch,a.yaw)),qb(fcRnedFlu(b.roll,b.pitch,b.yaw));
  return qa.slerp(t,qb).normalized().toRotationMatrix();
}

void processOne(Integrator10& in,const ImuSample10& s,const Eigen::Matrix3d& Rfc,const Pipeline10& pipe){
  if(!in.armed)return;
  double dt=0;if(in.last_us&&s.us>in.last_us)dt=(s.us-in.last_us)*1e-6;in.last_us=s.us;
  if(dt<0||dt>0.03)dt=0;
  const Eigen::Vector3d acc(s.ax,s.ay,s.az),gyr(s.gx,s.gy,s.gz);
  const Eigen::Vector3d ac=acc-in.fixed_ba,w=gyr-in.fixed_bg;
  if(dt>0){const Eigen::Vector3d th=w*dt;const double a=th.norm();if(a>1e-12)in.Rgyro=in.Rgyro*Eigen::AngleAxisd(a,th/a).toRotationMatrix();}
  const Eigen::Vector3d gN(0,0,9.81);
  const Eigen::Vector3d af=Rfc*ac+gN;
  const Eigen::Vector3d ag=in.Rgyro*ac+gN;
  if(dt>0){in.vfc+=af*dt;in.vgyro+=ag*dt;}
  Backend10 b;pipe.latest(&b);
  Row10 r;r.imu_us=s.us;r.dt=dt;r.ax=s.ax;r.ay=s.ay;r.az=s.az;r.gx=s.gx;r.gy=s.gy;r.gz=s.gz;
  Eigen::Quaterniond qf(Rfc);Eigen::Vector3d ef=Rfc.eulerAngles(0,1,2)*180.0/kPi;Eigen::Vector3d eg=in.Rgyro.eulerAngles(0,1,2)*180.0/kPi;
  r.fc_roll=ef.x();r.fc_pitch=ef.y();r.fc_yaw=ef.z();r.gyro_roll=eg.x();r.gyro_pitch=eg.y();r.gyro_yaw=eg.z();
  r.afx=af.x();r.afy=af.y();r.afz=af.z();r.agx=ag.x();r.agy=ag.y();r.agz=ag.z();
  r.vfx=in.vfc.x();r.vfy=in.vfc.y();r.vfz=in.vfc.z();r.vgx=in.vgyro.x();r.vgy=in.vgyro.y();r.vgz=in.vgyro.z();
  if(b.valid){r.backend_vxy=std::hypot(b.vx,b.vy);r.pim_vxy=b.pim_vxy;}
  in.rows.push_back(r);
  in.max_fc_vxy=std::max(in.max_fc_vxy,std::hypot(r.vfx,r.vfy));in.max_gyro_vxy=std::max(in.max_gyro_vxy,std::hypot(r.vgx,r.vgy));
  in.max_backend_vxy=std::max(in.max_backend_vxy,r.backend_vxy);in.max_pim_vxy=std::max(in.max_pim_vxy,r.pim_vxy);
}

void saveCsv(const Integrator10& in){
  std::ofstream f(kCsv10,std::ios::trunc);
  f<<"imu_us,dt,ax,ay,az,gx,gy,gz,fcR_roll,fcR_pitch,fcR_yaw,gyroR_roll,gyroR_pitch,gyroR_yaw,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,backend_vxy,pim_vxy\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&r:in.rows)f<<r.imu_us<<','<<r.dt<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<','<<r.gyro_roll<<','<<r.gyro_pitch<<','<<r.gyro_yaw<<','<<r.afx<<','<<r.afy<<','<<r.afz<<','<<r.agx<<','<<r.agy<<','<<r.agz<<','<<r.vfx<<','<<r.vfy<<','<<r.vfz<<','<<r.vgx<<','<<r.vgy<<','<<r.vgz<<','<<r.backend_vxy<<','<<r.pim_vxy<<'\n';
}

void drawHud(const cv::Mat& gray,const Telemetry& tel,bool ready,bool returning,bool done,int stable,const Integrator10& in,double ref_roll,double ref_pitch,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ПОКАДРОВЫЙ IMU ИНТЕГРАТОР v10",20,48,.72,white,2);
  double rr=0,pp=0,yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double ty=returning?0:80,tp=0,tr=0,tol=returning?3:2;
  bool target=ready&&!done&&std::abs(yy-ty)<=tol&&std::abs(pp)<=3&&std::abs(rr)<=3;
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ НЕПОДВИЖНО",150,285,.85,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);uiText(c,"SPACE — ВЫХОД",305,365,.68,white,2);}
  else if(target){uiText(c,"СТОП! НЕ ДВИГАТЬ",215,300,1.05,green,3);}
  else if(returning)uiText(c,"ВЕРНИТЕ YAW / PITCH / ROLL К НУЛЮ",90,315,.70,white,2);else uiText(c,"ПОВЕРНИТЕ YAW К +80°, PITCH/ROLL ОКОЛО НУЛЯ",55,315,.62,white,2);
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);
  drawBar(s,"YAW",yy,-180,180,ty,tol,150);drawBar(s,"PITCH",pp,-45,45,tp,3,235);drawBar(s,"ROLL",rr,-45,45,tr,3,320);
  char b[256];std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable10);uiText(s,b,18,390,.44,white,1);
  std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,430,.44,gn<=kGyroStill10?green:red,1);
  std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,465,.44,std::abs(an-9.81)<=kAccTol10?green:red,1);
  std::snprintf(b,sizeof(b),"FC-интегратор Vxy: %.3f m/s",in.vfc.head<2>().norm());uiText(s,b,18,525,.48,green,2);
  std::snprintf(b,sizeof(b),"GYRO-интегратор Vxy: %.3f m/s",in.vgyro.head<2>().norm());uiText(s,b,18,565,.48,yellow,2);
  if(!in.rows.empty()){const auto&r=in.rows.back();std::snprintf(b,sizeof(b),"Backend Vxy: %.3f m/s",r.backend_vxy);uiText(s,b,18,620,.46,white,1);std::snprintf(b,sizeof(b),"PIM Vxy: %.3f m/s",r.pim_vxy);uiText(s,b,18,655,.46,red,1);}
  uiText(s,"Цель: выяснить, где рождается скорость",18,720,.38,muted,1);uiText(s,"FC vs GYRO vs GTSAM PIM",18,750,.42,white,1);
  uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv10,720,910,.40,muted,1);cv::imshow(kWindow10,c);
}

} // namespace jtzero_v10

#ifndef JTZERO_V10_NO_MAIN
int main(int argc,char**argv){
  using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;
  std::shared_ptr<Pipeline10> pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline10>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow10,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow10,1440,940);
    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10> iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator10 integ;bool ready=false,returning=false,done=false;int stable=0;double ref_roll=0,ref_pitch=0,ref_accum=0;size_t raw=0,sel=0,imu_rx=0,imu_fed=0,att_rx=0;
    std::cout<<"\nJT-ZERO IMU INTEGRATOR v10\nSPACE fixes zero, BA and BG. Test: YAW +80 then return to zero.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us){processOne(integ,im,interpR(prev_att,cur_att,im.us),*pipe);iq.pop_front();}else iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double rr,pp,yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double ty=returning?0:80,tol=returning?3:2;bool ok=std::abs(yy-ty)<=tol&&std::abs(pp)<=3&&std::abs(rr)<=3&&gn<=kGyroStill10&&std::abs(an-9.81)<=kAccTol10;if(ok)++stable;else stable=0;if(stable>=kStable10){stable=0;if(!returning){returning=true;std::cout<<"[STEP] YAW +80 fixed. Return to zero. FCint Vxy="<<integ.vfc.head<2>().norm()<<" GYROint Vxy="<<integ.vgyro.head<2>().norm()<<"\n";}else{done=true;std::cout<<"[TEST] completed at zero.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double gx = h.xgyro;
const double gy = -h.ygyro;
const double gz = -h.zgyro;

const double gx_corr = gx + 0.014570 * gz;
const double gy_corr = gy + 0.082383 * gz;

ImuSample10 im{
    h.time_usec,
    h.xacc,
    -h.yacc,
    -h.zacc,
    gx_corr,
    gy_corr,
    gz
};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>200)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){drawHud(last_gray,tel,ready,returning,done,stable,integ,ref_roll,ref_pitch,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend10 b;bool bv=pipe->latest(&b),fv=false;AttSample10 a;{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_roll=tel.fc_roll;ref_pitch=tel.fc_pitch;ref_accum=tel.fc_accum_yaw;}}if(bv&&fv&&have_att){integ.fixed_ba=Eigen::Vector3d(b.bax,b.bay,b.baz);integ.fixed_bg=Eigen::Vector3d(b.bgx,b.bgy,b.bgz);integ.Rgyro=fcRnedFlu(cur_att.roll,cur_att.pitch,cur_att.yaw);integ.vfc.setZero();integ.vgyro.setZero();integ.last_us=0;integ.armed=true;ready=true;std::cout<<"[ZERO] fixed BA ["<<b.bax<<", "<<b.bay<<", "<<b.baz<<"] BG ["<<b.bgx<<", "<<b.bgy<<", "<<b.bgz<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;saveCsv(integ);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO IMU INTEGRATOR v10 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<integ.rows.size()<<"\nmax FC-integrator Vxy: "<<integ.max_fc_vxy<<" m/s\nmax GYRO-integrator Vxy: "<<integ.max_gyro_vxy<<" m/s\nmax backend Vxy: "<<integ.max_backend_vxy<<" m/s\nmax PIM Vxy: "<<integ.max_pim_vxy<<" m/s\nCSV: "<<kCsv10<<"\nOpen CSV:\n  code "<<kCsv10<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
#endif // JTZERO_V10_NO_MAIN
