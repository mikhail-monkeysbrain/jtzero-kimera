// JT-ZERO YPR fixed-bias gravity diagnostic v9.
// Keeps the v8 camera/MAVLink/time-sync plumbing, but freezes accelerometer bias at ZERO.
// A step is accepted only when Y/P/R are on target, gyro is quiet and |acc|-g is small.

#define JTZERO_V8_NO_MAIN
#include "live_mono_imu_ypr_dashboard_v8.cpp"
#undef JTZERO_V8_NO_MAIN

namespace jtzero_v9 {

constexpr int kStableSamples9 = 20;
constexpr double kGyroStillMax = 0.08;       // rad/s
constexpr double kAccelNormTol = 0.35;       // m/s^2 around 9.81
constexpr const char* kCsv9 = "/home/vio/jtzero_live_ypr_dashboard_v9.csv";
constexpr const char* kWindow9 = "JT-ZERO: YPR GRAVITY v9";

struct State9 {
  int64_t wall_ns=0,timestamp_ns=0,keyframe=0;
  double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
  double roll=0,pitch=0,yaw=0;
  double bax=0,bay=0,baz=0;
  double fixed_bax=0,fixed_bay=0,fixed_baz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0,fc_accum_yaw=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  double acx=0,acy=0,acz=0,acc_norm=0,gyro_norm=0;
  double gvx=0,gvy=0,gvz=0,gfx=0,gfy=0,gfz=0;
  double res_vio=0,res_fc=0,angle_vio=0,angle_fc=0;
  double awx=0,awy=0,awz=0,awxy=0,pred_vxy=0;
  int step=0;
  bool fixed_bias_valid=false;
};

class Pipeline9 final : public VIO::MonoImuPipeline {
 public:
  Pipeline9(const VIO::VioParams& p,Telemetry* t):VIO::MonoImuPipeline(p),telemetry_(t){}

  void setStep(int s){step_.store(s);}

  void setFixedBias(double x,double y,double z){
    std::lock_guard<std::mutex> l(bias_mutex_);
    fixed_ba_=gtsam::Vector3(x,y,z);
    fixed_bias_valid_=true;
  }

  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      const auto& st=out->W_State_Blkf_;
      const auto p=st.pose_.translation();
      const auto& rr=st.pose_.rotation();
      const auto R=rr.matrix();
      const auto rpy=rr.rpy();
      const auto& v=st.velocity_;
      const auto ba=st.imu_bias_.accelerometer();

      State9 s;
      s.wall_ns=monotonicNs();s.timestamp_ns=st.timestamp_;s.keyframe=out->cur_kf_id_;
      s.px=p.x();s.py=p.y();s.pz=p.z();s.vx=v.x();s.vy=v.y();s.vz=v.z();
      s.roll=rpy.x()*180.0/kPi;s.pitch=rpy.y()*180.0/kPi;s.yaw=rpy.z()*180.0/kPi;
      s.bax=ba.x();s.bay=ba.y();s.baz=ba.z();s.step=step_.load();

      {
        std::lock_guard<std::mutex> l(telemetry_->mutex);
        s.fc_roll=telemetry_->fc_roll;s.fc_pitch=telemetry_->fc_pitch;
        s.fc_yaw=telemetry_->fc_yaw;s.fc_accum_yaw=telemetry_->fc_accum_yaw;
        s.ax=telemetry_->ax;s.ay=telemetry_->ay;s.az=telemetry_->az;
        s.gx=telemetry_->gx;s.gy=telemetry_->gy;s.gz=telemetry_->gz;
      }

      gtsam::Vector3 fba=ba;
      {
        std::lock_guard<std::mutex> l(bias_mutex_);
        if(fixed_bias_valid_){fba=fixed_ba_;s.fixed_bias_valid=true;}
      }
      s.fixed_bax=fba.x();s.fixed_bay=fba.y();s.fixed_baz=fba.z();

      const gtsam::Vector3 ac(s.ax-fba.x(),s.ay-fba.y(),s.az-fba.z());
      s.acx=ac.x();s.acy=ac.y();s.acz=ac.z();s.acc_norm=ac.norm();
      s.gyro_norm=std::sqrt(s.gx*s.gx+s.gy*s.gy+s.gz*s.gz);

      const gtsam::Vector3 gv=R.transpose()*gtsam::Vector3(0,0,9.81);
      const gtsam::Vector3 gf=fcSpecificForceFlu(s.fc_roll,s.fc_pitch,s.fc_yaw);
      s.gvx=gv.x();s.gvy=gv.y();s.gvz=gv.z();
      s.gfx=gf.x();s.gfy=gf.y();s.gfz=gf.z();
      s.res_vio=(ac-gv).norm();s.res_fc=(ac-gf).norm();

      auto angleDeg=[](const gtsam::Vector3& a,const gtsam::Vector3& b){
        const double den=a.norm()*b.norm();if(den<1e-9)return 180.0;
        const double c=std::max(-1.0,std::min(1.0,a.dot(b)/den));
        return std::acos(c)*180.0/kPi;
      };
      s.angle_vio=angleDeg(ac,gv);s.angle_fc=angleDeg(ac,gf);

      const gtsam::Vector3 aw=R*ac+gtsam::Vector3(0,0,-9.81);
      s.awx=aw.x();s.awy=aw.y();s.awz=aw.z();s.awxy=std::hypot(s.awx,s.awy);
      const auto& pv=out->debug_info_.navstate_k_.velocity();
      s.pred_vxy=std::hypot(pv.x(),pv.y());

      std::lock_guard<std::mutex> l(mutex_);
      latest_=s;have_latest_=true;states_.push_back(s);
    });
  }

  bool latest(State9* s)const{
    std::lock_guard<std::mutex> l(mutex_);if(!have_latest_)return false;*s=latest_;return true;
  }
  std::vector<State9> states()const{
    std::lock_guard<std::mutex> l(mutex_);return states_;
  }

 private:
  Telemetry* telemetry_;
  std::atomic<int> step_{0};
  mutable std::mutex mutex_,bias_mutex_;
  bool have_latest_=false,fixed_bias_valid_=false;
  State9 latest_;
  gtsam::Vector3 fixed_ba_=gtsam::Vector3::Zero();
  std::vector<State9> states_;
};

struct Ref9 {bool valid=false;double fc_roll=0,fc_pitch=0,fc_accum=0;double vio_roll=0,vio_pitch=0,vio_yaw=0;};
struct Att9 {double roll=0,pitch=0,yaw=0,vio_roll=0,vio_pitch=0,vio_yaw=0;};
struct Step9 {double yaw,pitch,roll,yt,pt,rt;const char* title;const char* instruction;};

const std::vector<Step9>& steps9(){
  static const std::vector<Step9> s={
    {80,0,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL 0°","Поверните YAW к +80°. PITCH и ROLL держите около нуля"},
    {80,20,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH +20°, ROLL 0°","Сохраняйте YAW +80° и установите PITCH +20°"},
    {80,-20,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH -20°, ROLL 0°","Сохраняйте YAW +80° и установите PITCH -20°"},
    {80,0,20,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL +20°","Сохраняйте YAW +80° и установите ROLL +20°"},
    {80,0,-20,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL -20°","Сохраняйте YAW +80° и установите ROLL -20°"},
    {0,0,0,kReturnTol,kReturnTol,kReturnTol,"ВОЗВРАТ: YAW 0°, PITCH 0°, ROLL 0°","Верните аппарат в исходное положение"}
  };return s;
}

Att9 relativeAtt9(const State9& s,const Ref9& r){
  Att9 a;if(!r.valid)return a;
  a.roll=wrapDeg(s.fc_roll-r.fc_roll);a.pitch=wrapDeg(s.fc_pitch-r.fc_pitch);a.yaw=s.fc_accum_yaw-r.fc_accum;
  a.vio_roll=wrapDeg(s.roll-r.vio_roll);a.vio_pitch=wrapDeg(s.pitch-r.vio_pitch);a.vio_yaw=wrapDeg(s.yaw-r.vio_yaw);
  return a;
}

bool targetOk(const Att9& a,const Step9& s){
  return std::abs(a.yaw-s.yaw)<=s.yt&&std::abs(a.pitch-s.pitch)<=s.pt&&std::abs(a.roll-s.roll)<=s.rt;
}
bool stillOk(const State9& s){return s.gyro_norm<=kGyroStillMax&&std::abs(s.acc_norm-9.81)<=kAccelNormTol;}

void renderHud9(const cv::Mat& gray,const Pipeline9& pipe,const Ref9& ref,int step,int stable,bool ready,bool done){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat canvas(940,1440,CV_8UC3,bg),video;
  if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(canvas(cv::Rect(20,80,820,615)));
  uiText(canvas,"JT-ZERO: ПРОВЕРКА ГРАВИТАЦИИ v9",20,48,.78,white,2);

  State9 s;const bool have=pipe.latest(&s);Att9 a;if(have)a=relativeAtt9(s,ref);
  bool t=false,q=false;if(ready&&!done&&have){t=targetOk(a,steps9()[step]);q=stillOk(s);}
  if(!ready){
    uiText(canvas,"УСТАНОВИТЕ АППАРАТ НЕПОДВИЖНО В ИСХОДНОЕ ПОЛОЖЕНИЕ",45,300,.65,white,2);
    uiText(canvas,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",150,365,.72,yellow,2);
  }else if(done){
    uiText(canvas,"ТЕСТ ЗАВЕРШЁН",250,305,1.10,green,3);uiText(canvas,"SPACE — ВЫХОД",305,365,.68,white,2);
  }else if(t&&!q){
    uiText(canvas,"СТОП. НЕ ДВИГАТЬ!",220,285,1.15,yellow,3);
    uiText(canvas,"Ждём малую угловую скорость и нормальный |ACC|",125,350,.55,white,2);
  }else if(t&&q){
    uiText(canvas,"СТОП! ПОЛОЖЕНИЕ СТАБИЛЬНО",155,285,1.05,green,3);
    uiText(canvas,"Удерживайте все три оси",250,350,.58,white,2);
  }else uiText(canvas,steps9()[step].instruction,70,320,.58,white,2);

  cv::Mat side=canvas(cv::Rect(860,70,560,850));cv::rectangle(side,{0,0},{559,849},cv::Scalar(45,45,45),1);
  if(!ready)uiText(side,"ПОДГОТОВКА",18,42,.70,yellow,2);else if(done)uiText(side,"ГОТОВО",18,42,.78,green,3);else{
    char b[128];std::snprintf(b,sizeof(b),"ШАГ %d/%zu",step+1,steps9().size());uiText(side,b,18,42,.68,green,2);
    uiText(side,steps9()[step].title,18,78,.43,white,2);
    std::snprintf(b,sizeof(b),"Стабильность: %d/%d",stable,kStableSamples9);uiText(side,b,18,108,.40,muted,1);
  }

  Step9 target{0,0,0,kReturnTol,kReturnTol,kReturnTol,"",""};if(ready&&!done)target=steps9()[step];
  drawBar(side,"YAW",a.yaw,-180,180,target.yaw,target.yt,180);
  drawBar(side,"PITCH",a.pitch,-45,45,target.pitch,target.pt,260);
  drawBar(side,"ROLL",a.roll,-45,45,target.roll,target.rt,340);

  if(have){
    char b[256];
    std::snprintf(b,sizeof(b),"GYRO |w| %.3f rad/s  лимит %.2f",s.gyro_norm,kGyroStillMax);uiText(side,b,18,435,.40,s.gyro_norm<=kGyroStillMax?green:red,1);
    std::snprintf(b,sizeof(b),"ACC |a| %.3f  |Δg| %.3f",s.acc_norm,std::abs(s.acc_norm-9.81));uiText(side,b,18,465,.40,std::abs(s.acc_norm-9.81)<=kAccelNormTol?green:red,1);
    std::snprintf(b,sizeof(b),"FIXED BA [%+.3f %+.3f %+.3f]",s.fixed_bax,s.fixed_bay,s.fixed_baz);uiText(side,b,18,500,.38,white,1);
    std::snprintf(b,sizeof(b),"CURRENT BA [%+.3f %+.3f %+.3f]",s.bax,s.bay,s.baz);uiText(side,b,18,530,.38,muted,1);
    std::snprintf(b,sizeof(b),"ACC FIXED [%+.2f %+.2f %+.2f]",s.acx,s.acy,s.acz);uiText(side,b,18,565,.38,white,1);
    std::snprintf(b,sizeof(b),"GRAV VIO  [%+.2f %+.2f %+.2f]",s.gvx,s.gvy,s.gvz);uiText(side,b,18,595,.38,white,1);
    std::snprintf(b,sizeof(b),"GRAV FC   [%+.2f %+.2f %+.2f]",s.gfx,s.gfy,s.gfz);uiText(side,b,18,625,.38,white,1);
    std::snprintf(b,sizeof(b),"Угол ACC↔VIO %.2f°  residual %.3f",s.angle_vio,s.res_vio);uiText(side,b,18,665,.40,s.angle_vio>3.0?red:white,1);
    std::snprintf(b,sizeof(b),"Угол ACC↔FC  %.2f°  residual %.3f",s.angle_fc,s.res_fc);uiText(side,b,18,700,.40,s.angle_fc>3.0?red:green,1);
    std::snprintf(b,sizeof(b),"WORLD XY %.3f  Vxy %.3f  PIM %.3f",s.awxy,std::hypot(s.vx,s.vy),s.pred_vxy);uiText(side,b,18,745,.38,white,1);
    uiText(side,t?"Y/P/R: В ЦЕЛИ":"Y/P/R: ВНЕ ЦЕЛИ",18,790,.42,t?green:red,2);
    uiText(side,q?"ПОКОЙ: ДА":"ПОКОЙ: НЕТ",270,790,.42,q?green:red,2);
  }
  uiText(canvas,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);
  uiText(canvas,std::string("CSV: ")+kCsv9,720,910,.40,muted,1);
  cv::imshow(kWindow9,canvas);
}

void saveCsv9(const std::vector<State9>& ss,const Ref9& ref){
  std::ofstream f(kCsv9,std::ios::trunc);
  f<<"step,target_yaw,target_pitch,target_roll,wall_ns,keyframe,timestamp_ns,fc_droll,fc_dpitch,fc_dyaw,vio_droll,vio_dpitch,vio_dyaw,ax,ay,az,gx,gy,gz,gyro_norm,fixed_bax,fixed_bay,fixed_baz,current_bax,current_bay,current_baz,acx,acy,acz,acc_norm,gvx,gvy,gvz,gfx,gfy,gfz,angle_vio_deg,angle_fc_deg,res_vio,res_fc,awx,awy,awz,awxy,vxy,pred_vxy,px,py,pz\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:ss){const Att9 a=relativeAtt9(s,ref);const int idx=std::max(0,std::min((int)steps9().size()-1,s.step));const auto&t=steps9()[idx];
    f<<s.step<<','<<t.yaw<<','<<t.pitch<<','<<t.roll<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','
     <<a.roll<<','<<a.pitch<<','<<a.yaw<<','<<a.vio_roll<<','<<a.vio_pitch<<','<<a.vio_yaw<<','
     <<s.ax<<','<<s.ay<<','<<s.az<<','<<s.gx<<','<<s.gy<<','<<s.gz<<','<<s.gyro_norm<<','
     <<s.fixed_bax<<','<<s.fixed_bay<<','<<s.fixed_baz<<','<<s.bax<<','<<s.bay<<','<<s.baz<<','
     <<s.acx<<','<<s.acy<<','<<s.acz<<','<<s.acc_norm<<','
     <<s.gvx<<','<<s.gvy<<','<<s.gvz<<','<<s.gfx<<','<<s.gfy<<','<<s.gfz<<','
     <<s.angle_vio<<','<<s.angle_fc<<','<<s.res_vio<<','<<s.res_fc<<','
     <<s.awx<<','<<s.awy<<','<<s.awz<<','<<s.awxy<<','<<std::hypot(s.vx,s.vy)<<','<<s.pred_vxy<<','<<s.px<<','<<s.py<<','<<s.pz<<'\n';
  }
}

} // namespace jtzero_v9

int main(int argc,char**argv){
  using namespace jtzero_v9;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;Telemetry telemetry;std::shared_ptr<Pipeline9> pipe;std::thread pipe_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<Pipeline9>(vp,&telemetry);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow9,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow9,1440,940);

    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs(),next_hud=monotonicNs();
    size_t raw=0,sel=0,imu_rx=0,imu_fed=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    Ref9 ref;bool ready=false,done=false;int step_idx=0,stable=0;pipe->setStep(0);
    std::cout<<"\nJT-ZERO YPR FIXED-BIAS GRAVITY DASHBOARD v9\nSPACE fixes zero and accelerometer bias. Step fixation also requires stillness.\n";

    while(true){
      const int64_t now=monotonicNs();if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample q;q.t0_rpi_ns=pending;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending;q.rpi_mid_ns=pending+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);const double y=a.yaw*180.0/kPi;{std::lock_guard<std::mutex>l(telemetry.mutex);if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;}
          if(ready&&!done){State9 st;if(pipe->latest(&st)){const Att9 rel=relativeAtt9(st,ref);if(targetOk(rel,steps9()[step_idx])&&stillOk(st))++stable;else stable=0;if(stable>=kStableSamples9){std::cout<<"[STEP] fixed "<<(step_idx+1)<<"/"<<steps9().size()<<": "<<steps9()[step_idx].title<<" | angle VIO="<<st.angle_vio<<" deg FC="<<st.angle_fc<<" deg | Vxy="<<std::hypot(st.vx,st.vy)<<" PIM="<<st.pred_vxy<<"\n";stable=0;++step_idx;if(step_idx>=(int)steps9().size()){done=true;pipe->setStep((int)steps9().size()-1);std::cout<<"[TEST] all steps completed\n";}else{pipe->setStep(step_idx);std::cout<<"[STEP] next: "<<steps9()[step_idx].title<<"\n";}}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;{std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;}if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs)continue;VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}
      }}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderHud9(last_gray,*pipe,ref,step_idx,stable,ready,done);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){State9 st;bool hv=pipe->latest(&st),fv=false;{std::lock_guard<std::mutex>l(telemetry.mutex);fv=telemetry.fc_valid;if(hv&&fv){ref.valid=true;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.fc_accum=telemetry.fc_accum_yaw;ref.vio_roll=st.roll;ref.vio_pitch=st.pitch;ref.vio_yaw=st.yaw;}}if(ref.valid){pipe->setFixedBias(st.bax,st.bay,st.baz);ready=true;stable=0;step_idx=0;pipe->setStep(0);std::cout<<"[ZERO] reference and fixed BA: ["<<st.bax<<", "<<st.bay<<", "<<st.baz<<"]\n[STEP] 1: "<<steps9()[0].title<<"\n";}}
      else if(key==' '&&done)break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipe->states();saveCsv9(states,ref);
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO YPR FIXED-BIAS GRAVITY DASHBOARD v9 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kCsv9<<"\nOpen CSV:\n  code "<<kCsv9<<"\n";
    return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
