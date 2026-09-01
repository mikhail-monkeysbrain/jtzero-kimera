// JT-ZERO coordinated YAW/PITCH/ROLL gravity consistency dashboard v8.
// Event-driven manual test: no timers. All three axes must be inside target zones.
// Adds independent accelerometer-vs-gravity residuals from VIO R_WB and FC ATTITUDE.

#define main jtzero_hud_base_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include <opencv2/freetype.hpp>

namespace {

constexpr int kAttRateHz = 50;
constexpr int kStableSamples = 15;
constexpr double kYawTol = 2.0;
constexpr double kPitchTol = 2.5;
constexpr double kRollTol = 2.5;
constexpr double kReturnTol = 3.0;
constexpr const char* kCsv = "/home/vio/jtzero_live_ypr_dashboard_v8.csv";
constexpr const char* kWindow = "JT-ZERO: YAW PITCH ROLL v8";
constexpr const char* kFont = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";

cv::Ptr<cv::freetype::FreeType2> uiFont() {
  static cv::Ptr<cv::freetype::FreeType2> ft = [] {
    auto p = cv::freetype::createFreeType2();
    p->loadFontData(kFont, 0);
    return p;
  }();
  return ft;
}

void uiText(cv::Mat& img,const std::string& s,int x,int y,double scale=.55,
            cv::Scalar color={235,235,235},int th=1) {
  const int h=std::max(12,(int)(32.0*scale));
  uiFont()->putText(img,s,{x,y},h,color,th,cv::LINE_AA,true);
}

struct Telemetry {
  mutable std::mutex mutex;
  bool fc_valid=false, have_prev_yaw=false;
  double fc_roll=0,fc_pitch=0,fc_yaw=0,fc_accum_yaw=0,prev_yaw=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
};

struct State {
  int64_t wall_ns=0,timestamp_ns=0,keyframe=0;
  double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
  double roll=0,pitch=0,yaw=0;
  double bax=0,bay=0,baz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0,fc_accum_yaw=0;
  double ax=0,ay=0,az=0;
  double acx=0,acy=0,acz=0;
  double awx=0,awy=0,awz=0,awxy=0,pred_vxy=0;
  double gvx=0,gvy=0,gvz=0;
  double gfx=0,gfy=0,gfz=0;
  double res_vio=0,res_fc=0,res_vio_xy=0,res_fc_xy=0;
  int step=0;
};

// Expected stationary specific force in FLU from MAVLink ATTITUDE.
// MAVLink ATTITUDE is NED/FRD. Compute -R_NB^T*g_NED in FRD, then FRD->FLU [x,-y,-z].
gtsam::Vector3 fcSpecificForceFlu(double roll_deg,double pitch_deg,double yaw_deg) {
  const double r=roll_deg*kPi/180.0,p=pitch_deg*kPi/180.0,y=yaw_deg*kPi/180.0;
  const gtsam::Rot3 R_NB=gtsam::Rot3::RzRyRx(r,p,y);
  const gtsam::Vector3 g_ned(0,0,9.81);
  const gtsam::Vector3 f_frd=-(R_NB.matrix().transpose()*g_ned);
  return gtsam::Vector3(f_frd.x(),-f_frd.y(),-f_frd.z());
}

class Pipeline final : public VIO::MonoImuPipeline {
 public:
  Pipeline(const VIO::VioParams& p,Telemetry* t):VIO::MonoImuPipeline(p),telemetry_(t){}
  void setStep(int s){step_.store(s);}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      const auto& st=out->W_State_Blkf_;
      const auto p=st.pose_.translation();
      const auto& rr=st.pose_.rotation(); const auto R=rr.matrix(); const auto rpy=rr.rpy();
      const auto& v=st.velocity_; const auto ba=st.imu_bias_.accelerometer();
      State s; s.wall_ns=monotonicNs(); s.timestamp_ns=st.timestamp_; s.keyframe=out->cur_kf_id_;
      s.px=p.x();s.py=p.y();s.pz=p.z();s.vx=v.x();s.vy=v.y();s.vz=v.z();
      s.roll=rpy.x()*180.0/kPi;s.pitch=rpy.y()*180.0/kPi;s.yaw=rpy.z()*180.0/kPi;
      s.bax=ba.x();s.bay=ba.y();s.baz=ba.z();s.step=step_.load();
      {std::lock_guard<std::mutex> l(telemetry_->mutex);
        s.fc_roll=telemetry_->fc_roll;s.fc_pitch=telemetry_->fc_pitch;s.fc_yaw=telemetry_->fc_yaw;s.fc_accum_yaw=telemetry_->fc_accum_yaw;
        s.ax=telemetry_->ax;s.ay=telemetry_->ay;s.az=telemetry_->az;}

      const gtsam::Vector3 ac(s.ax-s.bax,s.ay-s.bay,s.az-s.baz);
      s.acx=ac.x();s.acy=ac.y();s.acz=ac.z();
      const gtsam::Vector3 aw=R*ac+gtsam::Vector3(0,0,-9.81);
      s.awx=aw.x();s.awy=aw.y();s.awz=aw.z();s.awxy=std::hypot(s.awx,s.awy);

      // VIO prediction of stationary specific force in body FLU: -R_WB^T*g_W.
      const gtsam::Vector3 gv=R.transpose()*gtsam::Vector3(0,0,9.81);
      s.gvx=gv.x();s.gvy=gv.y();s.gvz=gv.z();
      const gtsam::Vector3 gf=fcSpecificForceFlu(s.fc_roll,s.fc_pitch,s.fc_yaw);
      s.gfx=gf.x();s.gfy=gf.y();s.gfz=gf.z();
      const gtsam::Vector3 ev=ac-gv, ef=ac-gf;
      s.res_vio=ev.norm();s.res_fc=ef.norm();
      s.res_vio_xy=std::hypot(ev.x(),ev.y());s.res_fc_xy=std::hypot(ef.x(),ef.y());

      const auto& pv=out->debug_info_.navstate_k_.velocity();s.pred_vxy=std::hypot(pv.x(),pv.y());
      {std::lock_guard<std::mutex> l(mutex_);latest_=s;have_latest_=true;states_.push_back(s);}
    });
  }
  bool latest(State* s)const{std::lock_guard<std::mutex> l(mutex_);if(!have_latest_)return false;*s=latest_;return true;}
  std::vector<State> states()const{std::lock_guard<std::mutex> l(mutex_);return states_;}
 private:
  Telemetry* telemetry_; std::atomic<int> step_{0}; mutable std::mutex mutex_; bool have_latest_=false; State latest_; std::vector<State> states_;
};

struct Ref {bool valid=false;double fc_roll=0,fc_pitch=0,fc_accum=0;State vio;};
struct Att {double roll=0,pitch=0,yaw=0,vio_roll=0,vio_pitch=0,vio_yaw=0;};

Att relativeAtt(const State& s,const Ref& r){
  Att a;if(!r.valid)return a;
  a.roll=wrapDeg(s.fc_roll-r.fc_roll);a.pitch=wrapDeg(s.fc_pitch-r.fc_pitch);a.yaw=s.fc_accum_yaw-r.fc_accum;
  a.vio_roll=wrapDeg(s.roll-r.vio.roll);a.vio_pitch=wrapDeg(s.pitch-r.vio.pitch);a.vio_yaw=wrapDeg(s.yaw-r.vio.yaw);return a;
}

struct Step {double yaw,pitch,roll;double yt,pt,rt;const char* title;const char* instruction;};
const std::vector<Step>& steps(){
  static const std::vector<Step> s={
    {80,0,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL 0°","Поверните YAW к +80°. Удерживайте PITCH и ROLL около нуля"},
    {80,20,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH +20°, ROLL 0°","Сохраняйте YAW +80° и наклоните PITCH к +20°"},
    {80,-20,0,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH -20°, ROLL 0°","Сохраняйте YAW +80° и наклоните PITCH к -20°"},
    {80,0,20,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL +20°","Сохраняйте YAW +80° и наклоните ROLL к +20°"},
    {80,0,-20,kYawTol,kPitchTol,kRollTol,"YAW +80°, PITCH 0°, ROLL -20°","Сохраняйте YAW +80° и наклоните ROLL к -20°"},
    {0,0,0,kReturnTol,kReturnTol,kReturnTol,"ВОЗВРАТ: YAW 0°, PITCH 0°, ROLL 0°","Верните все три оси в исходное положение"}
  };return s;
}

bool inTarget(const Att& a,const Step& s){
  return std::abs(a.yaw-s.yaw)<=s.yt && std::abs(a.pitch-s.pitch)<=s.pt && std::abs(a.roll-s.roll)<=s.rt;
}

void drawBar(cv::Mat& p,const std::string& name,double v,double minv,double maxv,double target,double tol,int y){
  const cv::Scalar white(235,235,235),green(80,220,80),red(60,60,170),yellow(0,220,255);
  const int bx=120,bw=350,bh=32;
  uiText(p,name,18,y+24,.50,white,2);
  auto xf=[&](double x){double t=(x-minv)/(maxv-minv);t=std::max(0.0,std::min(1.0,t));return bx+(int)std::lround(t*bw);};
  cv::rectangle(p,{bx,y},{bx+bw,y+bh},red,cv::FILLED);
  cv::rectangle(p,{xf(target-tol),y},{xf(target+tol),y+bh},green,cv::FILLED);
  cv::rectangle(p,{bx,y},{bx+bw,y+bh},white,1);
  cv::line(p,{xf(target),y-5},{xf(target),y+bh+5},white,2);
  cv::line(p,{xf(v),y-7},{xf(v),y+bh+7},yellow,4);
  char b[64];std::snprintf(b,sizeof(b),"%+.1f°",v);uiText(p,b,482,y+24,.50,white,2);
}

void renderHud(const cv::Mat& gray,const Pipeline& pipe,const Ref& ref,int step,int stable,bool ready,bool done){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat canvas(940,1440,CV_8UC3,bg),video;
  if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0,0,0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(canvas(cv::Rect(20,80,820,615)));
  uiText(canvas,"JT-ZERO YAW PITCH ROLL TEST v8",20,48,.78,white,2);
  State s;bool have=pipe.latest(&s);Att a;if(have)a=relativeAtt(s,ref);
  if(!ready){uiText(canvas,"УСТАНОВИТЕ АППАРАТ В ИСХОДНОЕ ПОЛОЖЕНИЕ",80,305,.72,white,2);uiText(canvas,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",205,360,.72,yellow,2);}
  else if(done){uiText(canvas,"ТЕСТ ЗАВЕРШЁН",250,305,1.10,green,3);uiText(canvas,"НАЖМИТЕ SPACE ДЛЯ ВЫХОДА",190,365,.62,white,2);}
  else {const auto& st=steps()[step];if(inTarget(a,st)){uiText(canvas,"СТОП!",300,285,1.55,green,4);uiText(canvas,"УДЕРЖИВАЙТЕ ВСЕ ТРИ ПОЛОСЫ В ЗЕЛЁНОЙ ЗОНЕ",95,350,.60,white,2);}else uiText(canvas,st.instruction,75,320,.58,white,2);}

  cv::Mat side=canvas(cv::Rect(860,70,560,850));cv::rectangle(side,{0,0},{559,849},cv::Scalar(45,45,45),1);
  uiText(side,"ИНСТРУКЦИЯ",18,34,.56,white,2);
  if(!ready)uiText(side,"ПОДГОТОВКА",18,80,.72,yellow,2);
  else if(done)uiText(side,"ГОТОВО",18,80,.82,green,3);
  else {char b[256];std::snprintf(b,sizeof(b),"ШАГ %d/%zu",step+1,steps().size());uiText(side,b,18,80,.68,green,2);uiText(side,steps()[step].title,18,115,.48,white,2);std::snprintf(b,sizeof(b),"Фиксация: %d/%d",stable,kStableSamples);uiText(side,b,18,145,.44,muted,1);}
  for(size_t i=0;i<steps().size();++i){bool pass=ready&&(done||(int)i<step),cur=ready&&!done&&(int)i==step;std::string line=(pass?"✓ ":(cur?"→ ":"  "))+std::to_string(i+1)+". "+steps()[i].title;uiText(side,line,18,188+(int)i*28,.36,pass?green:(cur?yellow:muted),pass||cur?2:1);}

  Step target{0,0,0,kReturnTol,kReturnTol,kReturnTol,"",""};if(ready&&!done)target=steps()[step];
  drawBar(side,"YAW",a.yaw,-180,180,target.yaw,target.yt,400);
  drawBar(side,"PITCH",a.pitch,-45,45,target.pitch,target.pt,480);
  drawBar(side,"ROLL",a.roll,-45,45,target.roll,target.rt,560);
  if(have){
    char b[256];
    std::snprintf(b,sizeof(b),"ACC_B      [%+.2f %+.2f %+.2f]",s.acx,s.acy,s.acz);uiText(side,b,18,642,.36,white,1);
    std::snprintf(b,sizeof(b),"GRAV VIO   [%+.2f %+.2f %+.2f]",s.gvx,s.gvy,s.gvz);uiText(side,b,18,670,.36,white,1);
    std::snprintf(b,sizeof(b),"GRAV FC    [%+.2f %+.2f %+.2f]",s.gfx,s.gfy,s.gfz);uiText(side,b,18,698,.36,white,1);
    std::snprintf(b,sizeof(b),"|ACC-G VIO| %.3f   XY %.3f",s.res_vio,s.res_vio_xy);uiText(side,b,18,730,.40,s.res_vio_xy>.5?red:white,1);
    std::snprintf(b,sizeof(b),"|ACC-G FC | %.3f   XY %.3f",s.res_fc,s.res_fc_xy);uiText(side,b,18,760,.40,s.res_fc_xy>.5?red:green,1);
    std::snprintf(b,sizeof(b),"WORLD XY %.3f   Vxy %.3f   PIM %.3f",s.awxy,std::hypot(s.vx,s.vy),s.pred_vxy);uiText(side,b,18,792,.37,s.awxy>.5?red:white,1);
  }
  uiText(side,"Все три полосы должны быть зелёными",18,828,.38,yellow,1);
  uiText(canvas,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(canvas,std::string("CSV: ")+kCsv,720,910,.40,muted,1);
  cv::imshow(kWindow,canvas);
}

void saveCsv(const std::vector<State>& ss,const Ref& ref){
  std::ofstream f(kCsv,std::ios::trunc);
  f<<"step,target_yaw,target_pitch,target_roll,tol_yaw,tol_pitch,tol_roll,wall_ns,keyframe,timestamp_ns,fc_droll,fc_dpitch,fc_dyaw,vio_droll,vio_dpitch,vio_dyaw,err_yaw,err_pitch,err_roll,ax,ay,az,acx,acy,acz,gvx,gvy,gvz,gfx,gfy,gfz,res_vio,res_fc,res_vio_xy,res_fc_xy,awx,awy,awz,awxy,vxy,pred_vxy,bax,bay,baz,px,py,pz\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:ss){Att a=relativeAtt(s,ref);int idx=std::max(0,std::min((int)steps().size()-1,s.step));const auto&t=steps()[idx];
    f<<s.step<<','<<t.yaw<<','<<t.pitch<<','<<t.roll<<','<<t.yt<<','<<t.pt<<','<<t.rt<<','<<s.wall_ns<<','<<s.keyframe<<','<<s.timestamp_ns<<','
     <<a.roll<<','<<a.pitch<<','<<a.yaw<<','<<a.vio_roll<<','<<a.vio_pitch<<','<<a.vio_yaw<<','<<(a.yaw-t.yaw)<<','<<(a.pitch-t.pitch)<<','<<(a.roll-t.roll)<<','
     <<s.ax<<','<<s.ay<<','<<s.az<<','<<s.acx<<','<<s.acy<<','<<s.acz<<','<<s.gvx<<','<<s.gvy<<','<<s.gvz<<','<<s.gfx<<','<<s.gfy<<','<<s.gfz<<','
     <<s.res_vio<<','<<s.res_fc<<','<<s.res_vio_xy<<','<<s.res_fc_xy<<','<<s.awx<<','<<s.awy<<','<<s.awz<<','<<s.awxy<<','<<std::hypot(s.vx,s.vy)<<','<<s.pred_vxy<<','
     <<s.bax<<','<<s.bay<<','<<s.baz<<','<<s.px<<','<<s.py<<','<<s.pz<<'\n';}
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;Telemetry telemetry;std::shared_ptr<Pipeline> pipe;std::thread pipe_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipe=std::make_shared<Pipeline>(vp,&telemetry);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t hb_deadline=monotonicNs()+10000000000LL;while(monotonicNs()<hb_deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow,1440,940);
    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs(),next_hud=monotonicNs();size_t raw=0,sel=0,imu_rx=0,imu_fed=0,att_rx=0;uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    Ref ref;bool ready=false,done=false;int step_idx=0,stable=0;pipe->setStep(0);
    std::cout<<"\nJT-ZERO YPR GRAVITY DASHBOARD v8\nNo timers. SPACE fixes zero. Y/P/R targets plus VIO-vs-FC gravity consistency.\n";
    while(true){
      const int64_t now=monotonicNs();if(now>=next_sync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample q;q.t0_rpi_ns=pending;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending;q.rpi_mid_ns=pending+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){++att_rx;mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);const double y=a.yaw*180.0/kPi;{std::lock_guard<std::mutex>l(telemetry.mutex);if(telemetry.have_prev_yaw)telemetry.fc_accum_yaw+=wrapDeg(y-telemetry.prev_yaw);telemetry.prev_yaw=y;telemetry.have_prev_yaw=true;telemetry.fc_roll=a.roll*180.0/kPi;telemetry.fc_pitch=a.pitch*180.0/kPi;telemetry.fc_yaw=y;telemetry.fc_valid=true;}
          if(ready&&!done){State st;if(pipe->latest(&st)){Att rel=relativeAtt(st,ref);if(inTarget(rel,steps()[step_idx]))++stable;else stable=0;if(stable>=kStableSamples){std::cout<<"[STEP] fixed "<<(step_idx+1)<<"/"<<steps().size()<<": "<<steps()[step_idx].title<<"\n";stable=0;++step_idx;if(step_idx>=(int)steps().size()){done=true;pipe->setStep((int)steps().size()-1);std::cout<<"[TEST] all steps completed\n";}else{pipe->setStep(step_idx);std::cout<<"[STEP] next: "<<steps()[step_idx].title<<"\n";}}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);const double ax=h.xacc,ay=-h.yacc,az=-h.zacc,gx=h.xgyro,gy=-h.ygyro,gz=-h.zgyro;{std::lock_guard<std::mutex>l(telemetry.mutex);telemetry.ax=ax;telemetry.ay=ay;telemetry.az=az;telemetry.gx=gx;telemetry.gy=gy;telemetry.gz=gz;}if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs)continue;VIO::ImuAccGyr d;d<<ax,ay,az,gx,gy,gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++imu_fed;}
      }}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;const bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;++sel;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){renderHud(last_gray,*pipe,ref,step_idx,stable,ready,done);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){State st;bool hv=pipe->latest(&st),fv=false;{std::lock_guard<std::mutex>l(telemetry.mutex);fv=telemetry.fc_valid;if(hv&&fv){ref.valid=true;ref.fc_roll=telemetry.fc_roll;ref.fc_pitch=telemetry.fc_pitch;ref.fc_accum=telemetry.fc_accum_yaw;ref.vio=st;}}if(ref.valid){ready=true;stable=0;step_idx=0;pipe->setStep(0);std::cout<<"[ZERO] reference fixed. Step 1: "<<steps()[0].title<<"\n";}}
      else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipe->states();saveCsv(states,ref);
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO YPR GRAVITY DASHBOARD v8 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nraw camera frames: "<<raw<<"\nselected frames: "<<sel<<"\nIMU received: "<<imu_rx<<"\nIMU fed: "<<imu_fed<<"\nATTITUDE received: "<<att_rx<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kCsv<<"\n";
    std::cout<<"Open CSV:\n  code \""<<kCsv<<"\"\n";
    return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
