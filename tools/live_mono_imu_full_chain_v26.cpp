// JT-ZERO v26: один стендовый прогон для диагностики всей текущей IMU->PIM цепочки.
// Маршрут: YAW 0 -> +30 -> 0 -> -30 -> 0.
// Одновременно проверяет timing, raw IMU, BG/BA, FC ATTITUDE, gyro integration,
// accel/gravity residual, PIM PRED, OPT и чувствительность к выбору gyro bias.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

namespace jtzero_v26 {
using namespace jtzero_v10;

constexpr const char* kCsv26 = "/home/vio/jtzero_live_full_chain_v26.csv";
constexpr const char* kWindow26 = "JT-ZERO: ПОЛНАЯ IMU ЦЕПОЧКА v26";
constexpr double kTargets26[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames26[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable26 = 20;
constexpr double kYawTol26 = 2.0;
constexpr double kGyroStill26 = 0.08;
constexpr double kAccTol26 = 0.35;

static double clamp26(double x){return std::max(-1.0,std::min(1.0,x));}
static double rotErr26(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){
  const Eigen::Matrix3d D=A.transpose()*B;
  return std::acos(clamp26((D.trace()-1.0)*0.5))*180.0/kPi;
}
static double gravErr26(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){
  const Eigen::Vector3d z(0,0,1);
  return std::acos(clamp26((A.transpose()*z).normalized().dot((B.transpose()*z).normalized())))*180.0/kPi;
}
static Eigen::Matrix3d stepR26(const Eigen::Matrix3d&R,const Eigen::Vector3d&w,double dt){
  if(dt<=0)return R; const Eigen::Vector3d th=w*dt; const double a=th.norm();
  if(a<=1e-12)return R; return R*Eigen::AngleAxisd(a,th/a).toRotationMatrix();
}

struct Backend26{
  bool valid=false; int64_t kf=0,ts=0;
  Eigen::Matrix3d Rpred=Eigen::Matrix3d::Identity(), Ropt=Eigen::Matrix3d::Identity();
  Eigen::Vector3d Vpred=Eigen::Vector3d::Zero(), Vopt=Eigen::Vector3d::Zero();
  Eigen::Vector3d BA=Eigen::Vector3d::Zero(), BG=Eigen::Vector3d::Zero();
};

class Pipeline26 final: public VIO::MonoImuPipeline{
 public:
  explicit Pipeline26(const VIO::VioParams&p):VIO::MonoImuPipeline(p){}
  void install(){registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>&o){
    if(!o)return; Backend26 b; const auto&st=o->W_State_Blkf_;
    b.valid=true;b.kf=o->cur_kf_id_;b.ts=st.timestamp_;
    b.Rpred=o->debug_info_.navstate_k_.pose().rotation().matrix();
    b.Vpred=o->debug_info_.navstate_k_.velocity();
    b.Ropt=st.pose_.rotation().matrix();b.Vopt=st.velocity_;
    b.BA=st.imu_bias_.accelerometer();b.BG=st.imu_bias_.gyroscope();
    std::lock_guard<std::mutex>q(m_);last_=b;
  });}
  bool latest(Backend26*b)const{std::lock_guard<std::mutex>q(m_);if(!last_.valid)return false;*b=last_;return true;}
 private: mutable std::mutex m_; Backend26 last_;
};

struct Row26{
  int phase=0; uint64_t imu_us=0; double dt=0; int64_t kf=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
  double ba_x=0,ba_y=0,ba_z=0,bg_x=0,bg_y=0,bg_z=0;
  double bg0_x=0,bg0_y=0,bg0_z=0;
  double grav_be_x=0,grav_be_y=0,grav_be_z=0;
  double afc_x=0,afc_y=0,afc_z=0;
  double abg_x=0,abg_y=0,abg_z=0;
  double ast_x=0,ast_y=0,ast_z=0;
  double vfc_x=0,vfc_y=0,vfc_z=0;
  double vbg_x=0,vbg_y=0,vbg_z=0;
  double vst_x=0,vst_y=0,vst_z=0;
  double pim_x=0,pim_y=0,pim_z=0,opt_x=0,opt_y=0,opt_z=0;
  double r_bg_fc=0,g_bg_fc=0,r_st_fc=0,g_st_fc=0;
  double pred_opt_r=0,pred_opt_g=0,pred_opt_dv=0;
};

struct State26{
  bool armed=false; uint64_t last_us=0; int phase=0;
  Eigen::Vector3d bg_static=Eigen::Vector3d::Zero(), ba0=Eigen::Vector3d::Zero(), bg_backend0=Eigen::Vector3d::Zero();
  Eigen::Vector3d pim_v0=Eigen::Vector3d::Zero(), opt_v0=Eigen::Vector3d::Zero();
  Eigen::Vector3d vfc=Eigen::Vector3d::Zero(),vbg=Eigen::Vector3d::Zero(),vst=Eigen::Vector3d::Zero();
  Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity(),Rbg=Eigen::Matrix3d::Identity(),Rst=Eigen::Matrix3d::Identity();
  std::vector<Row26> rows;
  uint64_t imu_count=0,imu_gap_count=0; double imu_dt_sum=0,imu_dt_min=1e9,imu_dt_max=0;
};

static void process26(State26&s,const ImuSample10&im,const Eigen::Matrix3d&RfcN,const Pipeline26&pipe){
  if(!s.armed)return;
  double dt=0; if(s.last_us&&im.us>s.last_us)dt=(im.us-s.last_us)*1e-6; s.last_us=im.us;
  if(dt<=0||dt>0.03){if(dt>0.03)++s.imu_gap_count;dt=0;} else {++s.imu_count;s.imu_dt_sum+=dt;s.imu_dt_min=std::min(s.imu_dt_min,dt);s.imu_dt_max=std::max(s.imu_dt_max,dt);}
  Backend26 b; const bool bv=pipe.latest(&b);
  const Eigen::Vector3d ba=bv?b.BA:s.ba0, bg=bv?b.BG:s.bg_backend0;
  const Eigen::Vector3d acc(im.ax,im.ay,im.az), gyr(im.gx,im.gy,im.gz);
  const Eigen::Matrix3d Rfc=s.world_map*RfcN;
  if(dt>0){s.Rbg=stepR26(s.Rbg,gyr-bg,dt);s.Rst=stepR26(s.Rst,gyr-s.bg_static,dt);}
  const Eigen::Vector3d gW(0,0,9.81);
  const Eigen::Vector3d ac=acc-ba;
  const Eigen::Vector3d afc=Rfc*ac+gW, abg=s.Rbg*ac+gW, ast=s.Rst*ac+gW;
  if(dt>0){s.vfc+=afc*dt;s.vbg+=abg*dt;s.vst+=ast*dt;}
  const Eigen::Vector3d fexp=Rfc.transpose()*Eigen::Vector3d(0,0,-9.81);
  const Eigen::Vector3d gres=ac-fexp;
  Eigen::Vector3d pim = Eigen::Vector3d::Zero();
  Eigen::Vector3d opt = Eigen::Vector3d::Zero();
  if (bv) {
    pim = b.Vpred - s.pim_v0;
    opt = b.Vopt - s.opt_v0;
  }
  Row26 r;r.phase=s.phase;r.imu_us=im.us;r.dt=dt;r.kf=bv?b.kf:0;
  r.ax=im.ax;r.ay=im.ay;r.az=im.az;r.gx=im.gx;r.gy=im.gy;r.gz=im.gz;
  Eigen::Vector3d rr=RfcN.eulerAngles(0,1,2)*180.0/kPi;r.fc_roll=rr.x();r.fc_pitch=rr.y();r.fc_yaw=rr.z();
  r.ba_x=ba.x();r.ba_y=ba.y();r.ba_z=ba.z();r.bg_x=bg.x();r.bg_y=bg.y();r.bg_z=bg.z();
  r.bg0_x=s.bg_static.x();r.bg0_y=s.bg_static.y();r.bg0_z=s.bg_static.z();
  r.grav_be_x=gres.x();r.grav_be_y=gres.y();r.grav_be_z=gres.z();
  r.afc_x=afc.x();r.afc_y=afc.y();r.afc_z=afc.z();r.abg_x=abg.x();r.abg_y=abg.y();r.abg_z=abg.z();r.ast_x=ast.x();r.ast_y=ast.y();r.ast_z=ast.z();
  r.vfc_x=s.vfc.x();r.vfc_y=s.vfc.y();r.vfc_z=s.vfc.z();r.vbg_x=s.vbg.x();r.vbg_y=s.vbg.y();r.vbg_z=s.vbg.z();r.vst_x=s.vst.x();r.vst_y=s.vst.y();r.vst_z=s.vst.z();
  r.pim_x=pim.x();r.pim_y=pim.y();r.pim_z=pim.z();r.opt_x=opt.x();r.opt_y=opt.y();r.opt_z=opt.z();
  if(bv){r.r_bg_fc=rotErr26(s.Rbg,Rfc);r.g_bg_fc=gravErr26(s.Rbg,Rfc);r.r_st_fc=rotErr26(s.Rst,Rfc);r.g_st_fc=gravErr26(s.Rst,Rfc);r.pred_opt_r=rotErr26(b.Rpred,b.Ropt);r.pred_opt_g=gravErr26(b.Rpred,b.Ropt);r.pred_opt_dv=(b.Vopt-b.Vpred).norm();}
  s.rows.push_back(r);
}

static void save26(const State26&s){
  std::ofstream f(kCsv26,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,imu_us,dt,kf,ax,ay,az,gx,gy,gz,fc_roll,fc_pitch,fc_yaw,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,bg_static_x,bg_static_y,bg_static_z,grav_body_res_x,grav_body_res_y,grav_body_res_z,a_fc_x,a_fc_y,a_fc_z,a_bg_x,a_bg_y,a_bg_z,a_staticbg_x,a_staticbg_y,a_staticbg_z,v_fc_x,v_fc_y,v_fc_z,v_bg_x,v_bg_y,v_bg_z,v_staticbg_x,v_staticbg_y,v_staticbg_z,pim_dv_x,pim_dv_y,pim_dv_z,opt_dv_x,opt_dv_y,opt_dv_z,R_bg_vs_fc_deg,G_bg_vs_fc_deg,R_staticbg_vs_fc_deg,G_staticbg_vs_fc_deg,pred_opt_R_deg,pred_opt_G_deg,pred_opt_dV\n";
  for(const auto&r:s.rows)f<<r.phase<<','<<kNames26[r.phase]<<','<<r.imu_us<<','<<r.dt<<','<<r.kf<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<','<<r.ba_x<<','<<r.ba_y<<','<<r.ba_z<<','<<r.bg_x<<','<<r.bg_y<<','<<r.bg_z<<','<<r.bg0_x<<','<<r.bg0_y<<','<<r.bg0_z<<','<<r.grav_be_x<<','<<r.grav_be_y<<','<<r.grav_be_z<<','<<r.afc_x<<','<<r.afc_y<<','<<r.afc_z<<','<<r.abg_x<<','<<r.abg_y<<','<<r.abg_z<<','<<r.ast_x<<','<<r.ast_y<<','<<r.ast_z<<','<<r.vfc_x<<','<<r.vfc_y<<','<<r.vfc_z<<','<<r.vbg_x<<','<<r.vbg_y<<','<<r.vbg_z<<','<<r.vst_x<<','<<r.vst_y<<','<<r.vst_z<<','<<r.pim_x<<','<<r.pim_y<<','<<r.pim_z<<','<<r.opt_x<<','<<r.opt_y<<','<<r.opt_z<<','<<r.r_bg_fc<<','<<r.g_bg_fc<<','<<r.r_st_fc<<','<<r.g_st_fc<<','<<r.pred_opt_r<<','<<r.pred_opt_g<<','<<r.pred_opt_dv<<'\n';
}

static void summary26(const State26&s){
  std::cout<<"timing: imu_count="<<s.imu_count<<" gaps="<<s.imu_gap_count;
  if(s.imu_count)std::cout<<" dt_mean="<<(s.imu_dt_sum/s.imu_count)*1000.0<<"ms dt_min="<<s.imu_dt_min*1000.0<<"ms dt_max="<<s.imu_dt_max*1000.0<<"ms";std::cout<<"\n";
  std::cout<<"BG static=["<<s.bg_static.transpose()<<"] backend0=["<<s.bg_backend0.transpose()<<"] diff=["<<(s.bg_static-s.bg_backend0).transpose()<<"]\n";
  for(int p=0;p<4;++p){bool first=true;Row26 a,z;double maxGbg=0,maxGst=0,maxPO=0,maxRes=0,maxPimFc=0,maxPimBg=0;size_t n=0;double ssRes=0;
    for(const auto&r:s.rows)if(r.phase==p){if(first){a=r;first=false;}z=r;maxGbg=std::max(maxGbg,r.g_bg_fc);maxGst=std::max(maxGst,r.g_st_fc);maxPO=std::max(maxPO,r.pred_opt_dv);double q=std::hypot(r.grav_be_x,r.grav_be_y);maxRes=std::max(maxRes,q);ssRes+=q*q;++n;maxPimFc=std::max(maxPimFc,std::hypot(r.pim_x-r.vfc_x,r.pim_y-r.vfc_y));maxPimBg=std::max(maxPimBg,std::hypot(r.pim_x-r.vbg_x,r.pim_y-r.vbg_y));}
    if(first)continue;
    std::cout<<kNames26[p]<<": dV FC=["<<(z.vfc_x-a.vfc_x)<<","<<(z.vfc_y-a.vfc_y)<<"] BG=["<<(z.vbg_x-a.vbg_x)<<","<<(z.vbg_y-a.vbg_y)<<"] staticBG=["<<(z.vst_x-a.vst_x)<<","<<(z.vst_y-a.vst_y)<<"] PIM=["<<(z.pim_x-a.pim_x)<<","<<(z.pim_y-a.pim_y)<<"]"
             <<" max|PIM-FC|="<<maxPimFc<<" max|PIM-BG|="<<maxPimBg<<" maxG(BG/FC)="<<maxGbg<<"deg maxG(static/FC)="<<maxGst<<"deg"
             <<" accelBodyXY rms="<<std::sqrt(ssRes/std::max<size_t>(1,n))<<" max="<<maxRes<<" m/s2 maxPredOptDV="<<maxPO<<"\n";
  }
}

static void hud26(const cv::Mat&gray,const Telemetry&tel,bool ready,bool done,int phase,int stable,const State26&s,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),v;if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);cv::resize(v,v,{820,615},0,0,cv::INTER_NEAREST);v.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ПОЛНАЯ IMU ЦЕПОЧКА v26",20,48,.72,white,2);double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double target=kTargets26[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}else if(done)uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,120,300,.70,yellow,3);}
  cv::Mat x=c(cv::Rect(860,70,560,850));drawBar(x,"YAW",yy,-60,60,target,kYawTol26,145);char b[256];snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d",phase+1,stable,kStable26);uiText(x,b,18,210,.42,white,1);snprintf(b,sizeof(b),"|gyro| %.3f   |acc| %.3f",gn,an);uiText(x,b,18,250,.42,white,1);
  if(!s.rows.empty()){const auto&r=s.rows.back();double vf=std::hypot(r.vfc_x,r.vfc_y),vb=std::hypot(r.vbg_x,r.vbg_y),vp=std::hypot(r.pim_x,r.pim_y),gr=std::hypot(r.grav_be_x,r.grav_be_y);snprintf(b,sizeof(b),"FC Vxy %.3f   PIM Vxy %.3f",vf,vp);uiText(x,b,18,330,.44,(vp>.5?red:green),2);snprintf(b,sizeof(b),"GYRO(BG) Vxy %.3f",vb);uiText(x,b,18,375,.44,(vb>.5?red:green),2);snprintf(b,sizeof(b),"G gyro↔FC %.3f°",r.g_bg_fc);uiText(x,b,18,425,.44,(r.g_bg_fc>1?red:green),2);snprintf(b,sizeof(b),"G staticBG↔FC %.3f°",r.g_st_fc);uiText(x,b,18,470,.44,(r.g_st_fc>1?red:green),2);snprintf(b,sizeof(b),"Accel body XY residual %.3f",gr);uiText(x,b,18,520,.42,(gr>.2?red:green),2);snprintf(b,sizeof(b),"PRED→OPT dV %.6f",r.pred_opt_dv);uiText(x,b,18,565,.42,white,1);snprintf(b,sizeof(b),"KF %lld",(long long)r.kf);uiText(x,b,18,610,.40,muted,1);}
  uiText(x,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);uiText(x,"Timing / Bias / R / Accel / PIM / OPT",18,750,.38,muted,1);uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);cv::imshow(kWindow26,c);
}

} // namespace jtzero_v26

int main(int argc,char**argv){using namespace jtzero_v26;using namespace jtzero_v10;google::InitGoogleLogging(argv[0]);FLAGS_no_incremental_pose=true;FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
 int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline26>pipe;std::thread pipe_thread;Telemetry tel;
 try{VIO::VioParams vp(params);pipe=std::make_shared<Pipeline26>(vp);pipe->install();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow26,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow26,1440,940);
 std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));std::deque<ImuSample10>iq,recent;AttSample10 prev_att{},cur_att{};bool have_att=false;State26 st;bool ready=false,done=false;int phase=0,stable=0;double ref=0;std::cout<<"\nJT-ZERO FULL CHAIN v26\nSPACE fixes zero+bias. Test: +30 -> 0 -> -30 -> 0.\n";
 while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
  if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
   if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
   else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
    if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us&&ready){st.phase=phase;process26(st,im,interpR(prev_att,cur_att,im.us),*pipe);}iq.pop_front();}}else{cur_att=na;have_att=true;}
    if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets26[phase])<=kYawTol26&&gn<=kGyroStill26&&std::abs(an-9.81)<=kAccTol26;if(ok)++stable;else stable=0;if(stable>=kStable26){std::cout<<"[STEP] "<<kNames26[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets26[phase]<<" deg.\n";}}}
   else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>q(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}recent.push_back(im);while(recent.size()>400)recent.pop_front();iq.push_back(im);while(iq.size()>400)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
  }}}
  if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
  if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
  if(now>=next_hud){hud26(last_gray,tel,ready,done,phase,stable,st,ref);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend26 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv){ref=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att&&recent.size()>=100){Eigen::Vector3d m=Eigen::Vector3d::Zero();for(const auto&im:recent)m+=Eigen::Vector3d(im.gx,im.gy,im.gz);m/=recent.size();st.bg_static=m;st.bg_backend0=bb.BG;st.ba0=bb.BA;st.pim_v0=bb.Vpred;st.opt_v0=bb.Vopt;st.world_map=bb.Rpred*Rfc0.transpose();st.Rbg=bb.Rpred;st.Rst=bb.Rpred;st.vfc.setZero();st.vbg.setZero();st.vst.setZero();st.last_us=0;st.armed=true;ready=true;std::cout<<"[ZERO] KF "<<bb.kf<<" BA ["<<bb.BA.transpose()<<"] BG backend ["<<bb.BG.transpose()<<"] BG static ["<<m.transpose()<<"]. Target +30 deg.\n";}}else if(key==' '&&done)break;}
 std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save26(st);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);std::cout<<"\n============================================================\nJT-ZERO FULL CHAIN v26 RESULT\n============================================================\naborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nsamples logged: "<<st.rows.size()<<"\n";summary26(st);std::cout<<"CSV: "<<kCsv26<<"\nOpen CSV:\n  code "<<kCsv26<<"\n";return aborted?2:0;
 }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
