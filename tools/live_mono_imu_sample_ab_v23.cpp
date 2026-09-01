// JT-ZERO v23: покадровое сравнение HIGHRES_IMU при YAW 0 -> +30 -> 0 -> -30 -> 0.
// TRUE IMU-only. Одинаковые raw IMU, dt и фиксированные startup BA/BG.
// Сравнение: FC ATTITUDE integration, gyro integration, Kimera/GTSAM PIM velocity.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

namespace jtzero_v23 {
using namespace jtzero_v10;

constexpr const char* kCsv23 = "/home/vio/jtzero_live_imu_sample_ab_v23.csv";
constexpr const char* kWindow23 = "JT-ZERO: IMU A/B v23";
constexpr double kTargets23[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames23[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable23 = 20;
constexpr double kYawTol23 = 2.0;
constexpr double kGyroStill23 = 0.08;
constexpr double kAccTol23 = 0.35;

struct Backend23 {
  bool valid=false;
  int64_t timestamp_ns=0, keyframe=0;
  Eigen::Matrix3d Rpred=Eigen::Matrix3d::Identity();
  Eigen::Vector3d Vpred=Eigen::Vector3d::Zero();
  Eigen::Vector3d BA=Eigen::Vector3d::Zero();
  Eigen::Vector3d BG=Eigen::Vector3d::Zero();
};

class Pipeline23 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline23(const VIO::VioParams& p):VIO::MonoImuPipeline(p){}
  void installCallback(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out){
      if(!out)return;
      Backend23 b;
      const auto& st=out->W_State_Blkf_;
      b.valid=true;
      b.timestamp_ns=st.timestamp_;
      b.keyframe=out->cur_kf_id_;
      b.Rpred=out->debug_info_.navstate_k_.pose().rotation().matrix();
      b.Vpred=out->debug_info_.navstate_k_.velocity();
      b.BA=st.imu_bias_.accelerometer();
      b.BG=st.imu_bias_.gyroscope();
      std::lock_guard<std::mutex> l(m_); latest_=b;
    });
  }
  bool latest(Backend23* b)const{
    std::lock_guard<std::mutex> l(m_);
    if(!latest_.valid)return false;
    *b=latest_;
    return true;
  }
 private:
  mutable std::mutex m_;
  Backend23 latest_;
};

struct Row23 {
  int phase=0;
  uint64_t imu_us=0;
  double dt=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  double acx=0,acy=0,acz=0,wcx=0,wcy=0,wcz=0;
  double afx=0,afy=0,afz=0,agx=0,agy=0,agz=0;
  double vfx=0,vfy=0,vfz=0,vgx=0,vgy=0,vgz=0;
  int64_t keyframe=0;
  double pvdx=0,pvdy=0,pvdz=0;
  double resfx=0,resfy=0,resfz=0;
  double resgx=0,resgy=0,resgz=0;
};

struct Integrator23 {
  bool armed=false;
  uint64_t last_us=0;
  Eigen::Matrix3d world_map=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rgyro=Eigen::Matrix3d::Identity();
  Eigen::Vector3d fixed_ba=Eigen::Vector3d::Zero();
  Eigen::Vector3d fixed_bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d vfc=Eigen::Vector3d::Zero();
  Eigen::Vector3d vgyro=Eigen::Vector3d::Zero();
  Eigen::Vector3d pim_v0=Eigen::Vector3d::Zero();
  std::vector<Row23> rows;
};

static void process23(Integrator23& in,int phase,const ImuSample10& s,
                      const Eigen::Matrix3d& Rfc,const Pipeline23& pipe){
  if(!in.armed)return;
  double dt=0.0;
  if(in.last_us && s.us>in.last_us)dt=(s.us-in.last_us)*1e-6;
  in.last_us=s.us;
  if(dt<0.0 || dt>0.03)dt=0.0;

  const Eigen::Vector3d acc(s.ax,s.ay,s.az), gyr(s.gx,s.gy,s.gz);
  const Eigen::Vector3d ac=acc-in.fixed_ba;
  const Eigen::Vector3d wc=gyr-in.fixed_bg;

  if(dt>0.0){
    const Eigen::Vector3d th=wc*dt;
    const double a=th.norm();
    if(a>1e-12)in.Rgyro=in.Rgyro*Eigen::AngleAxisd(a,th/a).toRotationMatrix();
  }

  const Eigen::Matrix3d RfcW=in.world_map*Rfc;
  const Eigen::Vector3d gN(0,0,9.81);
  const Eigen::Vector3d af=RfcW*ac+gN;
  const Eigen::Vector3d ag=in.Rgyro*ac+gN;
  if(dt>0.0){in.vfc+=af*dt;in.vgyro+=ag*dt;}

  Backend23 b; pipe.latest(&b);
  const Eigen::Vector3d pvd=b.valid?(b.Vpred-in.pim_v0):Eigen::Vector3d::Zero();
  const Eigen::Vector3d rf=pvd-in.vfc, rg=pvd-in.vgyro;

  Row23 r;
  r.phase=phase;r.imu_us=s.us;r.dt=dt;
  r.ax=s.ax;r.ay=s.ay;r.az=s.az;r.gx=s.gx;r.gy=s.gy;r.gz=s.gz;
  r.acx=ac.x();r.acy=ac.y();r.acz=ac.z();r.wcx=wc.x();r.wcy=wc.y();r.wcz=wc.z();
  r.afx=af.x();r.afy=af.y();r.afz=af.z();r.agx=ag.x();r.agy=ag.y();r.agz=ag.z();
  r.vfx=in.vfc.x();r.vfy=in.vfc.y();r.vfz=in.vfc.z();r.vgx=in.vgyro.x();r.vgy=in.vgyro.y();r.vgz=in.vgyro.z();
  r.keyframe=b.valid?b.keyframe:0;r.pvdx=pvd.x();r.pvdy=pvd.y();r.pvdz=pvd.z();
  r.resfx=rf.x();r.resfy=rf.y();r.resfz=rf.z();r.resgx=rg.x();r.resgy=rg.y();r.resgz=rg.z();
  in.rows.push_back(r);
}

static void save23(const Integrator23& in){
  std::ofstream f(kCsv23,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,imu_us,dt,ax,ay,az,gx,gy,gz,acx,acy,acz,wcx,wcy,wcz,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,keyframe,pim_dv_x,pim_dv_y,pim_dv_z,pim_minus_fc_x,pim_minus_fc_y,pim_minus_fc_z,pim_minus_gyro_x,pim_minus_gyro_y,pim_minus_gyro_z\n";
  for(const auto&r:in.rows){
    f<<r.phase<<','<<kNames23[r.phase]<<','<<r.imu_us<<','<<r.dt<<','
     <<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','
     <<r.acx<<','<<r.acy<<','<<r.acz<<','<<r.wcx<<','<<r.wcy<<','<<r.wcz<<','
     <<r.afx<<','<<r.afy<<','<<r.afz<<','<<r.agx<<','<<r.agy<<','<<r.agz<<','
     <<r.vfx<<','<<r.vfy<<','<<r.vfz<<','<<r.vgx<<','<<r.vgy<<','<<r.vgz<<','
     <<r.keyframe<<','<<r.pvdx<<','<<r.pvdy<<','<<r.pvdz<<','
     <<r.resfx<<','<<r.resfy<<','<<r.resfz<<','<<r.resgx<<','<<r.resgy<<','<<r.resgz<<'\n';
  }
}

static void hud23(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,
                  int phase,int stable,const Integrator23& in,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),v;
  if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);
  cv::resize(v,v,{820,615},0,0,cv::INTER_NEAREST);v.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ПОКАДРОВЫЙ IMU A/B v23",20,48,.72,white,2);
  double yy=0,gn=0,an=0;{
    std::lock_guard<std::mutex>q(tel.mutex);
    yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);
  }
  const double target=kTargets23[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done)uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);
  else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,120,300,.70,yellow,3);}

  cv::Mat s=c(cv::Rect(860,70,560,850));drawBar(s,"YAW",yy,-60,60,target,kYawTol23,145);char b[256];
  snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d",phase+1,stable,kStable23);uiText(s,b,18,210,.42,white,1);
  snprintf(b,sizeof(b),"|gyro| %.3f   |acc| %.3f",gn,an);uiText(s,b,18,250,.42,white,1);
  if(!in.rows.empty()){
    const auto&r=in.rows.back();
    const double vf=std::hypot(r.vfx,r.vfy),vg=std::hypot(r.vgx,r.vgy),vp=std::hypot(r.pvdx,r.pvdy),rf=std::hypot(r.resfx,r.resfy),rg=std::hypot(r.resgx,r.resgy);
    snprintf(b,sizeof(b),"FC интегратор Vxy: %.3f м/с",vf);uiText(s,b,18,330,.46,vf>.5?red:green,2);
    snprintf(b,sizeof(b),"GYRO интегратор Vxy: %.3f м/с",vg);uiText(s,b,18,375,.46,vg>.5?red:green,2);
    snprintf(b,sizeof(b),"PIM ΔVxy: %.3f м/с",vp);uiText(s,b,18,420,.48,vp>.5?red:green,2);
    snprintf(b,sizeof(b),"PIM-FC XY: %.3f м/с",rf);uiText(s,b,18,475,.46,rf>.5?red:green,2);
    snprintf(b,sizeof(b),"PIM-GYRO XY: %.3f м/с",rg);uiText(s,b,18,520,.46,rg>.5?red:green,2);
    snprintf(b,sizeof(b),"aFC XY: %+.3f %+.3f",r.afx,r.afy);uiText(s,b,18,575,.42,white,1);
    snprintf(b,sizeof(b),"aGYRO XY: %+.3f %+.3f",r.agx,r.agy);uiText(s,b,18,615,.42,white,1);
    snprintf(b,sizeof(b),"KF: %lld",(long long)r.keyframe);uiText(s,b,18,655,.42,muted,1);
  }
  uiText(s,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);
  uiText(s,"Одинаковые IMU / dt / BA / BG",18,750,.40,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);cv::imshow(kWindow23,c);
}

static void printSummary23(const Integrator23& in){
  for(int p=0;p<4;++p){
    bool first=true;Row23 a,z;double max_rf=0,max_rg=0;
    for(const auto&r:in.rows)if(r.phase==p){
      if(first){a=r;first=false;}z=r;
      max_rf=std::max(max_rf,std::hypot(r.resfx,r.resfy));
      max_rg=std::max(max_rg,std::hypot(r.resgx,r.resgy));
    }
    if(first)continue;
    std::cout<<kNames23[p]
      <<": FC dVxy=["<<(z.vfx-a.vfx)<<", "<<(z.vfy-a.vfy)<<"]"
      <<" GYRO dVxy=["<<(z.vgx-a.vgx)<<", "<<(z.vgy-a.vgy)<<"]"
      <<" PIM dVxy=["<<(z.pvdx-a.pvdx)<<", "<<(z.pvdy-a.pvdy)<<"]"
      <<" max|PIM-FC|="<<max_rf
      <<" max|PIM-GYRO|="<<max_rg<<"\n";
  }
}

} // namespace jtzero_v23

int main(int argc,char**argv){
  using namespace jtzero_v23;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);
  FLAGS_no_incremental_pose=true;FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;
  std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline23>pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline23>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow23,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow23,1440,940);

    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator23 in;bool ready=false,done=false;int phase=0,stable=0;double ref=0;
    std::cout<<"\nJT-ZERO IMU SAMPLE A/B v23\nSPACE fixes zero+bias. Test: +30 -> 0 -> -30 -> 0.\n";

    while(true){
      const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{
          std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us&&ready)process23(in,phase,im,interpR(prev_att,cur_att,im.us),*pipe);iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets23[phase])<=kYawTol23&&gn<=kGyroStill23&&std::abs(an-9.81)<=kAccTol23;if(ok)++stable;else stable=0;if(stable>=kStable23){std::cout<<"[STEP] "<<kNames23[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets23[phase]<<" deg.\n";}}}
        }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{
          std::lock_guard<std::mutex>q(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}
          iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud23(last_gray,tel,ready,done,phase,stable,in,ref);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){Backend23 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv){ref=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){in.world_map=bb.Rpred*Rfc0.transpose();in.Rgyro=bb.Rpred;in.fixed_ba=bb.BA;in.fixed_bg=bb.BG;in.pim_v0=bb.Vpred;in.vfc.setZero();in.vgyro.setZero();in.last_us=0;in.armed=true;ready=true;std::cout<<"[ZERO] KF "<<bb.keyframe<<" BA ["<<bb.BA.transpose()<<"] BG ["<<bb.BG.transpose()<<"]. Target +30 deg.\n";}}
      else if(key==' '&&done)break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save23(in);cv::destroyAllWindows();
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO IMU SAMPLE A/B v23 RESULT\n============================================================\naborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nsamples logged: "<<in.rows.size()<<"\nfixed BA: ["<<in.fixed_ba.transpose()<<"]\nfixed BG: ["<<in.fixed_bg.transpose()<<"]\n";printSummary23(in);std::cout<<"CSV: "<<kCsv23<<"\nOpen CSV:\n  code "<<kCsv23<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
