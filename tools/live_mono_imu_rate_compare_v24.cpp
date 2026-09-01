// JT-ZERO v24: прямое сравнение HIGHRES_IMU gyro и MAVLink ATTITUDE body rates.
// Стендовый маршрут: YAW 0 -> +30 -> 0 -> -30 -> 0.
// Оба потока приводятся из FRD в FLU: [x, -y, -z].

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v24 {
using namespace jtzero_v10;

constexpr const char* kCsv24 = "/home/vio/jtzero_live_rate_compare_v24.csv";
constexpr const char* kWindow24 = "JT-ZERO: GYRO / FC RATES v24";
constexpr double kTargets24[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames24[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable24 = 20;
constexpr double kYawTol24 = 2.0;
constexpr double kGyroStill24 = 0.08;
constexpr double kAccTol24 = 0.35;

struct Att24 {
  uint64_t us=0;
  double roll=0,pitch=0,yaw=0;
  double wx=0,wy=0,wz=0; // FLU body rates
};

struct Row24 {
  int phase=0;
  uint64_t imu_us=0;
  double dt=0;
  double igx=0,igy=0,igz=0;
  double fwx=0,fwy=0,fwz=0;
  double dgx=0,dgy=0,dgz=0;
  double iangx=0,iangy=0,iangz=0;
  double fangx=0,fangy=0,fangz=0;
  double rot_err=0,grav_err=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
};

struct Log24 {
  bool armed=false;
  uint64_t last_us=0;
  Eigen::Vector3d fixed_bg=Eigen::Vector3d::Zero();
  Eigen::Vector3d int_imu=Eigen::Vector3d::Zero();
  Eigen::Vector3d int_fc=Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rimu=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rrate=Eigen::Matrix3d::Identity();
  std::vector<Row24> rows;
};

static double clamp24(double x){return std::max(-1.0,std::min(1.0,x));}
static double rotErr24(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  const Eigen::Matrix3d D=A.transpose()*B;
  return std::acos(clamp24((D.trace()-1.0)*0.5))*180.0/kPi;
}
static double gravErr24(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  const Eigen::Vector3d z(0,0,1);
  return std::acos(clamp24((A.transpose()*z).normalized().dot((B.transpose()*z).normalized())))*180.0/kPi;
}
static Eigen::Matrix3d stepR24(const Eigen::Matrix3d& R,const Eigen::Vector3d& w,double dt){
  if(dt<=0)return R;
  const Eigen::Vector3d th=w*dt;
  const double a=th.norm();
  if(a<=1e-12)return R;
  return R*Eigen::AngleAxisd(a,th/a).toRotationMatrix();
}
static Att24 interp24(const Att24& a,const Att24& b,uint64_t us){
  if(b.us<=a.us)return b;
  double t=(double)(us-a.us)/(double)(b.us-a.us);
  t=std::max(0.0,std::min(1.0,t));
  Att24 q;
  q.us=us;
  q.roll=a.roll+(b.roll-a.roll)*t;
  q.pitch=a.pitch+(b.pitch-a.pitch)*t;
  double dy=wrapDeg(b.yaw-a.yaw);
  q.yaw=a.yaw+dy*t;
  q.wx=a.wx+(b.wx-a.wx)*t;
  q.wy=a.wy+(b.wy-a.wy)*t;
  q.wz=a.wz+(b.wz-a.wz)*t;
  return q;
}

static void process24(Log24& log,int phase,const ImuSample10& im,const Att24& at){
  if(!log.armed)return;
  double dt=0;
  if(log.last_us&&im.us>log.last_us)dt=(im.us-log.last_us)*1e-6;
  log.last_us=im.us;
  if(dt<0||dt>0.03)dt=0;

  const Eigen::Vector3d wi(im.gx,im.gy,im.gz);
  const Eigen::Vector3d wimu=wi-log.fixed_bg;
  const Eigen::Vector3d wfc(at.wx,at.wy,at.wz);
  if(dt>0){
    log.int_imu+=wimu*dt;
    log.int_fc+=wfc*dt;
    log.Rimu=stepR24(log.Rimu,wimu,dt);
    log.Rrate=stepR24(log.Rrate,wfc,dt);
  }
  const Eigen::Matrix3d Rfc=fcRnedFlu(at.roll,at.pitch,at.yaw);

  Row24 r;
  r.phase=phase;r.imu_us=im.us;r.dt=dt;
  r.igx=wimu.x();r.igy=wimu.y();r.igz=wimu.z();
  r.fwx=wfc.x();r.fwy=wfc.y();r.fwz=wfc.z();
  r.dgx=wimu.x()-wfc.x();r.dgy=wimu.y()-wfc.y();r.dgz=wimu.z()-wfc.z();
  r.iangx=log.int_imu.x()*180.0/kPi;r.iangy=log.int_imu.y()*180.0/kPi;r.iangz=log.int_imu.z()*180.0/kPi;
  r.fangx=log.int_fc.x()*180.0/kPi;r.fangy=log.int_fc.y()*180.0/kPi;r.fangz=log.int_fc.z()*180.0/kPi;
  r.rot_err=rotErr24(log.Rimu,Rfc);r.grav_err=gravErr24(log.Rimu,Rfc);
  r.fc_roll=at.roll;r.fc_pitch=at.pitch;r.fc_yaw=at.yaw;
  log.rows.push_back(r);
}

static void save24(const Log24& log){
  std::ofstream f(kCsv24,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,imu_us,dt,imu_wx,imu_wy,imu_wz,fc_wx,fc_wy,fc_wz,dw_x,dw_y,dw_z,int_imu_x_deg,int_imu_y_deg,int_imu_z_deg,int_fc_x_deg,int_fc_y_deg,int_fc_z_deg,rot_imu_vs_fc_deg,gravity_imu_vs_fc_deg,fc_roll_deg,fc_pitch_deg,fc_yaw_deg\n";
  for(const auto&r:log.rows){
    f<<r.phase<<','<<kNames24[r.phase]<<','<<r.imu_us<<','<<r.dt<<','
     <<r.igx<<','<<r.igy<<','<<r.igz<<','<<r.fwx<<','<<r.fwy<<','<<r.fwz<<','
     <<r.dgx<<','<<r.dgy<<','<<r.dgz<<','
     <<r.iangx<<','<<r.iangy<<','<<r.iangz<<','<<r.fangx<<','<<r.fangy<<','<<r.fangz<<','
     <<r.rot_err<<','<<r.grav_err<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<'\n';
  }
}

static void summary24(const Log24& log){
  for(int p=0;p<4;++p){
    bool first=true;Row24 a,z;double maxwx=0,maxwy=0,maxwz=0,maxrot=0,maxgrav=0;
    double ssx=0,ssy=0,ssz=0;size_t n=0;
    for(const auto&r:log.rows)if(r.phase==p){
      if(first){a=r;first=false;}z=r;
      maxwx=std::max(maxwx,std::abs(r.dgx));maxwy=std::max(maxwy,std::abs(r.dgy));maxwz=std::max(maxwz,std::abs(r.dgz));
      maxrot=std::max(maxrot,r.rot_err);maxgrav=std::max(maxgrav,r.grav_err);
      ssx+=r.dgx*r.dgx;ssy+=r.dgy*r.dgy;ssz+=r.dgz*r.dgz;++n;
    }
    if(first)continue;
    std::cout<<kNames24[p]
      <<": dAng IMU=["<<(z.iangx-a.iangx)<<", "<<(z.iangy-a.iangy)<<", "<<(z.iangz-a.iangz)<<"] deg"
      <<" dAng FC=["<<(z.fangx-a.fangx)<<", "<<(z.fangy-a.fangy)<<", "<<(z.fangz-a.fangz)<<"] deg"
      <<" RMS dW=["<<std::sqrt(ssx/n)<<", "<<std::sqrt(ssy/n)<<", "<<std::sqrt(ssz/n)<<"] rad/s"
      <<" max dW=["<<maxwx<<", "<<maxwy<<", "<<maxwz<<"]"
      <<" maxRot="<<maxrot<<" deg maxGrav="<<maxgrav<<" deg\n";
  }
}

static void hud24(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int phase,int stable,const Log24& log,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),v;
  if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);
  cv::resize(v,v,{820,615},0,0,cv::INTER_NEAREST);v.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: GYRO / СКОРОСТИ FC v24",20,48,.72,white,2);
  double yy=0,gn=0,an=0;{
    std::lock_guard<std::mutex>q(tel.mutex);
    yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);
  }
  double target=kTargets24[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BG",135,355,.70,yellow,2);}
  else if(done)uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);
  else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,120,300,.70,yellow,3);}
  cv::Mat s=c(cv::Rect(860,70,560,850));drawBar(s,"YAW",yy,-60,60,target,kYawTol24,145);char b[256];
  snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d",phase+1,stable,kStable24);uiText(s,b,18,210,.42,white,1);
  snprintf(b,sizeof(b),"|gyro| %.3f   |acc| %.3f",gn,an);uiText(s,b,18,250,.42,white,1);
  if(!log.rows.empty()){
    const auto&r=log.rows.back();
    snprintf(b,sizeof(b),"IMU ω: %+.3f %+.3f %+.3f",r.igx,r.igy,r.igz);uiText(s,b,18,330,.40,white,1);
    snprintf(b,sizeof(b),"FC  ω: %+.3f %+.3f %+.3f",r.fwx,r.fwy,r.fwz);uiText(s,b,18,370,.40,white,1);
    snprintf(b,sizeof(b),"Δω:   %+.3f %+.3f %+.3f",r.dgx,r.dgy,r.dgz);uiText(s,b,18,415,.42,(std::hypot(r.dgx,r.dgy)>.03||std::abs(r.dgz)>.03)?red:green,2);
    snprintf(b,sizeof(b),"∫IMU XYZ: %+.1f %+.1f %+.1f°",r.iangx,r.iangy,r.iangz);uiText(s,b,18,470,.40,white,1);
    snprintf(b,sizeof(b),"∫FC  XYZ: %+.1f %+.1f %+.1f°",r.fangx,r.fangy,r.fangz);uiText(s,b,18,510,.40,white,1);
    snprintf(b,sizeof(b),"R IMU ↔ FC: %.3f°",r.rot_err);uiText(s,b,18,565,.46,r.rot_err>1?red:green,2);
    snprintf(b,sizeof(b),"Gravity IMU ↔ FC: %.3f°",r.grav_err);uiText(s,b,18,610,.46,r.grav_err>1?red:green,2);
  }
  uiText(s,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);
  uiText(s,"Сравнение body rates в FLU",18,750,.40,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);cv::imshow(kWindow24,c);
}

} // namespace jtzero_v24

int main(int argc,char**argv){
  using namespace jtzero_v24;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;
  std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline10>pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline10>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow24,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow24,1440,940);

    std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10>iq;Att24 prev_att{},cur_att{};bool have_att=false;Log24 log;bool ready=false,done=false;int phase=0,stable=0;double ref=0;
    std::cout<<"\nJT-ZERO RATE COMPARE v24\nSPACE fixes zero+BG. Test: +30 -> 0 -> -30 -> 0.\n";

    while(true){
      const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            const int64_t rx=monotonicNs();
            if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){
              mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);
              if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}
            }else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);
              Att24 na;na.us=(uint64_t)a.time_boot_ms*1000ULL;na.roll=a.roll*180.0/kPi;na.pitch=a.pitch*180.0/kPi;na.yaw=a.yaw*180.0/kPi;na.wx=a.rollspeed;na.wy=-a.pitchspeed;na.wz=-a.yawspeed;
              {
                std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;
              }
              if(have_att){
                prev_att=cur_att;cur_att=na;
                while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us&&ready)process24(log,phase,im,interp24(prev_att,cur_att,im.us));iq.pop_front();}
              }else{cur_att=na;have_att=true;}
              if(ready&&!done){
                double yy,gn,an;{
                  std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);
                }
                bool ok=std::abs(yy-kTargets24[phase])<=kYawTol24&&gn<=kGyroStill24&&std::abs(an-9.81)<=kAccTol24;
                if(ok)++stable;else stable=0;
                if(stable>=kStable24){std::cout<<"[STEP] "<<kNames24[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets24[phase]<<" deg.\n";}
              }
            }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};
              {std::lock_guard<std::mutex>q(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}
              iq.push_back(im);while(iq.size()>300)iq.pop_front();
              if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}
            }
          }
        }
      }
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){
        for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}
      }
      if(now>=next_hud){hud24(last_gray,tel,ready,done,phase,stable,log,ref);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){
        Backend10 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d R0=Eigen::Matrix3d::Identity();
        {std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv){ref=tel.fc_accum_yaw;R0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}
        if(bv&&fv&&have_att){log.fixed_bg=Eigen::Vector3d(bb.bgx,bb.bgy,bb.bgz);log.Rimu=R0;log.Rrate=R0;log.last_us=0;log.int_imu.setZero();log.int_fc.setZero();log.armed=true;ready=true;std::cout<<"[ZERO] BG ["<<log.fixed_bg.transpose()<<"]. Target +30 deg.\n";}
      }else if(key==' '&&done)break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save24(log);cv::destroyAllWindows();
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO RATE COMPARE v24 RESULT\n============================================================\naborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nsamples logged: "<<log.rows.size()<<"\nfixed BG: ["<<log.fixed_bg.transpose()<<"]\n";summary24(log);std::cout<<"CSV: "<<kCsv24<<"\nOpen CSV:\n  code "<<kCsv24<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
