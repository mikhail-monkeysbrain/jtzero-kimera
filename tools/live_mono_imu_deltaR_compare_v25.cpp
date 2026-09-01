// JT-ZERO v25: сравнение приращений ориентации IMU и FC на одних ATTITUDE-интервалах.
// Стендовый маршрут: YAW 0 -> +30 -> 0 -> -30 -> 0.
// Не использует backend для вычислений: BG берется как среднее последних неподвижных HIGHRES_IMU перед SPACE.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v25 {
using namespace jtzero_v10;

constexpr const char* kCsv25 = "/home/vio/jtzero_live_deltaR_compare_v25.csv";
constexpr const char* kWindow25 = "JT-ZERO: DELTA-R IMU / FC v25";
constexpr double kTargets25[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames25[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable25 = 20;
constexpr double kYawTol25 = 2.0;
constexpr double kGyroStill25 = 0.08;
constexpr double kAccTol25 = 0.35;

struct Att25 {
  uint64_t us=0;
  double roll=0,pitch=0,yaw=0;
};

struct Row25 {
  int phase=0;
  uint64_t att_us=0;
  double dt=0;
  int imu_n=0;
  double ix=0,iy=0,iz=0;
  double fx=0,fy=0,fz=0;
  double ex=0,ey=0,ez=0;
  double interval_err=0;
  double cum_rot_err=0,cum_grav_err=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
};

struct Log25 {
  bool armed=false;
  Eigen::Vector3d fixed_bg=Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rimu=Eigen::Matrix3d::Identity();
  Att25 prev_att{};
  Eigen::Matrix3d prev_Rfc=Eigen::Matrix3d::Identity();
  bool have_prev=false;
  std::vector<Row25> rows;
};

static double clamp25(double x){return std::max(-1.0,std::min(1.0,x));}
static double rotAngle25(const Eigen::Matrix3d& R){return std::acos(clamp25((R.trace()-1.0)*0.5))*180.0/kPi;}
static double gravErr25(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  const Eigen::Vector3d z(0,0,1);
  return std::acos(clamp25((A.transpose()*z).normalized().dot((B.transpose()*z).normalized())))*180.0/kPi;
}
static Eigen::Matrix3d exp25(const Eigen::Vector3d& th){
  const double a=th.norm();
  if(a<=1e-12)return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a,th/a).toRotationMatrix();
}
static Eigen::Vector3d rotVecDeg25(const Eigen::Matrix3d& R){
  Eigen::AngleAxisd aa(R);
  return aa.axis()*aa.angle()*180.0/kPi;
}

static bool processInterval25(Log25& log,int phase,const Att25& cur,std::deque<ImuSample10>& iq){
  if(!log.armed)return false;
  const Eigen::Matrix3d Rfc=fcRnedFlu(cur.roll,cur.pitch,cur.yaw);
  if(!log.have_prev){
    log.prev_att=cur;log.prev_Rfc=Rfc;log.Rimu=Rfc;log.have_prev=true;
    while(!iq.empty()&&iq.front().us<=cur.us)iq.pop_front();
    return false;
  }
  if(cur.us<=log.prev_att.us)return false;

  std::vector<ImuSample10> s;
  while(!iq.empty()&&iq.front().us<=cur.us){
    if(iq.front().us>=log.prev_att.us)s.push_back(iq.front());
    iq.pop_front();
  }
  if(s.empty()){log.prev_att=cur;log.prev_Rfc=Rfc;return false;}

  Eigen::Matrix3d dRi=Eigen::Matrix3d::Identity();
  uint64_t t=log.prev_att.us;
  Eigen::Vector3d lastw(s.front().gx,s.front().gy,s.front().gz);lastw-=log.fixed_bg;
  for(const auto& im:s){
    if(im.us>t){double dt=(im.us-t)*1e-6;if(dt>0&&dt<0.03)dRi=dRi*exp25(lastw*dt);}
    lastw=Eigen::Vector3d(im.gx,im.gy,im.gz)-log.fixed_bg;
    t=im.us;
  }
  if(cur.us>t){double dt=(cur.us-t)*1e-6;if(dt>0&&dt<0.03)dRi=dRi*exp25(lastw*dt);}

  const Eigen::Matrix3d dRf=log.prev_Rfc.transpose()*Rfc;
  const Eigen::Matrix3d dE=dRf.transpose()*dRi;
  log.Rimu=log.Rimu*dRi;

  const Eigen::Vector3d vi=rotVecDeg25(dRi),vf=rotVecDeg25(dRf),ve=rotVecDeg25(dE);
  Row25 r;
  r.phase=phase;r.att_us=cur.us;r.dt=(cur.us-log.prev_att.us)*1e-6;r.imu_n=(int)s.size();
  r.ix=vi.x();r.iy=vi.y();r.iz=vi.z();r.fx=vf.x();r.fy=vf.y();r.fz=vf.z();
  r.ex=ve.x();r.ey=ve.y();r.ez=ve.z();r.interval_err=rotAngle25(dE);
  r.cum_rot_err=rotAngle25(Rfc.transpose()*log.Rimu);r.cum_grav_err=gravErr25(log.Rimu,Rfc);
  r.fc_roll=cur.roll;r.fc_pitch=cur.pitch;r.fc_yaw=cur.yaw;
  log.rows.push_back(r);
  log.prev_att=cur;log.prev_Rfc=Rfc;
  return true;
}

static void save25(const Log25& log){
  std::ofstream f(kCsv25,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,att_us,dt,imu_n,dtheta_imu_x_deg,dtheta_imu_y_deg,dtheta_imu_z_deg,dtheta_fc_x_deg,dtheta_fc_y_deg,dtheta_fc_z_deg,delta_err_x_deg,delta_err_y_deg,delta_err_z_deg,interval_rot_err_deg,cum_rot_err_deg,cum_grav_err_deg,fc_roll_deg,fc_pitch_deg,fc_yaw_deg\n";
  for(const auto&r:log.rows)f<<r.phase<<','<<kNames25[r.phase]<<','<<r.att_us<<','<<r.dt<<','<<r.imu_n<<','<<r.ix<<','<<r.iy<<','<<r.iz<<','<<r.fx<<','<<r.fy<<','<<r.fz<<','<<r.ex<<','<<r.ey<<','<<r.ez<<','<<r.interval_err<<','<<r.cum_rot_err<<','<<r.cum_grav_err<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<'\n';
}

static void summary25(const Log25& log){
  for(int p=0;p<4;++p){
    Eigen::Vector3d si=Eigen::Vector3d::Zero(),sf=Eigen::Vector3d::Zero(),se=Eigen::Vector3d::Zero();
    double ss=0,maxi=0,maxr=0,maxg=0;size_t n=0;
    for(const auto&r:log.rows)if(r.phase==p){
      si+=Eigen::Vector3d(r.ix,r.iy,r.iz);sf+=Eigen::Vector3d(r.fx,r.fy,r.fz);se+=Eigen::Vector3d(r.ex,r.ey,r.ez);
      ss+=r.interval_err*r.interval_err;maxi=std::max(maxi,r.interval_err);maxr=std::max(maxr,r.cum_rot_err);maxg=std::max(maxg,r.cum_grav_err);++n;
    }
    if(!n)continue;
    std::cout<<kNames25[p]<<": sum dTheta IMU=["<<si.transpose()<<"] deg"
             <<" FC=["<<sf.transpose()<<"] deg"
             <<" sum errVec=["<<se.transpose()<<"] deg"
             <<" RMS interval="<<std::sqrt(ss/n)<<" deg"
             <<" max interval="<<maxi<<" deg"
             <<" maxCumRot="<<maxr<<" deg maxCumGrav="<<maxg<<" deg\n";
  }
}

static void hud25(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int phase,int stable,const Log25& log,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),v;
  if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);
  cv::resize(v,v,{820,615},0,0,cv::INTER_NEAREST);v.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ΔR IMU / FC v25",20,48,.72,white,2);
  double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double target=kTargets25[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BG",135,355,.70,yellow,2);}else if(done)uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,120,300,.70,yellow,3);}
  cv::Mat s=c(cv::Rect(860,70,560,850));drawBar(s,"YAW",yy,-60,60,target,kYawTol25,145);char b[256];
  snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d",phase+1,stable,kStable25);uiText(s,b,18,210,.42,white,1);
  snprintf(b,sizeof(b),"|gyro| %.3f   |acc| %.3f",gn,an);uiText(s,b,18,250,.42,white,1);
  if(!log.rows.empty()){const auto&r=log.rows.back();
    snprintf(b,sizeof(b),"Δθ IMU: %+.3f %+.3f %+.3f°",r.ix,r.iy,r.iz);uiText(s,b,18,330,.40,white,1);
    snprintf(b,sizeof(b),"Δθ FC:  %+.3f %+.3f %+.3f°",r.fx,r.fy,r.fz);uiText(s,b,18,370,.40,white,1);
    snprintf(b,sizeof(b),"Ошибка интервала: %.3f°",r.interval_err);uiText(s,b,18,425,.46,r.interval_err>.25?red:green,2);
    snprintf(b,sizeof(b),"Накопленная R ошибка: %.3f°",r.cum_rot_err);uiText(s,b,18,475,.46,r.cum_rot_err>1?red:green,2);
    snprintf(b,sizeof(b),"Ошибка gravity: %.3f°",r.cum_grav_err);uiText(s,b,18,525,.46,r.cum_grav_err>1?red:green,2);
    snprintf(b,sizeof(b),"IMU samples/interval: %d",r.imu_n);uiText(s,b,18,575,.42,muted,1);
  }
  uiText(s,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);uiText(s,"Сравнение ΔR на ATTITUDE-интервалах",18,750,.38,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);cv::imshow(kWindow25,c);
}

} // namespace jtzero_v25

int main(int argc,char**argv){
  using namespace jtzero_v25;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);
  int cfd=-1,sfd=-1;bool streaming=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;Telemetry tel;
  try{
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
    cv::setNumThreads(1);cv::namedWindow(kWindow25,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow25,1440,940);

    std::deque<ImuSample10> iq,recent;Log25 log;bool ready=false,done=false;int phase=0,stable=0;double ref=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));int64_t next_hud=monotonicNs();
    std::cout<<"\nJT-ZERO DELTA-R COMPARE v25\nSPACE fixes zero+BG. Test: +30 -> 0 -> -30 -> 0.\n";

    while(true){
      const int64_t now=monotonicNs();pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};
              {std::lock_guard<std::mutex>q(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}
              iq.push_back(im);recent.push_back(im);while(iq.size()>800)iq.pop_front();while(recent.size()>200)recent.pop_front();
            }else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);Att25 at{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};
              {std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(at.yaw-tel.prev_yaw);tel.prev_yaw=at.yaw;tel.have_prev_yaw=true;tel.fc_roll=at.roll;tel.fc_pitch=at.pitch;tel.fc_yaw=at.yaw;tel.fc_valid=true;}
              if(ready)processInterval25(log,phase,at,iq);
              if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets25[phase])<=kYawTol25&&gn<=kGyroStill25&&std::abs(an-9.81)<=kAccTol25;if(ok)++stable;else stable=0;if(stable>=kStable25){std::cout<<"[STEP] "<<kNames25[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets25[phase]<<" deg.\n";}}
            }
          }
        }
      }
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!g.empty())last_gray=g;if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud25(last_gray,tel,ready,done,phase,stable,log,ref);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){bool fv=false;double rr=0,pp=0,yy0=0;{std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv){ref=tel.fc_accum_yaw;rr=tel.fc_roll;pp=tel.fc_pitch;yy0=tel.fc_yaw;}}if(fv&&recent.size()>=50){Eigen::Vector3d bg=Eigen::Vector3d::Zero();for(const auto&im:recent)bg+=Eigen::Vector3d(im.gx,im.gy,im.gz);bg/=double(recent.size());log.fixed_bg=bg;log.Rimu=fcRnedFlu(rr,pp,yy0);log.armed=true;log.have_prev=false;while(!iq.empty())iq.pop_front();ready=true;std::cout<<"[ZERO] BG ["<<bg.transpose()<<"]. Target +30 deg.\n";}else std::cout<<"[WAIT] Need valid ATTITUDE and >=50 IMU samples.\n";}
      else if(key==' '&&done)break;
    }

    save25(log);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO DELTA-R COMPARE v25 RESULT\n============================================================\naborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nintervals logged: "<<log.rows.size()<<"\nfixed BG: ["<<log.fixed_bg.transpose()<<"]\n";summary25(log);std::cout<<"CSV: "<<kCsv25<<"\nOpen CSV:\n  code "<<kCsv25<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
