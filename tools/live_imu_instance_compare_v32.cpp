// JT-ZERO v32: compare HIGHRES_IMU / SCALED_IMU / SCALED_IMU2 / SCALED_IMU3 against ATTITUDE body rates.
// Manual stand test with Russian GUI. Route: yaw 0 -> +30 -> 0 -> -30 -> 0.
// All gyro streams are converted FRD -> FLU: [x, -y, -z].

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>

namespace jtzero_v32 {
using namespace jtzero_v10;

constexpr const char* kCsv32 = "/home/vio/jtzero_live_imu_instance_compare_v32.csv";
constexpr const char* kWindow32 = "JT-ZERO: СРАВНЕНИЕ IMU ИСТОЧНИКОВ v32";
constexpr double kTargets32[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames32[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable32 = 20;
constexpr double kYawTol32 = 2.0;
constexpr double kGyroStill32 = 0.08;
constexpr double kAccTol32 = 0.35;

struct RateSample32 {
  bool valid=false;
  uint64_t us=0;
  Eigen::Vector3d w=Eigen::Vector3d::Zero();
};

struct SourceStats32 {
  uint64_t count=0;
  Eigen::Vector3d ss=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum=Eigen::Vector3d::Zero();
  double max_norm=0;
  void add(const Eigen::Vector3d& e){++count;sum+=e;ss+=e.cwiseProduct(e);max_norm=std::max(max_norm,e.norm());}
  Eigen::Vector3d mean() const {
    Eigen::Vector3d out = Eigen::Vector3d::Zero();
    if (count) out = sum / double(count);
    return out;
  }

  Eigen::Vector3d rms() const {
    Eigen::Vector3d out = Eigen::Vector3d::Zero();
    if (count) out = (ss / double(count)).cwiseSqrt();
    return out;
  }
};

struct Row32 {
  int phase=0;
  uint64_t t_us=0;
  double yaw=0;
  Eigen::Vector3d fc=Eigen::Vector3d::Zero();
  std::array<RateSample32,4> src;
};

struct Log32 {
  bool ready=false;
  int phase=0;
  std::array<RateSample32,4> latest;
  std::array<std::array<SourceStats32,4>,4> stats; // [phase][source]
  std::vector<Row32> rows;
};

static const char* srcName32(int i){
  static const char* n[4]={"HIGHRES_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3"};
  return n[i];
}

static Eigen::Vector3d frdGyroToFlu(double x,double y,double z){return Eigen::Vector3d(x,-y,-z);}
static bool fresh32(const RateSample32&s,uint64_t t){return s.valid && t>=s.us && t-s.us<=30000ULL;}

static void appendRow32(Log32& log,uint64_t t,double yaw,const Eigen::Vector3d& fc){
  if(!log.ready)return;
  Row32 r;r.phase=log.phase;r.t_us=t;r.yaw=yaw;r.fc=fc;r.src=log.latest;
  for(int i=0;i<4;++i)if(fresh32(r.src[i],t))log.stats[log.phase][i].add(r.src[i].w-fc);
  log.rows.push_back(r);
}

static void save32(const Log32& log){
  std::ofstream f(kCsv32,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,t_us,yaw_deg,fc_wx,fc_wy,fc_wz";
  for(int i=0;i<4;++i)f<<','<<srcName32(i)<<"_valid,"<<srcName32(i)<<"_wx,"<<srcName32(i)<<"_wy,"<<srcName32(i)<<"_wz";
  f<<"\n";
  for(const auto&r:log.rows){
    f<<r.phase<<','<<kNames32[r.phase]<<','<<r.t_us<<','<<r.yaw<<','<<r.fc.x()<<','<<r.fc.y()<<','<<r.fc.z();
    for(int i=0;i<4;++i){bool ok=fresh32(r.src[i],r.t_us);f<<','<<(ok?1:0)<<','<<r.src[i].w.x()<<','<<r.src[i].w.y()<<','<<r.src[i].w.z();}
    f<<'\n';
  }
}

static void summary32(const Log32& log){
  std::cout<<std::fixed<<std::setprecision(9);
  for(int p=0;p<4;++p){
    std::cout<<"\n"<<kNames32[p]<<"\n";
    for(int i=0;i<4;++i){const auto&s=log.stats[p][i];std::cout<<"  "<<srcName32(i)<<": n="<<s.count<<" mean dW=["<<s.mean().transpose()<<"] rms dW=["<<s.rms().transpose()<<"] max|dW|="<<s.max_norm<<" rad/s\n";}
  }
  std::cout<<"\nGLOBAL availability:\n";
  for(int i=0;i<4;++i){uint64_t n=0;for(int p=0;p<4;++p)n+=log.stats[p][i].count;std::cout<<"  "<<srcName32(i)<<": matched="<<n<<"\n";}
}

static void hud32(const Telemetry& tel,const Log32& log,bool ready,bool done,int phase,int stable,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(820,1280,CV_8UC3,bg);
  uiText(c,"JT-ZERO: СРАВНЕНИЕ ИСТОЧНИКОВ IMU v32",24,48,.72,white,2);
  double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  double target=kTargets32[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",110,150,.78,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",110,205,.62,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",110,150,.82,green,3);uiText(c,"SPACE — ВЫХОД",110,205,.55,white,2);}
  else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,85,155,.62,yellow,2);}
  cv::Mat s=c(cv::Rect(40,250,1200,500));drawBar(s,"YAW",yy,-60,60,target,kYawTol32,55);char b[256];
  snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d   |gyro| %.3f   |acc| %.3f",phase+1,stable,kStable32,gn,an);uiText(s,b,20,115,.42,white,1);
  for(int i=0;i<4;++i){uint64_t n=0;for(int p=0;p<4;++p)n+=log.stats[p][i].count;bool av=log.latest[i].valid;snprintf(b,sizeof(b),"%-12s  %s   пакетов/совпадений: %llu",srcName32(i),av?"ЕСТЬ":"НЕТ",(unsigned long long)n);uiText(s,b,20,175+i*55,.44,av?green:red,2);}
  uiText(s,"Цель: определить, какой IMU-поток ближе к ATTITUDE body rates",20,420,.40,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",40,790,.42,muted,1);
  cv::imshow(kWindow32,c);
}

} // namespace jtzero_v32

int main(int argc,char**argv){
  using namespace jtzero_v32;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);
  int sfd=-1;bool aborted=false,done=false;uint8_t sys=0,comp=0;mavlink_status_t mst{};mavlink_message_t msg{};Telemetry tel;Log32 log;
  bool ready=false;int phase=0,stable=0;double ref=0;uint64_t last_att_us=0;
  try{
    sfd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,100);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,100);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,100);
    cv::namedWindow(kWindow32,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow32,1280,820);
    std::cout<<"\nJT-ZERO IMU INSTANCE COMPARE v32\nSPACE starts. Route: +30 -> 0 -> -30 -> 0.\n";
    int64_t next_hud=monotonicNs();
    while(true){
      pollfd p{sfd,POLLIN,0};poll(&p,1,5);
      if(p.revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
        if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);log.latest[0].valid=true;log.latest[0].us=h.time_usec;log.latest[0].w=frdGyroToFlu(h.xgyro,h.ygyro,h.zgyro);{std::lock_guard<std::mutex>q(tel.mutex);tel.gx=log.latest[0].w.x();tel.gy=log.latest[0].w.y();tel.gz=log.latest[0].w.z();tel.ax=h.xacc;tel.ay=-h.yacc;tel.az=-h.zacc;}}
        else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU){mavlink_scaled_imu_t h{};mavlink_msg_scaled_imu_decode(&msg,&h);log.latest[1].valid=true;log.latest[1].us=(uint64_t)h.time_boot_ms*1000ULL;log.latest[1].w=frdGyroToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);}
        else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU2){mavlink_scaled_imu2_t h{};mavlink_msg_scaled_imu2_decode(&msg,&h);log.latest[2].valid=true;log.latest[2].us=(uint64_t)h.time_boot_ms*1000ULL;log.latest[2].w=frdGyroToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);}
        else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU3){mavlink_scaled_imu3_t h{};mavlink_msg_scaled_imu3_decode(&msg,&h);log.latest[3].valid=true;log.latest[3].us=(uint64_t)h.time_boot_ms*1000ULL;log.latest[3].w=frdGyroToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);uint64_t t=(uint64_t)a.time_boot_ms*1000ULL;if(t==last_att_us)continue;last_att_us=t;double yaw=a.yaw*180.0/kPi;Eigen::Vector3d fc=frdGyroToFlu(a.rollspeed,a.pitchspeed,a.yawspeed);{std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(yaw-tel.prev_yaw);tel.prev_yaw=yaw;tel.have_prev_yaw=true;tel.fc_roll=a.roll*180.0/kPi;tel.fc_pitch=a.pitch*180.0/kPi;tel.fc_yaw=yaw;tel.fc_valid=true;}
          if(ready){log.phase=phase;appendRow32(log,t,yaw,fc);double yy,gn,an;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets32[phase])<=kYawTol32&&gn<=kGyroStill32&&std::abs(an-9.81)<=kAccTol32;if(ok)++stable;else stable=0;if(stable>=kStable32){std::cout<<"[STEP] "<<kNames32[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets32[phase]<<" deg.\n";}}}
      }}}
      if(monotonicNs()>=next_hud){hud32(tel,log,ready,done,phase,stable,ref);next_hud=monotonicNs()+33333333LL;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){bool fv=false;{std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv)ref=tel.fc_accum_yaw;}if(fv){ready=true;log.ready=true;std::cout<<"[ZERO] start. Target +30 deg.\n";}}else if(key==' '&&done)break;
    }
    save32(log);summary32(log);std::cout<<"\nCSV: "<<kCsv32<<"\nOpen CSV:\n  code "<<kCsv32<<"\n";
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,0);
    if(sfd>=0)close(sfd);cv::destroyAllWindows();return aborted?2:0;
  }catch(const std::exception&e){if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
