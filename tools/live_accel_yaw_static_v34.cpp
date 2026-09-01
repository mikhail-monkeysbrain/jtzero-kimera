// JT-ZERO v34: static accelerometer residual vs yaw using HIGHRES_IMU + ATTITUDE.
// No Kimera pipeline is started. Russian GUI. Route: 0,+30,+60,+90,0,-30,-60,-90,0 deg.
// HIGHRES_IMU and ATTITUDE are converted FRD -> FLU consistently.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>

namespace jtzero_v34 {
using namespace jtzero_v10;

constexpr const char* kCsv = "/home/vio/jtzero_live_accel_yaw_static_v34.csv";
constexpr const char* kWindow = "JT-ZERO: СТАТИЧЕСКИЙ ACCEL/YAW v34";
constexpr int kPhases = 9;
constexpr double kTargets[kPhases] = {0,30,60,90,0,-30,-60,-90,0};
constexpr const char* kNames[kPhases] = {"0_start","+30","+60","+90","0_mid","-30","-60","-90","0_final"};
constexpr double kYawTol = 1.5;
constexpr double kGyroStill = 0.035;
constexpr double kAccNormTol = 0.45;
constexpr int kStableSamples = 100;   // ~0.5 s at 200 Hz; event-driven, not a timer.
constexpr int kCollectSamples = 400;  // ~2 s at 200 Hz.

struct Att {
  bool valid=false;
  uint64_t us=0;
  double roll=0,pitch=0,yaw=0;
  Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
};

struct Row {
  int phase=0;
  uint64_t us=0;
  double rel_yaw=0;
  double roll=0,pitch=0,yaw=0;
  Eigen::Vector3d acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
  Eigen::Vector3d expected=Eigen::Vector3d::Zero();
  Eigen::Vector3d residual_body=Eigen::Vector3d::Zero();
};

struct Stats {
  int n=0;
  Eigen::Vector3d sum_acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_res=Eigen::Vector3d::Zero();
  Eigen::Vector3d ss_res=Eigen::Vector3d::Zero();
  double sum_roll=0,sum_pitch=0,sum_yaw=0;
  void add(const Row& r){
    ++n; sum_acc+=r.acc; sum_res+=r.residual_body;
    ss_res+=r.residual_body.cwiseProduct(r.residual_body);
    sum_roll+=r.roll;sum_pitch+=r.pitch;sum_yaw+=r.rel_yaw;
  }
  Eigen::Vector3d meanAcc() const {Eigen::Vector3d o=Eigen::Vector3d::Zero();if(n)o=sum_acc/double(n);return o;}
  Eigen::Vector3d meanRes() const {Eigen::Vector3d o=Eigen::Vector3d::Zero();if(n)o=sum_res/double(n);return o;}
  Eigen::Vector3d rmsRes() const {Eigen::Vector3d o=Eigen::Vector3d::Zero();if(n)o=(ss_res/double(n)).cwiseSqrt();return o;}
};

static double normDeg(double x){while(x>180)x-=360;while(x<-180)x+=360;return x;}

static void save(const std::vector<Row>& rows){
  std::ofstream f(kCsv,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,imu_us,rel_yaw_deg,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,ax,ay,az,gx,gy,gz,exp_ax,exp_ay,exp_az,res_bx,res_by,res_bz\n";
  for(const auto&r:rows){
    f<<r.phase<<','<<kNames[r.phase]<<','<<r.us<<','<<r.rel_yaw<<','<<r.roll<<','<<r.pitch<<','<<r.yaw<<','
     <<r.acc.x()<<','<<r.acc.y()<<','<<r.acc.z()<<','<<r.gyro.x()<<','<<r.gyro.y()<<','<<r.gyro.z()<<','
     <<r.expected.x()<<','<<r.expected.y()<<','<<r.expected.z()<<','
     <<r.residual_body.x()<<','<<r.residual_body.y()<<','<<r.residual_body.z()<<'\n';
  }
}

static void printSummary(const std::array<Stats,kPhases>& st){
  std::cout<<std::fixed<<std::setprecision(9);
  Eigen::Vector3d b0=st[0].meanRes();
  std::cout<<"\n============================================================\nJT-ZERO STATIC ACCEL/YAW v34 RESULT\n============================================================\n";
  std::cout<<"baseline residual @0 = ["<<b0.transpose()<<"] m/s^2\n";
  double max_world_xy=0,max_body_delta=0;
  for(int p=0;p<kPhases;++p){
    Eigen::Vector3d mr=st[p].meanRes();
    Eigen::Vector3d dr=mr-b0;
    double rr=st[p].n?st[p].sum_roll/st[p].n:0;
    double pp=st[p].n?st[p].sum_pitch/st[p].n:0;
    double yy=st[p].n?st[p].sum_yaw/st[p].n:0;
    Eigen::Matrix3d R=fcRnedFlu(rr,pp,yy);
    Eigen::Vector3d world=R*dr;
    max_world_xy=std::max(max_world_xy,std::hypot(world.x(),world.y()));
    max_body_delta=std::max(max_body_delta,std::hypot(dr.x(),dr.y()));
    std::cout<<"\n"<<kNames[p]<<" target="<<kTargets[p]<<" deg n="<<st[p].n
      <<" mean R/P/Yrel=["<<rr<<","<<pp<<","<<yy<<"] deg\n"
      <<"  mean acc=["<<st[p].meanAcc().transpose()<<"] m/s^2\n"
      <<"  mean raw residual body=["<<mr.transpose()<<"] m/s^2\n"
      <<"  RMS raw residual body=["<<st[p].rmsRes().transpose()<<"] m/s^2\n"
      <<"  delta vs 0 body=["<<dr.transpose()<<"] |xy|="<<std::hypot(dr.x(),dr.y())<<" m/s^2\n"
      <<"  delta vs 0 world=["<<world.transpose()<<"] |xy|="<<std::hypot(world.x(),world.y())<<" m/s^2\n";
  }
  Eigen::Vector3d close=st[kPhases-1].meanRes()-b0;
  std::cout<<"\nDecision helper:\n";
  std::cout<<"  max yaw-dependent body XY delta="<<max_body_delta<<" m/s^2\n";
  std::cout<<"  max yaw-dependent world XY delta="<<max_world_xy<<" m/s^2\n";
  std::cout<<"  final 0 return body delta=["<<close.transpose()<<"] |xy|="<<std::hypot(close.x(),close.y())<<" m/s^2\n";
  if(max_body_delta<0.05)
    std::cout<<"  RESULT: residual is nearly yaw-invariant in body frame; constant accel bias is the leading model.\n";
  else if(std::hypot(close.x(),close.y())>0.08)
    std::cout<<"  RESULT: poor return-to-zero repeatability. Check stand tilt/cable force/vibration/thermal drift before calibration fitting.\n";
  else
    std::cout<<"  RESULT: repeatable yaw-dependent accel residual exists. Fit accel axis misalignment/scale/cross-axis model next.\n";
}

static void hud(const Att& at,double relYaw,const Eigen::Vector3d& acc,const Eigen::Vector3d& gyro,
                bool started,bool done,int phase,int stable,int collected){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(850,1300,CV_8UC3,bg);
  uiText(c,"JT-ZERO: СТАТИЧЕСКАЯ ПРОВЕРКА ACCEL ПО YAW v34",30,52,.68,white,2);
  if(!started){uiText(c,"УСТАНОВИТЕ СТЕНД В ИСХОДНЫЙ 0°",70,125,.65,yellow,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И НАЧАТЬ",70,180,.58,white,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",70,125,.82,green,3);uiText(c,"SPACE — ВЫХОД",70,180,.55,white,2);}
  else{
    char t[160];snprintf(t,sizeof(t),"ЭТАП %d/%d: ПОВЕРНИТЕ YAW К %+.0f°",phase+1,kPhases,kTargets[phase]);uiText(c,t,70,120,.65,yellow,2);
    if(stable<kStableSamples)uiText(c,"ПОПАДИТЕ В ЗЕЛЁНУЮ ЗОНУ И ОСТАНОВИТЕСЬ",70,175,.52,white,2);
    else uiText(c,"СТОП! СТЕНД НЕ ДВИГАТЬ — ИДЁТ СБОР",70,175,.58,green,2);
  }
  cv::Mat s=c(cv::Rect(50,235,1200,540));double target=done?0:kTargets[std::min(phase,kPhases-1)];drawBar(s,"YAW",relYaw,-105,105,target,kYawTol,65);
  char b[256];
  snprintf(b,sizeof(b),"YAW: %+.2f°   ROLL: %+.2f°   PITCH: %+.2f°",relYaw,at.roll,at.pitch);uiText(s,b,20,135,.48,white,1);
  snprintf(b,sizeof(b),"|gyro| %.4f rad/s   |acc| %.4f m/s²",gyro.norm(),acc.norm());uiText(s,b,20,185,.48,white,1);
  if(started&&!done){
    snprintf(b,sizeof(b),"Стабилизация: %d/%d",stable,kStableSamples);uiText(s,b,20,255,.45,stable>=kStableSamples?green:white,1);
    snprintf(b,sizeof(b),"Собрано: %d/%d",collected,kCollectSamples);uiText(s,b,20,305,.48,collected>=kCollectSamples?green:white,2);
    int x=20,y=350,w=1120,h=30;cv::rectangle(s,{x,y,w,h},cv::Scalar(80,80,80),1);double q=std::min(1.0,collected/double(kCollectSamples));cv::rectangle(s,{x+2,y+2,int((w-4)*q),h-4},green,-1);
  }
  uiText(s,"Маршрут: 0 → +30 → +60 → +90 → 0 → -30 → -60 → -90 → 0",20,455,.40,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",50,825,.42,muted,1);
  cv::imshow(kWindow,c);
}

} // namespace jtzero_v34

int main(int argc,char**argv){
  using namespace jtzero_v34;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);
  int sfd=-1;uint8_t sys=0,comp=0;mavlink_status_t mst{};mavlink_message_t msg{};
  bool started=false,done=false,aborted=false;int phase=0,stable=0,collected=0;double refYaw=0;bool haveRef=false;
  Att at;Eigen::Vector3d acc=Eigen::Vector3d::Zero(),gyro=Eigen::Vector3d::Zero();
  std::array<Stats,kPhases> stats;std::vector<Row> rows;uint64_t lastImuUs=0;
  try{
    sfd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
    cv::namedWindow(kWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow,1300,850);int64_t nextHud=0;
    std::cout<<"\nJT-ZERO STATIC ACCEL/YAW v34\nSPACE starts. Route 0,+30,+60,+90,0,-30,-60,-90,0.\n";
    while(true){
      pollfd p{sfd,POLLIN,0};poll(&p,1,5);
      if(p.revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
        if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);at.valid=true;at.us=(uint64_t)a.time_boot_ms*1000ULL;at.roll=a.roll*180.0/kPi;at.pitch=a.pitch*180.0/kPi;at.yaw=a.yaw*180.0/kPi;at.R=fcRnedFlu(at.roll,at.pitch,at.yaw);}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(h.time_usec==lastImuUs)continue;lastImuUs=h.time_usec;acc=Eigen::Vector3d(h.xacc,-h.yacc,-h.zacc);gyro=Eigen::Vector3d(h.xgyro,-h.ygyro,-h.zgyro);
          if(started&&!done&&at.valid&&haveRef){double ry=normDeg(at.yaw-refYaw);bool targetOk=std::abs(normDeg(ry-kTargets[phase]))<=kYawTol;bool still=gyro.norm()<=kGyroStill&&std::abs(acc.norm()-9.81)<=kAccNormTol;
            if(targetOk&&still){if(stable<kStableSamples)++stable;else if(collected<kCollectSamples){
                const Eigen::Vector3d gW(0,0,9.81);Eigen::Vector3d expected=-at.R.transpose()*gW;
                Row r;r.phase=phase;r.us=h.time_usec;r.rel_yaw=ry;r.roll=at.roll;r.pitch=at.pitch;r.yaw=at.yaw;r.acc=acc;r.gyro=gyro;r.expected=expected;r.residual_body=acc-expected;rows.push_back(r);stats[phase].add(r);++collected;
                if(collected>=kCollectSamples){std::cout<<"[STEP] "<<kNames[phase]<<" collected.\n";stable=0;collected=0;if(++phase>=kPhases){phase=kPhases-1;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets[phase]<<" deg.\n";}
              }}else{stable=0;collected=0;}
          }
        }
      }}}
      if(monotonicNs()>=nextHud){double rel=haveRef&&at.valid?normDeg(at.yaw-refYaw):0;hud(at,rel,acc,gyro,started,done,phase,stable,collected);nextHud=monotonicNs()+33333333LL;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!started){if(at.valid){refYaw=at.yaw;haveRef=true;started=true;phase=0;stable=0;collected=0;std::cout<<"[ZERO] reference yaw fixed. Collecting initial 0 deg.\n";}}
      else if(key==' '&&done)break;
    }
    save(rows);printSummary(stats);std::cout<<"\nCSV: "<<kCsv<<"\nOpen CSV:\n  code "<<kCsv<<"\n";
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(sfd>=0)close(sfd);cv::destroyAllWindows();return aborted?2:0;
  }catch(const std::exception&e){if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
