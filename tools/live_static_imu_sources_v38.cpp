// JT-ZERO v38: статическое сравнение IMU-источников ArduPilot.
// Никаких движений стенда. Логирует HIGHRES_IMU (включая id), SCALED_IMU1/2/3 и ATTITUDE.
// Все операторские строки — на русском.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>

namespace jtzero_v38 {
using namespace jtzero_v10;

constexpr const char* kCsv = "/home/vio/jtzero_static_imu_sources_v38.csv";
constexpr const char* kWindow = "JT-ZERO: СТАТИЧЕСКОЕ СРАВНЕНИЕ IMU v38";
constexpr double kDurationSec = 30.0;

struct Src {
  bool valid=false;
  uint64_t t_us=0;
  int id=-1;
  Eigen::Vector3d acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
};

struct Att {
  bool valid=false;
  uint64_t t_us=0;
  double roll=0,pitch=0,yaw=0;
};

struct Row {
  uint64_t wall_ns=0;
  double elapsed=0;
  Att att;
  std::array<Src,4> s;
};

static const char* srcName(int i){
  static const char* n[4]={"HIGHRES_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3"};
  return n[i];
}

static Eigen::Vector3d frdToFlu(double x,double y,double z){
  return Eigen::Vector3d(x,-y,-z);
}

static bool fresh(const Src& s,uint64_t ref_us){
  if(!s.valid) return false;
  uint64_t d = ref_us>s.t_us ? ref_us-s.t_us : s.t_us-ref_us;
  return d <= 50000ULL;
}

static void save(const std::vector<Row>& rows){
  std::ofstream f(kCsv,std::ios::trunc);
  f<<std::fixed<<std::setprecision(9);
  f<<"wall_ns,elapsed_s,att_valid,att_roll_deg,att_pitch_deg,att_yaw_deg";
  for(int i=0;i<4;++i){
    f<<','<<srcName(i)<<"_valid"
     <<','<<srcName(i)<<"_id"
     <<','<<srcName(i)<<"_ax_flu"
     <<','<<srcName(i)<<"_ay_flu"
     <<','<<srcName(i)<<"_az_flu"
     <<','<<srcName(i)<<"_gx_flu"
     <<','<<srcName(i)<<"_gy_flu"
     <<','<<srcName(i)<<"_gz_flu";
  }
  f<<"\n";
  for(const auto&r:rows){
    f<<r.wall_ns<<','<<r.elapsed<<','<<(r.att.valid?1:0)<<','
     <<r.att.roll<<','<<r.att.pitch<<','<<r.att.yaw;
    for(int i=0;i<4;++i){
      const auto&s=r.s[i];
      f<<','<<(s.valid?1:0)<<','<<s.id
       <<','<<s.acc.x()<<','<<s.acc.y()<<','<<s.acc.z()
       <<','<<s.gyro.x()<<','<<s.gyro.y()<<','<<s.gyro.z();
    }
    f<<"\n";
  }
}

static void hud(const std::array<Src,4>& src,const Att& att,double elapsed,bool recording){
  cv::Mat c(860,1280,CV_8UC3,cv::Scalar(15,15,15));
  cv::Scalar white(235,235,235),green(80,220,80),yellow(0,220,255),red(70,70,255),muted(150,150,150);
  uiText(c,"JT-ZERO: СТАТИЧЕСКОЕ СРАВНЕНИЕ IMU v38",28,52,.72,white,2);
  uiText(c,"СТЕНД НЕ ДВИГАТЬ. YAW НЕ МЕНЯТЬ.",40,112,.62,red,2);

  if(!recording){
    uiText(c,"SPACE — начать 30-секундную статическую запись",40,172,.58,yellow,2);
  }else{
    char b[128];
    snprintf(b,sizeof(b),"ИДЁТ ЗАПИСЬ: %.1f / %.1f с",elapsed,kDurationSec);
    uiText(c,b,40,172,.58,green,2);
  }

  char b[256];
  snprintf(b,sizeof(b),"FC ATTITUDE R/P/Y = %+.3f  %+.3f  %+.3f град",
           att.roll,att.pitch,att.yaw);
  uiText(c,b,40,238,.48,white,1);

  int y=310;
  for(int i=0;i<4;++i){
    const auto&s=src[i];
    snprintf(b,sizeof(b),"%s   %s   id=%d",srcName(i),s.valid?"ЕСТЬ":"НЕТ",s.id);
    uiText(c,b,40,y,.50,s.valid?green:red,2);
    y+=40;
    snprintf(b,sizeof(b),"  ACC FLU=[%+.4f %+.4f %+.4f] |a|=%.4f",
             s.acc.x(),s.acc.y(),s.acc.z(),s.acc.norm());
    uiText(c,b,55,y,.42,white,1);
    y+=34;
    snprintf(b,sizeof(b),"  GYRO FLU=[%+.5f %+.5f %+.5f] |g|=%.5f",
             s.gyro.x(),s.gyro.y(),s.gyro.z(),s.gyro.norm());
    uiText(c,b,55,y,.42,white,1);
    y+=55;
  }

  uiText(c,"ESC / Q — прервать",40,824,.42,muted,1);
  cv::imshow(kWindow,c);
}

} // namespace jtzero_v38

int main(int argc,char**argv){
  using namespace jtzero_v38;
  google::InitGoogleLogging(argv[0]);

  int fd=-1; uint8_t sys=0,comp=0;
  mavlink_status_t mst{}; mavlink_message_t msg{};
  std::array<Src,4> src; Att att;
  std::vector<Row> rows;
  bool recording=false;
  int64_t start_ns=0;
  uint64_t last_att_us=0;

  try{
    fd=openSerial();
    std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){
      pollfd p{fd,POLLIN,0};
      if(poll(&p,1,100)<=0) continue;
      uint8_t b[2048]; ssize_t n=read(fd,b,sizeof(b));
      if(n<=0) continue;
      for(ssize_t i=0;i<n;++i){
        if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst) && msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
          sys=msg.sysid; comp=msg.compid; break;
        }
      }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,100);

    cv::namedWindow(kWindow,cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindow,1280,860);

    while(true){
      pollfd p{fd,POLLIN,0}; poll(&p,1,5);
      if(p.revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(fd,b,sizeof(b));
          if(n==-1 && (errno==EAGAIN||errno==EWOULDBLOCK)) break;
          if(n<=0) break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)) continue;

            if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t h{}; mavlink_msg_highres_imu_decode(&msg,&h);
              src[0].valid=true; src[0].t_us=h.time_usec; src[0].id=h.id;
              src[0].acc=frdToFlu(h.xacc,h.yacc,h.zacc);
              src[0].gyro=frdToFlu(h.xgyro,h.ygyro,h.zgyro);
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU){
              mavlink_scaled_imu_t h{}; mavlink_msg_scaled_imu_decode(&msg,&h);
              src[1].valid=true; src[1].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[1].id=0;
              src[1].acc=frdToFlu(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[1].gyro=frdToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU2){
              mavlink_scaled_imu2_t h{}; mavlink_msg_scaled_imu2_decode(&msg,&h);
              src[2].valid=true; src[2].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[2].id=1;
              src[2].acc=frdToFlu(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[2].gyro=frdToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU3){
              mavlink_scaled_imu3_t h{}; mavlink_msg_scaled_imu3_decode(&msg,&h);
              src[3].valid=true; src[3].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[3].id=2;
              src[3].acc=frdToFlu(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[3].gyro=frdToFlu(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{}; mavlink_msg_attitude_decode(&msg,&a);
              uint64_t t=(uint64_t)a.time_boot_ms*1000ULL;
              if(t==last_att_us) continue; last_att_us=t;
              att.valid=true; att.t_us=t;
              att.roll=a.roll*180.0/kPi; att.pitch=a.pitch*180.0/kPi; att.yaw=a.yaw*180.0/kPi;

              if(recording){
                Row r; r.wall_ns=monotonicNs(); r.elapsed=(r.wall_ns-start_ns)/1e9; r.att=att;
                for(int j=0;j<4;++j){
                  r.s[j]=src[j];
                  if(!fresh(r.s[j],t)) r.s[j].valid=false;
                }
                rows.push_back(r);
              }
            }
          }
        }
      }

      double elapsed = recording ? (monotonicNs()-start_ns)/1e9 : 0.0;
      hud(src,att,elapsed,recording);
      int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q') break;
      if(key==' ' && !recording){
        rows.clear(); recording=true; start_ns=monotonicNs();
        std::cout<<"[ТЕСТ] Начата статическая запись на 30 секунд. Стенд не двигать.\n";
      }
      if(recording && elapsed>=kDurationSec){
        std::cout<<"[ТЕСТ] Статическая запись завершена.\n";
        break;
      }
    }

    save(rows);
    std::cout<<"CSV: "<<kCsv<<"\n";

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,0);
    close(fd); cv::destroyAllWindows();
    return 0;
  }catch(const std::exception&e){
    if(fd>=0) close(fd);
    cv::destroyAllWindows();
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    return 1;
  }
}
