// JT-ZERO v39: A->B->A сравнение двух IMU ArduPilot.
// YAW ДРОНА/СТЕНДА: примерно -90°.
// Логирует HIGHRES_IMU(id), SCALED_IMU, SCALED_IMU2, SCALED_IMU3 и ATTITUDE.
// Kimera не используется. Все операторские строки — на русском.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>

namespace jtzero_v39 {

constexpr const char* kCsv39="/home/vio/jtzero_dual_imu_translation_v39.csv";
constexpr const char* kWindow39="JT-ZERO: A-B-A СРАВНЕНИЕ ДВУХ IMU v39";

struct Att39 { bool valid=false; double roll=0,pitch=0,yaw=0; };
struct Src39 {
  bool valid=false;
  uint64_t t_us=0;
  int id=-1;
  Eigen::Vector3d acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
};
struct Row39 {
  int phase=0;
  int64_t wall_ns=0;
  Att39 att;
  std::array<Src39,4> src;
};

static const char* srcName39(int i){
  static const char* n[4]={"HIGHRES_IMU","SCALED_IMU","SCALED_IMU2","SCALED_IMU3"};
  return n[i];
}
static const char* phaseName39(int p){
  switch(p){
    case 0:return "ПОКОЙ_A_СТАРТ";
    case 1:return "ДВИЖЕНИЕ_A_В_B";
    case 2:return "ПОКОЙ_B";
    case 3:return "ДВИЖЕНИЕ_B_В_A";
    case 4:return "ПОКОЙ_A_ФИНИШ";
    default:return "ЗАВЕРШЕНО";
  }
}
static Eigen::Vector3d frdToFlu39(double x,double y,double z){
  return Eigen::Vector3d(x,-y,-z);
}
static bool fresh39(const Src39&s,uint64_t ref){
  if(!s.valid) return false;
  uint64_t d=ref>s.t_us?ref-s.t_us:s.t_us-ref;
  return d<=50000ULL;
}
static void save39(const std::vector<Row39>& rows){
  std::ofstream f(kCsv39,std::ios::trunc);
  f<<std::fixed<<std::setprecision(9);
  f<<"phase,phase_name,wall_ns,fc_roll_deg,fc_pitch_deg,fc_yaw_deg";
  for(int i=0;i<4;++i){
    f<<','<<srcName39(i)<<"_valid"
     <<','<<srcName39(i)<<"_id"
     <<','<<srcName39(i)<<"_t_us"
     <<','<<srcName39(i)<<"_ax_flu"
     <<','<<srcName39(i)<<"_ay_flu"
     <<','<<srcName39(i)<<"_az_flu"
     <<','<<srcName39(i)<<"_gx_flu"
     <<','<<srcName39(i)<<"_gy_flu"
     <<','<<srcName39(i)<<"_gz_flu";
  }
  f<<"\n";
  for(const auto&r:rows){
    f<<r.phase<<','<<phaseName39(r.phase)<<','<<r.wall_ns<<','
     <<r.att.roll<<','<<r.att.pitch<<','<<r.att.yaw;
    for(int i=0;i<4;++i){
      const auto&s=r.src[i];
      f<<','<<(s.valid?1:0)<<','<<s.id<<','<<s.t_us
       <<','<<s.acc.x()<<','<<s.acc.y()<<','<<s.acc.z()
       <<','<<s.gyro.x()<<','<<s.gyro.y()<<','<<s.gyro.z();
    }
    f<<"\n";
  }
}
static void hud39(const Att39&a,const std::array<Src39,4>&s,int phase,bool recording){
  cv::Mat c(900,1280,CV_8UC3,cv::Scalar(15,15,15));
  cv::Scalar w(235,235,235),gr(80,220,80),ye(0,220,255),red(70,70,255),muted(150,150,150);
  uiText(c,"JT-ZERO: A-B-A СРАВНЕНИЕ ДВУХ IMU v39",28,50,.72,w,2);
  uiText(c,"YAW ДРОНА/СТЕНДА: примерно -90°",40,105,.60,ye,2);
  char b[256];
  snprintf(b,sizeof(b),"ЭТАП %d/5: %s",std::min(phase+1,5),phaseName39(phase));
  uiText(c,b,40,165,.58,recording?gr:ye,2);

  if(phase==0) uiText(c,"Точка A. SPACE — начать запись покоя минимум 3 с.",40,220,.48,w,1);
  if(phase==1) uiText(c,"Переместите A -> B ровно 500 мм. SPACE — после полной остановки в B.",40,220,.48,w,1);
  if(phase==2) uiText(c,"Точка B. SPACE — после минимум 3 с покоя начать B -> A.",40,220,.48,w,1);
  if(phase==3) uiText(c,"Переместите B -> A ровно 500 мм. SPACE — после полной остановки в A.",40,220,.48,w,1);
  if(phase==4) uiText(c,"Точка A. SPACE — завершить после минимум 3 с покоя.",40,220,.48,w,1);

  snprintf(b,sizeof(b),"FC R/P/Y = %+.3f  %+.3f  %+.3f град",a.roll,a.pitch,a.yaw);
  uiText(c,b,40,280,.46,w,1);

  int y=340;
  for(int i=0;i<4;++i){
    snprintf(b,sizeof(b),"%s  %s  id=%d  |a|=%.4f  |g|=%.5f",
             srcName39(i),s[i].valid?"ЕСТЬ":"НЕТ",s[i].id,s[i].acc.norm(),s[i].gyro.norm());
    uiText(c,b,50,y,.43,s[i].valid?gr:red,1);
    y+=48;
  }
  uiText(c,"НЕ МЕНЯЙТЕ yaw, высоту и конфигурацию стенда.",40,590,.48,red,2);
  uiText(c,"Новые метки/ArUco не нужны. Видеозапись для этого теста не обязательна.",40,650,.44,w,1);
  uiText(c,"ESC / Q — прервать",40,850,.42,muted,1);
  cv::imshow(kWindow39,c);
}

} // namespace jtzero_v39

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  int fd=-1; uint8_t sys=0,comp=0; mavlink_status_t mst{}; mavlink_message_t msg{};
  jtzero_v39::Att39 att;
  std::array<jtzero_v39::Src39,4> src;
  std::vector<jtzero_v39::Row39> rows;
  int phase=0; bool recording=false; int64_t phase_start=0;
  uint64_t last_highres=0;

  try{
    fd=openSerial(); std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0)continue;
      uint8_t b[2048]; ssize_t n=read(fd,b,sizeof(b)); if(n<=0)continue;
      for(ssize_t i=0;i<n;++i)
        if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
          sys=msg.sysid;comp=msg.compid;break;
        }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,100);

    cv::namedWindow(jtzero_v39::kWindow39,cv::WINDOW_NORMAL);
    cv::resizeWindow(jtzero_v39::kWindow39,1280,900);

    while(true){
      pollfd p{fd,POLLIN,0}; poll(&p,1,5);
      if(p.revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(fd,b,sizeof(b));
          if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
          if(n<=0) break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)) continue;

            if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{}; mavlink_msg_attitude_decode(&msg,&a);
              att.valid=true; att.roll=a.roll*180.0/kPi; att.pitch=a.pitch*180.0/kPi; att.yaw=a.yaw*180.0/kPi;
            }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t h{}; mavlink_msg_highres_imu_decode(&msg,&h);
              if(h.time_usec==last_highres) continue; last_highres=h.time_usec;
              src[0].valid=true; src[0].t_us=h.time_usec; src[0].id=h.id;
              src[0].acc=jtzero_v39::frdToFlu39(h.xacc,h.yacc,h.zacc);
              src[0].gyro=jtzero_v39::frdToFlu39(h.xgyro,h.ygyro,h.zgyro);

              if(recording&&att.valid){
                jtzero_v39::Row39 r; r.phase=phase; r.wall_ns=monotonicNs(); r.att=att;
                r.src=src;
                for(int j=1;j<4;++j) if(!jtzero_v39::fresh39(r.src[j],h.time_usec)) r.src[j].valid=false;
                rows.push_back(r);
              }
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU){
              mavlink_scaled_imu_t h{}; mavlink_msg_scaled_imu_decode(&msg,&h);
              src[1].valid=true; src[1].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[1].id=0;
              src[1].acc=jtzero_v39::frdToFlu39(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[1].gyro=jtzero_v39::frdToFlu39(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU2){
              mavlink_scaled_imu2_t h{}; mavlink_msg_scaled_imu2_decode(&msg,&h);
              src[2].valid=true; src[2].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[2].id=1;
              src[2].acc=jtzero_v39::frdToFlu39(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[2].gyro=jtzero_v39::frdToFlu39(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU3){
              mavlink_scaled_imu3_t h{}; mavlink_msg_scaled_imu3_decode(&msg,&h);
              src[3].valid=true; src[3].t_us=(uint64_t)h.time_boot_ms*1000ULL; src[3].id=2;
              src[3].acc=jtzero_v39::frdToFlu39(h.xacc*9.80665/1000.0,h.yacc*9.80665/1000.0,h.zacc*9.80665/1000.0);
              src[3].gyro=jtzero_v39::frdToFlu39(h.xgyro*0.001,h.ygyro*0.001,h.zgyro*0.001);
            }
          }
        }
      }

      jtzero_v39::hud39(att,src,phase,recording);
      int k=cv::waitKey(1)&255;
      if(k==27||k=='q'||k=='Q') break;
      if(k==' '&&att.valid){
        if(!recording){
          recording=true; phase_start=monotonicNs();
          std::cout<<"[ЭТАП НАЧАТ] "<<jtzero_v39::phaseName39(phase)<<"\n";
        }else{
          double sec=(monotonicNs()-phase_start)/1e9;
          if((phase==0||phase==2||phase==4)&&sec<3.0){
            std::cout<<"[ЖДИТЕ] покой должен длиться минимум 3.0 с; сейчас="<<sec<<"\n";
            continue;
          }
          std::cout<<"[ЭТАП ЗАВЕРШЁН] "<<jtzero_v39::phaseName39(phase)<<" duration="<<sec<<" s\n";
          recording=false; ++phase;
          if(phase>=5){std::cout<<"[ТЕСТ] ЗАВЕРШЁН\n";break;}
          if(phase==1||phase==3){
            recording=true; phase_start=monotonicNs();
            std::cout<<"[ЭТАП НАЧАТ] "<<jtzero_v39::phaseName39(phase)<<"\n";
          }
        }
      }
    }

    jtzero_v39::save39(rows);
    std::cout<<"CSV: "<<jtzero_v39::kCsv39<<"\n";

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,0);
    close(fd); cv::destroyAllWindows(); return 0;
  }catch(const std::exception&e){
    if(fd>=0)close(fd); cv::destroyAllWindows();
    std::cerr<<"[FATAL] "<<e.what()<<"\n"; return 1;
  }
}
