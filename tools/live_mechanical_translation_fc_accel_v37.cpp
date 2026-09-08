// JT-ZERO v37: внешний механический эталонный тест перемещения на 500 мм.
// Kimera не используется. Логируются FC ATTITUDE + HIGHRES_IMU параллельно с боковой видеосъёмкой.
// Обязательная геометрия: yaw стенда/дрона примерно -90°.
// Последовательность: покой A -> A->B -> покой B -> B->A -> покой A.
#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN
#include <fstream>
#include <iomanip>

namespace jtzero_v37 {
constexpr const char* kCsv="/home/vio/jtzero_mechanical_translation_v37.csv";
constexpr const char* kWindow="JT-ZERO: МЕХАНИЧЕСКИЙ ТЕСТ ПЕРЕМЕЩЕНИЯ v37";

struct Att { bool valid=false; double roll=0,pitch=0,yaw=0; };
struct Row {
  uint64_t imu_us=0; int64_t wall_ns=0; int phase=0;
  double roll=0,pitch=0,yaw=0, ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
};
static const char* phaseName(int p){
  switch(p){
    case 0:return "ПОКОЙ_A_СТАРТ";
    case 1:return "ДВИЖЕНИЕ_A_В_B";
    case 2:return "ПОКОЙ_B";
    case 3:return "ДВИЖЕНИЕ_B_В_A";
    case 4:return "ПОКОЙ_A_ФИНИШ";
    default:return "ЗАВЕРШЕНО";
  }
}
static void save(const std::vector<Row>& rows){
  std::ofstream f(kCsv);
  f<<std::fixed<<std::setprecision(9)
   <<"phase,phase_name,imu_us,wall_ns,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,ax_flu,ay_flu,az_flu,gx_flu,gy_flu,gz_flu\n";
  for(const auto&r:rows)
    f<<r.phase<<','<<phaseName(r.phase)<<','<<r.imu_us<<','<<r.wall_ns<<','
     <<r.roll<<','<<r.pitch<<','<<r.yaw<<','<<r.ax<<','<<r.ay<<','<<r.az<<','
     <<r.gx<<','<<r.gy<<','<<r.gz<<'\n';
}
static void hud(const Att&a,const Eigen::Vector3d&acc,const Eigen::Vector3d&gyro,int phase,bool recording){
  cv::Mat c(820,1280,CV_8UC3,cv::Scalar(15,15,15));
  cv::Scalar w(235,235,235),gr(80,220,80),ye(0,220,255),red(80,80,255);
  uiText(c,"JT-ZERO: МЕХАНИЧЕСКИЙ ТЕСТ ПЕРЕМЕЩЕНИЯ v37",30,55,.72,w,2);
  uiText(c,"YAW ДРОНА/СТЕНДА: примерно -90°",45,115,.62,ye,2);
  uiText(c,"БОКОВОЕ ВИДЕО: телефон неподвижен; в кадре МЕТКА FC и ЖЁСТКАЯ ВЕРХНЯЯ ПЛОЩАДКА",45,165,.50,w,1);
  char t[256];
  snprintf(t,sizeof(t),"ЭТАП %d/5: %s",std::min(phase+1,5),phaseName(phase));
  uiText(c,t,45,235,.62,recording?gr:ye,2);
  if(phase==0) uiText(c,"Удерживайте стенд в точке A. SPACE — начать 3-секундную запись покоя.",45,295,.50,w,1);
  if(phase==1) uiText(c,"Переместите A -> B ровно на 500 мм. SPACE — когда стенд полностью остановлен в B.",45,295,.50,w,1);
  if(phase==2) uiText(c,"Удерживайте стенд в точке B. SPACE — начать движение B -> A.",45,295,.50,w,1);
  if(phase==3) uiText(c,"Переместите B -> A ровно на 500 мм. SPACE — когда стенд полностью остановлен в A.",45,295,.50,w,1);
  if(phase==4) uiText(c,"Удерживайте стенд в точке A. SPACE — завершить после минимум 3 секунд покоя.",45,295,.50,w,1);
  snprintf(t,sizeof(t),"FC R/P/Y = %+.3f  %+.3f  %+.3f град",a.roll,a.pitch,a.yaw);
  uiText(c,t,45,390,.50,w,1);
  snprintf(t,sizeof(t),"ACC FLU = [%+.4f %+.4f %+.4f] |a|=%.4f",acc.x(),acc.y(),acc.z(),acc.norm());
  uiText(c,t,45,445,.46,w,1);
  snprintf(t,sizeof(t),"GYRO FLU = [%+.5f %+.5f %+.5f] |g|=%.5f",gyro.x(),gyro.y(),gyro.z(),gyro.norm());
  uiText(c,t,45,500,.46,w,1);
  uiText(c,"НЕ МЕНЯЙТЕ yaw, высоту и конфигурацию стенда во время всего теста.",45,600,.48,red,1);
  uiText(c,"НЕ ПРЕРЫВАЙТЕ запись видео на телефоне до сообщения «ТЕСТ ЗАВЕРШЁН».",45,655,.48,red,1);
  cv::imshow(kWindow,c);
}
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  int fd=-1; uint8_t sys=0,comp=0; mavlink_status_t mst{}; mavlink_message_t msg{};
  jtzero_v37::Att at; Eigen::Vector3d acc=Eigen::Vector3d::Zero(),gyro=Eigen::Vector3d::Zero();
  std::vector<jtzero_v37::Row> rows; int phase=0; bool recording=false; int64_t phase_start=0; uint64_t last=0;
  try{
    fd=openSerial(); std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0)continue;
      uint8_t b[2048]; ssize_t n=read(fd,b,sizeof(b)); if(n<=0)continue;
      for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
    cv::namedWindow(jtzero_v37::kWindow);

    while(true){
      pollfd p{fd,POLLIN,0}; poll(&p,1,5);
      if(p.revents&POLLIN){
        uint8_t b[8192]; ssize_t n=read(fd,b,sizeof(b));
        if(n>0) for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)){
          if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
            mavlink_attitude_t a{}; mavlink_msg_attitude_decode(&msg,&a);
            at.valid=true; at.roll=a.roll*180/kPi; at.pitch=a.pitch*180/kPi; at.yaw=a.yaw*180/kPi;
          } else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
            mavlink_highres_imu_t h{}; mavlink_msg_highres_imu_decode(&msg,&h);
            if(h.time_usec==last) continue; last=h.time_usec;
            acc={h.xacc,-h.yacc,-h.zacc}; gyro={h.xgyro,-h.ygyro,-h.zgyro};
            if(recording && at.valid){
              jtzero_v37::Row r; r.imu_us=h.time_usec; r.wall_ns=monotonicNs(); r.phase=phase;
              r.roll=at.roll; r.pitch=at.pitch; r.yaw=at.yaw;
              r.ax=acc.x();r.ay=acc.y();r.az=acc.z();r.gx=gyro.x();r.gy=gyro.y();r.gz=gyro.z();
              rows.push_back(r);
            }
          }
        }
      }
      jtzero_v37::hud(at,acc,gyro,phase,recording);
      int k=cv::waitKey(1)&255;
      if(k==27||k=='q'||k=='Q') break;
      if(k==' ' && at.valid){
        if(!recording){
          recording=true; phase_start=monotonicNs();
          std::cout<<"[ЭТАП НАЧАТ] "<<jtzero_v37::phaseName(phase)<<" wall_ns="<<phase_start<<"\n";
        }else{
          const double sec=(monotonicNs()-phase_start)/1e9;
          if((phase==0||phase==2||phase==4) && sec<3.0){
            std::cout<<"[ЖДИТЕ] фаза покоя должна длиться минимум 3.0 с; сейчас="<<sec<<"\n";
            continue;
          }
          std::cout<<"[ЭТАП ЗАВЕРШЁН] "<<jtzero_v37::phaseName(phase)<<" duration="<<sec<<" s\n";
          recording=false; phase++;
          if(phase>=5){ std::cout<<"[ТЕСТ] ЗАВЕРШЁН\n"; break; }
          // movement phases should start immediately after operator confirmation
          if(phase==1||phase==3){
            recording=true; phase_start=monotonicNs();
            std::cout<<"[ЭТАП НАЧАТ] "<<jtzero_v37::phaseName(phase)<<" wall_ns="<<phase_start<<"\n";
          }
        }
      }
    }
    jtzero_v37::save(rows);
    std::cout<<"CSV: "<<jtzero_v37::kCsv<<"\n";
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    close(fd); cv::destroyAllWindows(); return 0;
  }catch(const std::exception&e){
    if(fd>=0)close(fd); cv::destroyAllWindows(); std::cerr<<"[FATAL] "<<e.what()<<"\n"; return 1;
  }
}
