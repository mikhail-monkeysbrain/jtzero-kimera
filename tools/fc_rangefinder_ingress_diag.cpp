// JT-Zero — короткая проверка пути DISTANCE_SENSOR -> AP_RangeFinder frontend.
// Камера и TF-Luna не используются. Параметры FC не меняются.
// 8 секунд отправляет synthetic downward DISTANCE_SENSOR:
//   0..4 s: 0.60 m
//   4..8 s: 0.70 m
// и слушает DISTANCE_SENSOR, который уже сам FC публикует из RangeFinder backend.
// Если FC echo повторяет ступень 0.60 -> 0.70, AP_RangeFinder_MAVLink вход доказан.

#include "ardupilotmega/mavlink.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace {
using Clock=std::chrono::steady_clock;
constexpr uint8_t kSelfSys=191;
constexpr uint8_t kSelfComp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

double elapsed(const Clock::time_point&t0){return std::chrono::duration<double>(Clock::now()-t0).count();}

int openSerial(const std::string&dev){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK); if(fd<0)return -1;
  termios t{}; if(tcgetattr(fd,&t)<0){::close(fd);return -1;} cfmakeraw(&t);
  cfsetispeed(&t,B460800); cfsetospeed(&t,B460800); t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB; t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;} tcflush(fd,TCIFLUSH); return fd;
}
bool writeMsg(int fd,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; const uint16_t n=mavlink_msg_to_send_buffer(b,&m); size_t o=0;
  while(o<n){ssize_t k=::write(fd,b+o,n-o); if(k>0){o+=(size_t)k;continue;}
    if(k<0&&errno==EINTR)continue;
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd p{fd,POLLOUT,0};(void)::poll(&p,1,10);continue;}
    return false;}
  return true;
}
void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
  mavlink_message_t m{}; mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
    MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)msgid,1000000.0f/hz,0,0,0,0,0); (void)writeMsg(fd,m);
}
void requestParam(int fd,uint8_t sys,uint8_t comp,const char*name){
  char id[16]{}; std::memcpy(id,name,std::min<size_t>(std::strlen(name),sizeof(id)));
  mavlink_message_t m{}; mavlink_msg_param_request_read_pack(kSelfSys,kSelfComp,&m,sys,comp,id,-1); (void)writeMsg(fd,m);
}
std::string pname(const mavlink_param_value_t&p){size_t n=0;while(n<16&&p.param_id[n])++n;return std::string(p.param_id,p.param_id+n);}

void sendDistance(int fd,double m){
  const uint16_t cm=(uint16_t)std::lround(m*100.0);
  float q[4]={0,0,0,0};
  mavlink_message_t msg{};
  mavlink_msg_distance_sensor_pack(
    kSelfSys,kSelfComp,&msg,0,
    10,800,cm,MAV_DISTANCE_SENSOR_LASER,
    0,MAV_SENSOR_ROTATION_PITCH_270,0,
    0.0f,0.0f,q,0);
  (void)writeMsg(fd,msg);
}
}

int main(int argc,char**argv){
  const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
  int fd=openSerial(dev); if(fd<0){std::cerr<<"ОШИБКА open "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}

  mavlink_status_t st{};mavlink_message_t msg{};uint8_t buf[4096];uint8_t sys=0,comp=0;
  const auto w0=Clock::now();
  while(elapsed(w0)<8&&!sys){
    pollfd p{fd,POLLIN,0};if(::poll(&p,1,100)<=0)continue;ssize_t n=::read(fd,buf,sizeof(buf));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);
      if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
    }
  }
  if(!sys){std::cerr<<"ОШИБКА: FC heartbeat не найден\n";::close(fd);return 3;}
  std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  const char* params[]={"RNGFND1_TYPE","RNGFND1_ORIENT","RNGFND1_GNDCLR","RNGFND1_MAX","EK3_SRC1_POSZ"};
  std::map<std::string,float> pv;
  for(auto n:params)requestParam(fd,sys,comp,n);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,10);
#ifdef MAVLINK_MSG_ID_RANGEFINDER
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_RANGEFINDER,10);
#endif

  uint64_t tx=0,fc_ds=0,fc_rf=0; double last_fc_m=NAN; int last_orient=-1,last_id=-1;
  const auto t0=Clock::now();auto next_tx=Clock::now();auto next_print=Clock::now()+std::chrono::seconds(1);
  while(elapsed(t0)<8.0){
    auto now=Clock::now(); const double t=elapsed(t0); const double target=(t<4.0)?0.60:0.70;
    if(now>=next_tx){sendDistance(fd,target);++tx;next_tx+=std::chrono::milliseconds(50);}
    pollfd p{fd,POLLIN,0};
    if(::poll(&p,1,5)>0){
      for(;;){ssize_t n=::read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)||msg.sysid!=sys)continue;
          if(msg.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){
            mavlink_distance_sensor_t q{};mavlink_msg_distance_sensor_decode(&msg,&q);
            ++fc_ds;last_fc_m=q.current_distance*0.01;last_orient=q.orientation;last_id=q.id;
          }
#ifdef MAVLINK_MSG_ID_RANGEFINDER
          else if(msg.msgid==MAVLINK_MSG_ID_RANGEFINDER){++fc_rf;}
#endif
          else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE){
            mavlink_param_value_t q{};mavlink_msg_param_value_decode(&msg,&q);const auto n=pname(q);
            for(auto w:params)if(n==w)pv[n]=q.param_value;
          }
        }
      }
    }
    if(now>=next_print){
      std::cout<<std::fixed<<std::setprecision(2)
        <<"t="<<t<<"s sent="<<target<<"m tx="<<tx
        <<" FC_DISTANCE_SENSOR="<<fc_ds;
      if(std::isfinite(last_fc_m))std::cout<<" fc_current="<<last_fc_m<<"m id="<<last_id<<" orient="<<last_orient;
      else std::cout<<" fc_current=NO_DATA";
      std::cout<<"\n";next_print+=std::chrono::seconds(1);
    }
  }

  std::cout<<"\n===== PARAMS =====\n";
  for(auto n:params){auto it=pv.find(n);std::cout<<n<<" = "<<(it==pv.end()?std::string("NO_RESPONSE"):std::to_string(it->second))<<"\n";}
  std::cout<<"\n===== VERDICT =====\n";
  std::cout<<"TX_DISTANCE_SENSOR="<<tx<<"\nFC_DISTANCE_SENSOR_ECHO="<<fc_ds<<"\n";
  if(fc_ds>0 && std::isfinite(last_fc_m) && std::fabs(last_fc_m-0.70)<0.03){
    std::cout<<"AP_RANGEFINDER_MAVLINK_INGRESS=PROVEN\n";
    std::cout<<"FC telemetry повторила synthetic 0.60 -> 0.70 м; MAVLink RangeFinder backend принимает сообщения.\n";
  }else if(fc_ds>0){
    std::cout<<"AP_RANGEFINDER_MAVLINK_INGRESS=AMBIGUOUS\n";
    std::cout<<"FC DISTANCE_SENSOR есть, но последняя ступень 0.70 м не подтверждена.\n";
  }else{
    std::cout<<"AP_RANGEFINDER_MAVLINK_INGRESS=NOT_PROVEN\n";
    std::cout<<"FC не публикует DISTANCE_SENSOR из backend. Проверить RNGFND1_TYPE/ORIENT и MAVLink ingress.\n";
  }
  ::close(fd);return 0;
}
