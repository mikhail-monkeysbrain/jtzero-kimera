// JT-Zero — реальный TF-Luna -> RPi -> MAVLink DISTANCE_SENSOR -> FC RangeFinder echo.
// Камера не используется. Параметры FC не меняются.
// 10 секунд читает TF-Luna на /dev/ttyAMA2, публикует вниз через DISTANCE_SENSOR,
// и слушает DISTANCE_SENSOR от самого FC.

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
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace {
using Clock=std::chrono::steady_clock;
constexpr uint8_t kSelfSys=191;
constexpr uint8_t kSelfComp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

double elapsed(const Clock::time_point&t0){return std::chrono::duration<double>(Clock::now()-t0).count();}

int openSerial(const std::string&dev,int baud){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK); if(fd<0)return -1;
  termios t{}; if(tcgetattr(fd,&t)<0){::close(fd);return -1;} cfmakeraw(&t);
  speed_t sp=B115200;
  if(baud==460800)sp=B460800;
  cfsetispeed(&t,sp); cfsetospeed(&t,sp); t.c_cflag|=CLOCAL|CREAD;
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
void sendDistance(int fd,double m){
  const uint16_t cm=(uint16_t)std::lround(m*100.0);
  float q[4]={0,0,0,0};
  mavlink_message_t msg{};
  mavlink_msg_distance_sensor_pack(
    kSelfSys,kSelfComp,&msg,0,
    10,700,cm,MAV_DISTANCE_SENSOR_LASER,
    0,MAV_SENSOR_ROTATION_PITCH_270,0,
    0.0f,0.0f,q,0);
  (void)writeMsg(fd,msg);
}

struct LunaParser {
  uint8_t b[9]{}; int n=0;
  bool feed(uint8_t x,double &m,uint16_t &strength){
    if(n==0 && x!=0x59)return false;
    if(n==1 && x!=0x59){n=0; return false;}
    b[n++]=x;
    if(n<9)return false;
    n=0;
    uint8_t sum=0;for(int i=0;i<8;i++)sum=(uint8_t)(sum+b[i]);
    if(sum!=b[8])return false;
    uint16_t cm=(uint16_t)b[2] | ((uint16_t)b[3]<<8);
    strength=(uint16_t)b[4] | ((uint16_t)b[5]<<8);
    m=cm*0.01;
    return cm>0;
  }
};
}

int main(int argc,char**argv){
  const std::string luna=(argc>1)?argv[1]:"/dev/ttyAMA2";
  const std::string fcdev=(argc>2)?argv[2]:"/dev/ttyAMA0";
  int lf=openSerial(luna,115200); if(lf<0){std::cerr<<"ОШИБКА open Luna "<<luna<<": "<<std::strerror(errno)<<"\n";return 2;}
  int ff=openSerial(fcdev,460800); if(ff<0){std::cerr<<"ОШИБКА open FC "<<fcdev<<": "<<std::strerror(errno)<<"\n";return 3;}

  mavlink_status_t st{};mavlink_message_t msg{};uint8_t buf[4096];uint8_t sys=0,comp=0;
  const auto w0=Clock::now();
  while(elapsed(w0)<8&&!sys){
    pollfd p{ff,POLLIN,0};if(::poll(&p,1,100)<=0)continue;ssize_t n=::read(ff,buf,sizeof(buf));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);
      if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
    }
  }
  if(!sys){std::cerr<<"ОШИБКА: FC heartbeat не найден\n";return 4;}
  std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
  requestRate(ff,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,10);

  LunaParser lp; double luna_m=NAN,fc_m=NAN; uint16_t strength=0;
  uint64_t luna_frames=0,tx=0,fc_echo=0; int64_t last_luna_ns=0;
  auto ns=[](){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();};
  const auto t0=Clock::now(); auto next_send=Clock::now(); auto next_print=Clock::now()+std::chrono::seconds(1);

  while(elapsed(t0)<10.0){
    pollfd ps[2]={{lf,POLLIN,0},{ff,POLLIN,0}};
    (void)::poll(ps,2,5);
    if(ps[0].revents&POLLIN){
      uint8_t x[512];ssize_t n=::read(lf,x,sizeof(x));
      for(ssize_t i=0;i<n;i++){double m;uint16_t s;if(lp.feed(x[i],m,s)){luna_m=m;strength=s;++luna_frames;last_luna_ns=ns();}}
    }
    if(ps[1].revents&POLLIN){
      for(;;){ssize_t n=::read(ff,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
        for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)&&msg.sysid==sys&&msg.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){
          mavlink_distance_sensor_t q{};mavlink_msg_distance_sensor_decode(&msg,&q);
          if(q.orientation==MAV_SENSOR_ROTATION_PITCH_270){fc_m=q.current_distance*0.01;++fc_echo;}
        }
      }
    }
    auto now=Clock::now();
    if(now>=next_send && std::isfinite(luna_m) && (ns()-last_luna_ns)<200000000LL){
      sendDistance(ff,luna_m);++tx;next_send+=std::chrono::milliseconds(50);
    }
    if(now>=next_print){
      std::cout<<std::fixed<<std::setprecision(3)
        <<"t="<<elapsed(t0)<<"s luna="<<(std::isfinite(luna_m)?luna_m:-1.0)<<"m"
        <<" strength="<<strength<<" luna_frames="<<luna_frames
        <<" tx="<<tx<<" FC_echo="<<fc_echo
        <<" fc="<<(std::isfinite(fc_m)?fc_m:-1.0)<<"m";
      if(std::isfinite(luna_m)&&std::isfinite(fc_m))std::cout<<" diff="<<(fc_m-luna_m)*1000.0<<"mm";
      std::cout<<"\n"; next_print+=std::chrono::seconds(1);
    }
  }

  std::cout<<"\n===== VERDICT =====\n";
  std::cout<<"LUNA_FRAMES="<<luna_frames<<"\nTX_DISTANCE_SENSOR="<<tx<<"\nFC_DISTANCE_SENSOR_ECHO="<<fc_echo<<"\n";
  if(luna_frames>20 && tx>20 && fc_echo>5 && std::isfinite(luna_m)&&std::isfinite(fc_m)&&std::fabs(fc_m-luna_m)<0.03){
    std::cout<<"REAL_LUNA_TO_FC_RANGEFINDER=PROVEN\n";
    std::cout<<"TF-Luna -> RPi -> MAVLink DISTANCE_SENSOR -> AP_RangeFinder_MAVLink подтвержден.\n";
  } else {
    std::cout<<"REAL_LUNA_TO_FC_RANGEFINDER=NOT_PROVEN\n";
  }
  ::close(lf);::close(ff);return 0;
}
