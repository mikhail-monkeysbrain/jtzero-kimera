// JT-Zero: проверка ExternalNav position aiding без физического движения.
// Требует EK3_SRC1_POSXY=6 и EK3_SRC1_VELXY=6.
// БПЛА должен оставаться неподвижным и DISARMED.
// Синтетический профиль NED North согласован по позиции и скорости:
// 0..3 c: x=0, v=0
// 3..7 c: x 0->0.4 m, v=+0.1 m/s
// 7..11 c: x 0.4->0 m, v=-0.1 m/s
// 11..14 c: x=0, v=0

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "common/mavlink.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr uint8_t kSelfSys = 191;
constexpr uint8_t kSelfComp = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

uint64_t monoUs(){return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();}
double elapsed(const Clock::time_point&t0){return std::chrono::duration<double>(Clock::now()-t0).count();}

bool writeMsg(int fd,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; const uint16_t n=mavlink_msg_to_send_buffer(b,&m); size_t o=0;
  while(o<n){ssize_t k=::write(fd,b+o,n-o);if(k>0){o+=(size_t)k;continue;}if(k<0&&errno==EINTR)continue;if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd p{fd,POLLOUT,0};(void)::poll(&p,1,10);continue;}return false;}return true;
}

int openSerial(const std::string&dev){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)return -1;termios t{};if(tcgetattr(fd,&t)<0){::close(fd);return -1;}cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;}tcflush(fd,TCIFLUSH);return fd;
}

void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)msgid,1000000.0f/hz,0,0,0,0,0);(void)writeMsg(fd,m);}

void profile(double t,float&x,float&v,const char*&name){
  if(t<3.0){x=0;v=0;name="ZERO_PRE";return;}
  if(t<7.0){double u=t-3.0;x=(float)(0.1*u);v=0.1f;name="FORWARD";return;}
  if(t<11.0){double u=t-7.0;x=(float)(0.4-0.1*u);v=-0.1f;name="BACK";return;}
  x=0;v=0;name="ZERO_POST";
}

void sendPose(int fd,float x){
  float cov[21]{};
  cov[0]=0.04f; cov[6]=0.04f; cov[11]=1.0f; cov[15]=1.0f; cov[18]=1.0f; cov[20]=1.0f;
  mavlink_message_t m{};
  mavlink_msg_vision_position_estimate_pack(kSelfSys,kSelfComp,&m,monoUs(),x,0.0f,0.0f,0.0f,0.0f,0.0f,cov,0);
  (void)writeMsg(fd,m);
}

void sendSpeed(int fd,float vn){
  float cov[9]={0.01f,0,0,0,0.01f,0,0,0,0.01f};
  mavlink_message_t m{};
  mavlink_msg_vision_speed_estimate_pack(kSelfSys,kSelfComp,&m,monoUs(),vn,0.0f,0.0f,cov,0);
  (void)writeMsg(fd,m);
}
}

int main(int argc,char**argv){
  const std::string dev=(argc>=2)?argv[1]:"/dev/ttyAMA0";
  int fd=openSerial(dev);if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}
  std::cerr<<"JT-ZERO ExternalNav position stationary test\nFC="<<dev<<"\nВАЖНО: EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=6, БПЛА НЕ ДВИГАТЬ И НЕ ARM.\n";

  mavlink_status_t st{};mavlink_message_t msg{};uint8_t buf[4096];uint8_t sys=0,comp=0;auto hb0=Clock::now();
  while(elapsed(hb0)<8.0&&!sys){pollfd p{fd,POLLIN,0};if(::poll(&p,1,100)<=0)continue;ssize_t n=::read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}}}
  if(!sys){std::cerr<<"ОШИБКА: heartbeat ArduPilot не найден\n";::close(fd);return 3;}
  std::cerr<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,20);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_GLOBAL_POSITION_INT,20);

  uint64_t local_count=0;float last_x=0,last_vx=0;bool have_local=false;auto t0=Clock::now(),next_tx=t0,next_print=t0;std::string last_phase;
  while(true){
    double t=elapsed(t0);if(t>=14.0)break;float cmd_x=0,cmd_v=0;const char*phase=nullptr;profile(t,cmd_x,cmd_v,phase);
    if(last_phase!=phase){std::cerr<<"\nPHASE "<<phase<<" cmd_x="<<cmd_x<<" cmd_vN="<<cmd_v<<"\n";last_phase=phase;}
    auto now=Clock::now();if(now>=next_tx){sendPose(fd,cmd_x);sendSpeed(fd,cmd_v);next_tx+=std::chrono::milliseconds(50);}

    pollfd p{fd,POLLIN,0};if(::poll(&p,1,5)>0){for(;;){ssize_t n=::read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;if(msg.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED&&msg.sysid==sys){mavlink_local_position_ned_t lp{};mavlink_msg_local_position_ned_decode(&msg,&lp);last_x=lp.x;last_vx=lp.vx;have_local=true;++local_count;}}}}

    if(now>=next_print){std::cout<<"t="<<t<<" phase="<<phase<<" cmd_x="<<cmd_x<<" cmd_vN="<<cmd_v<<" LOCAL rx="<<local_count;if(have_local)std::cout<<" x="<<last_x<<" vx="<<last_vx;std::cout<<"\n";next_print+=std::chrono::seconds(1);}
  }
  for(int i=0;i<10;i++){sendPose(fd,0.0f);sendSpeed(fd,0.0f);usleep(50000);}
  std::cout<<"\n=== RESULT ===\nLOCAL_POSITION_NED rx="<<local_count;if(have_local)std::cout<<" final_x="<<last_x<<" final_vx="<<last_vx;std::cout<<"\n";
  ::close(fd);return 0;
}
