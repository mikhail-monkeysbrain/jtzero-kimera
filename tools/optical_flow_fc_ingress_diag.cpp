// JT-Zero — минимальная проверка MAVLink OpticalFlow ingress внутри ArduPilot.
//
// Ничего не двигает и не использует камеру. Отправляет почти нулевой high-precision
// OPTICAL_FLOW в FC и одновременно запрашивает у самого FC OPTICAL_FLOW telemetry,
// EKF_STATUS_REPORT и несколько ключевых параметров.
//
// Если FC возвращает OPTICAL_FLOW от sysid автопилота, AP_OpticalFlow backend
// существует, healthy и реально принял MAVLink flow. Это отделяет ingress от EKF gate.

#include "ardupilotmega/mavlink.h"

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
using Clock = std::chrono::steady_clock;
constexpr uint8_t kSelfSys = 191;
constexpr uint8_t kSelfComp = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

uint64_t monoUs(){
  return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();
}
double secSince(const Clock::time_point& t0){
  return std::chrono::duration<double>(Clock::now()-t0).count();
}

int openSerial(const std::string& dev){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd<0)return -1;
  termios t{};
  if(tcgetattr(fd,&t)<0){::close(fd);return -1;}
  cfmakeraw(&t);
  cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
  t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
  t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;}
  tcflush(fd,TCIFLUSH);
  return fd;
}

bool writeMsg(int fd,const mavlink_message_t& m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n=mavlink_msg_to_send_buffer(b,&m);
  size_t o=0;
  while(o<n){
    const ssize_t k=::write(fd,b+o,n-o);
    if(k>0){o+=(size_t)k;continue;}
    if(k<0&&errno==EINTR)continue;
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
      pollfd p{fd,POLLOUT,0}; (void)::poll(&p,1,10); continue;
    }
    return false;
  }
  return true;
}

void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
  mavlink_message_t m{};
  mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
    MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);
  (void)writeMsg(fd,m);
}

void requestParam(int fd,uint8_t sys,uint8_t comp,const std::string& name){
  char id[16]{};
  std::memcpy(id,name.c_str(),std::min<size_t>(name.size(),sizeof(id)));
  mavlink_message_t m{};
  mavlink_msg_param_request_read_pack(kSelfSys,kSelfComp,&m,sys,comp,id,-1);
  (void)writeMsg(fd,m);
}

void sendFlow(int fd,float rx,float ry,uint8_t quality){
  mavlink_message_t m{};
  mavlink_msg_optical_flow_pack(kSelfSys,kSelfComp,&m,monoUs(),
    0,0,0,0.0f,0.0f,quality,-1.0f,rx,ry);
  (void)writeMsg(fd,m);
}

std::string paramName(const mavlink_param_value_t& p){
  size_t n=0; while(n<sizeof(p.param_id)&&p.param_id[n])++n;
  return std::string(p.param_id,p.param_id+n);
}

std::string flagsText(uint16_t f){
  std::string s;
  auto add=[&](const char* n,bool v){if(!s.empty())s+=' ';s+=n;s+='=';s+=(v?'1':'0');};
  add("att",f&1); add("velH",f&2); add("velV",f&4); add("posRel",f&8);
  add("posAbs",f&16); add("posVAbs",f&32); add("posVAGL",f&64);
  add("constPos",f&128); add("predRel",f&256); add("predAbs",f&512); add("uninit",f&1024);
  return s;
}
}

int main(int argc,char** argv){
  const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
  int fd=openSerial(dev);
  if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}

  mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t buf[4096];
  uint8_t sys=0,comp=0;
  const auto wait0=Clock::now();
  while(secSince(wait0)<8.0&&!sys){
    pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0)continue;
    const ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0)continue;
    for(ssize_t i=0;i<n;i++){
      if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
      if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
      mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&msg,&hb);
      if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
    }
  }
  if(!sys){std::cerr<<"ОШИБКА: heartbeat ArduPilot не найден\n";::close(fd);return 3;}
  std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  requestRate(fd,sys,comp,MAVLINK_MSG_ID_OPTICAL_FLOW,10);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,10);

  const char* params[]={"FLOW_TYPE","FLOW_OPTIONS","EK3_FLOW_USE","EK3_SRC1_VELXY","EK3_PRIMARY","AHRS_EKF_TYPE"};
  for(const char* n:params)requestParam(fd,sys,comp,n);

  // Один ненулевой quality=0 пакет переводит AP_OpticalFlow_MAV в flow_rate_x/y mode.
  sendFlow(fd,1.0e-6f,0.0f,0);

  uint64_t tx=0,fc_of=0,ekf_n=0,local_n=0;
  mavlink_optical_flow_t last_of{};
  mavlink_ekf_status_report_t last_ekf{};
  mavlink_local_position_ned_t last_local{};
  std::map<std::string,float> pv;

  const auto t0=Clock::now();
  auto next_tx=Clock::now();
  auto next_print=Clock::now()+std::chrono::seconds(1);
  while(secSince(t0)<7.0){
    const auto now=Clock::now();
    if(now>=next_tx){
      // Нулевой flow с max quality: стационарный synthetic probe.
      sendFlow(fd,0.0f,0.0f,255); ++tx;
      next_tx += std::chrono::milliseconds(20); // 50 Hz
    }

    pollfd p{fd,POLLIN,0};
    if(::poll(&p,1,5)>0){
      for(;;){
        const ssize_t n=::read(fd,buf,sizeof(buf));
        if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
        if(n<=0)break;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
          if(msg.sysid!=sys)continue;
          if(msg.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){
            mavlink_msg_optical_flow_decode(&msg,&last_of); ++fc_of;
          } else if(msg.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
            mavlink_msg_ekf_status_report_decode(&msg,&last_ekf); ++ekf_n;
          } else if(msg.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){
            mavlink_msg_local_position_ned_decode(&msg,&last_local); ++local_n;
          } else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE){
            mavlink_param_value_t q{}; mavlink_msg_param_value_decode(&msg,&q);
            const std::string n=paramName(q);
            for(const char* wanted:params)if(n==wanted)pv[n]=q.param_value;
          }
        }
      }
    }

    if(now>=next_print){
      std::cout<<std::fixed<<std::setprecision(4)
               <<"t="<<secSince(t0)<<"s tx="<<tx<<" FC_OF="<<fc_of;
      if(fc_of){
        std::cout<<" q="<<(int)last_of.quality
                 <<" rate=("<<last_of.flow_rate_x<<","<<last_of.flow_rate_y<<")"
                 <<" comp=("<<last_of.flow_comp_m_x<<","<<last_of.flow_comp_m_y<<")";
      }
      std::cout<<" EKF="<<ekf_n;
      if(ekf_n)std::cout<<" flags=0x"<<std::hex<<last_ekf.flags<<std::dec<<" ["<<flagsText(last_ekf.flags)<<"]";
      std::cout<<" LOCAL="<<local_n;
      if(local_n)std::cout<<" pN/E=("<<last_local.x<<","<<last_local.y<<") vN/E=("<<last_local.vx<<","<<last_local.vy<<")";
      std::cout<<"\n";
      next_print += std::chrono::seconds(1);
    }
  }

  std::cout<<"\n===== PARAMS =====\n";
  for(const char* n:params){
    auto it=pv.find(n);
    if(it==pv.end())std::cout<<n<<" = NO_RESPONSE\n";
    else std::cout<<n<<" = "<<it->second<<"\n";
  }

  std::cout<<"\n===== VERDICT =====\n";
  if(fc_of>0){
    std::cout<<"FC_OPTICAL_FLOW_ECHO=YES: ArduPilot AP_OpticalFlow backend существует, healthy и принимает MAVLink flow.\n";
  } else {
    std::cout<<"FC_OPTICAL_FLOW_ECHO=NO: ingress/health backend пока НЕ доказан.\n";
  }
  if(ekf_n>0)std::cout<<"EKF_STATUS flags=0x"<<std::hex<<last_ekf.flags<<std::dec<<" ["<<flagsText(last_ekf.flags)<<"]\n";
  std::cout<<"tx_flow="<<tx<<" fc_of="<<fc_of<<" ekf="<<ekf_n<<" local="<<local_n<<"\n";
  ::close(fd);
  return 0;
}
