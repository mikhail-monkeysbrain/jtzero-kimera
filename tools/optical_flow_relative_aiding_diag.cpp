// JT-Zero — диагностика перехода EKF3 из AID_NONE в AID_RELATIVE по OpticalFlow.
//
// Камера и TF-Luna не используются. Тест:
//  1) принудительно выбирает EKF source set 1 через MAV_CMD_SET_EKF_SOURCE_SET;
//  2) отправляет synthetic high-precision OPTICAL_FLOW 50 Гц;
//  3) следит за EKF_STATUS_REPORT, LOCAL_POSITION_NED, OPTICAL_FLOW echo и STATUSTEXT;
//  4) печатает ключевые EK3 source-параметры всех трёх source sets.
//
// Никакие параметры не записываются. Выбор source set меняется только runtime-командой.

#include "ardupilotmega/mavlink.h"

#include <cerrno>
#include <chrono>
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

uint64_t monoUs(){return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();}
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
  while(o<n){
    ssize_t k=::write(fd,b+o,n-o); if(k>0){o+=(size_t)k;continue;} if(k<0&&errno==EINTR)continue;
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd p{fd,POLLOUT,0};(void)::poll(&p,1,10);continue;}
    return false;
  }
  return true;
}

void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
  mavlink_message_t m{}; mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
    MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0); (void)writeMsg(fd,m);
}

void requestParam(int fd,uint8_t sys,uint8_t comp,const char*name){
  char id[16]{}; std::memcpy(id,name,std::min<size_t>(std::strlen(name),sizeof(id)));
  mavlink_message_t m{}; mavlink_msg_param_request_read_pack(kSelfSys,kSelfComp,&m,sys,comp,id,-1); (void)writeMsg(fd,m);
}

void forcePrimarySourceSet(int fd,uint8_t sys,uint8_t comp){
  mavlink_message_t m{};
  mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
    MAV_CMD_SET_EKF_SOURCE_SET,0,
    1.0f,0,0,0,0,0,0); // param1=1 => PRIMARY source set
  (void)writeMsg(fd,m);
}

void sendFlow(int fd,float rx,float ry,uint8_t quality){
  mavlink_message_t m{};
  mavlink_msg_optical_flow_pack(kSelfSys,kSelfComp,&m,monoUs(),0,0,0,0.0f,0.0f,quality,-1.0f,rx,ry);
  (void)writeMsg(fd,m);
}

std::string paramName(const mavlink_param_value_t&p){
  size_t n=0; while(n<sizeof(p.param_id)&&p.param_id[n])++n; return std::string(p.param_id,p.param_id+n);
}

std::string flagsText(uint16_t f){
  std::string s; auto a=[&](const char*n,bool v){if(!s.empty())s+=' ';s+=n;s+='=';s+=(v?'1':'0');};
  a("att",f&1);a("velH",f&2);a("velV",f&4);a("posRel",f&8);a("posAbs",f&16);a("posVAbs",f&32);
  a("posVAGL",f&64);a("constPos",f&128);a("predRel",f&256);a("predAbs",f&512);a("uninit",f&1024); return s;
}

std::string statustextText(const mavlink_statustext_t&q){
  size_t n=0; while(n<sizeof(q.text)&&q.text[n])++n; return std::string(q.text,q.text+n);
}
}

int main(int argc,char**argv){
  const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
  const double duration=(argc>2)?std::stod(argv[2]):60.0;
  if(!(duration>=10.0&&duration<=180.0)){std::cerr<<"ОШИБКА: duration должен быть 10..180 секунд\n";return 2;}

  int fd=openSerial(dev); if(fd<0){std::cerr<<"ОШИБКА: open "<<dev<<": "<<std::strerror(errno)<<"\n";return 3;}
  mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t buf[4096]; uint8_t sys=0,comp=0;

  const auto w0=Clock::now();
  while(elapsed(w0)<8.0&&!sys){
    pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0)continue; ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0)continue;
    for(ssize_t i=0;i<n;i++){
      if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)||msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
      mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&msg,&hb);
      if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
    }
  }
  if(!sys){std::cerr<<"ОШИБКА: heartbeat ArduPilot не найден\n";::close(fd);return 4;}
  std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  requestRate(fd,sys,comp,MAVLINK_MSG_ID_OPTICAL_FLOW,10);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,10);

  const char* params[]={
    "FLOW_TYPE","FLOW_OPTIONS","EK3_FLOW_USE","EK3_SRC_OPTIONS","EK3_PRIMARY","AHRS_EKF_TYPE",
    "EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_POSZ","EK3_SRC1_VELZ","EK3_SRC1_YAW",
    "EK3_SRC2_POSXY","EK3_SRC2_VELXY","EK3_SRC2_POSZ","EK3_SRC2_VELZ","EK3_SRC2_YAW",
    "EK3_SRC3_POSXY","EK3_SRC3_VELXY","EK3_SRC3_POSZ","EK3_SRC3_VELZ","EK3_SRC3_YAW"
  };
  for(const char*n:params)requestParam(fd,sys,comp,n);

  forcePrimarySourceSet(fd,sys,comp);
  std::cout<<"RUNTIME: отправлен MAV_CMD_SET_EKF_SOURCE_SET param1=1 (PRIMARY)\n";

  // Переводим MAV backend в high-precision flow_rate mode.
  sendFlow(fd,1.0e-6f,0.0f,0);

  uint64_t tx=0,fc_of=0,ekf_n=0,local_n=0; bool source_ack=false; uint8_t source_ack_result=255;
  mavlink_ekf_status_report_t last_ekf{}; mavlink_local_position_ned_t last_local{}; std::map<std::string,float> pv;
  uint16_t prev_flags=0xFFFF; bool ever_relative=false;

  const auto t0=Clock::now(); auto next_tx=Clock::now(); auto next_print=Clock::now()+std::chrono::seconds(5);
  while(elapsed(t0)<duration){
    const auto now=Clock::now();
    if(now>=next_tx){sendFlow(fd,0.0f,0.0f,255);++tx;next_tx+=std::chrono::milliseconds(20);}

    pollfd p{fd,POLLIN,0};
    if(::poll(&p,1,5)>0){
      for(;;){
        ssize_t n=::read(fd,buf,sizeof(buf)); if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break; if(n<=0)break;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)||msg.sysid!=sys)continue;
          if(msg.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){++fc_of;}
          else if(msg.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
            mavlink_msg_ekf_status_report_decode(&msg,&last_ekf);++ekf_n;
            if(last_ekf.flags!=prev_flags){
              std::cout<<"EKF_CHANGE t="<<std::fixed<<std::setprecision(3)<<elapsed(t0)<<"s flags=0x"<<std::hex<<last_ekf.flags<<std::dec
                       <<" ["<<flagsText(last_ekf.flags)<<"]\n";
              prev_flags=last_ekf.flags;
            }
            if((last_ekf.flags&8) && !(last_ekf.flags&128))ever_relative=true;
          }
          else if(msg.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){mavlink_msg_local_position_ned_decode(&msg,&last_local);++local_n;}
          else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE){
            mavlink_param_value_t q{};mavlink_msg_param_value_decode(&msg,&q);const std::string n=paramName(q);
            for(const char*w:params)if(n==w)pv[n]=q.param_value;
          }
          else if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK){
            mavlink_command_ack_t q{};mavlink_msg_command_ack_decode(&msg,&q);
            if(q.command==MAV_CMD_SET_EKF_SOURCE_SET){source_ack=true;source_ack_result=q.result;
              std::cout<<"SOURCE_SET_ACK result="<<(int)q.result<<"\n";}
          }
          else if(msg.msgid==MAVLINK_MSG_ID_STATUSTEXT){
            mavlink_statustext_t q{};mavlink_msg_statustext_decode(&msg,&q); const auto s=statustextText(q);
            if(s.find("EKF")!=std::string::npos || s.find("aiding")!=std::string::npos || s.find("flow")!=std::string::npos)
              std::cout<<"STATUSTEXT t="<<std::fixed<<std::setprecision(3)<<elapsed(t0)<<"s sev="<<(int)q.severity<<" "<<s<<"\n";
          }
        }
      }
    }

    if(now>=next_print){
      std::cout<<"t="<<std::fixed<<std::setprecision(1)<<elapsed(t0)<<"s tx="<<tx<<" FC_OF="<<fc_of<<" EKF="<<ekf_n;
      if(ekf_n)std::cout<<" flags=0x"<<std::hex<<last_ekf.flags<<std::dec<<" ["<<flagsText(last_ekf.flags)<<"]";
      std::cout<<" LOCAL="<<local_n<<"\n"; next_print+=std::chrono::seconds(5);
    }
  }

  std::cout<<"\n===== PARAMS =====\n";
  for(const char*n:params){auto it=pv.find(n);std::cout<<n<<" = ";if(it==pv.end())std::cout<<"NO_RESPONSE\n";else std::cout<<it->second<<"\n";}

  std::cout<<"\n===== VERDICT =====\n";
  std::cout<<"SOURCE_SET_PRIMARY_ACK="<<(source_ack?"YES":"NO")<<" result="<<(int)source_ack_result<<"\n";
  std::cout<<"FC_OPTICAL_FLOW_ECHO="<<(fc_of?"YES":"NO")<<"\n";
  std::cout<<"AID_RELATIVE_OBSERVED="<<(ever_relative?"YES":"NO")<<"\n";
  if(ekf_n)std::cout<<"FINAL_EKF flags=0x"<<std::hex<<last_ekf.flags<<std::dec<<" ["<<flagsText(last_ekf.flags)<<"]\n";
  std::cout<<"tx_flow="<<tx<<" fc_of="<<fc_of<<" ekf="<<ekf_n<<" local="<<local_n<<"\n";
  ::close(fd);return 0;
}
