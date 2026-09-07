#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include "common/mavlink.h"

static int open_serial(){
  int f=open("/dev/ttyAMA0",O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(f<0){perror("/dev/ttyAMA0");return -1;}
  termios t{}; tcgetattr(f,&t); cfmakeraw(&t);
  cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
  t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CRTSCTS;
  tcsetattr(f,TCSANOW,&t); return f;
}
static void send_msg(int f,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  uint16_t n=mavlink_msg_to_send_buffer(b,&m);
  if(write(f,b,n)<0) perror("write");
}
static bool wanted(const std::string& n){
  static const char* p[]={"INS_ACCSCAL_","INS_ACCOFFS_","INS_ACC2SCAL_","INS_ACC2OFFS_",
    "INS_ACC3SCAL_","INS_ACC3OFFS_","INS_ACC_ID","INS_ACC2_ID","INS_ACC3_ID",
    "AHRS_ORIENTATION","AHRS_TRIM_X","AHRS_TRIM_Y","AHRS_TRIM_Z"};
  for(auto x:p) if(n==x || n.rfind(x,0)==0) return true;
  return false;
}
int main(){
  int f=open_serial(); if(f<0) return 2;
  mavlink_message_t m{}; mavlink_status_t st{}; uint8_t sys=0,comp=0;
  std::cout<<"================ P11 ПАРАМЕТРЫ АКСЕЛЕРОМЕТРОВ FC ================\n";
  std::cout<<"Режим: ТОЛЬКО ЧТЕНИЕ. Параметры FC не изменяются.\n";
  std::cout<<"[MAV] ожидание HEARTBEAT...\n";
  while(!sys){
    pollfd p{f,POLLIN,0}; poll(&p,1,100);
    uint8_t b[2048]; int n=read(f,b,sizeof(b));
    for(int i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      sys=m.sysid; comp=m.compid; break;
    }
  }
  std::cout<<"[MAV] FC найден: sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";
  mavlink_message_t q{};
  mavlink_msg_param_request_list_pack(255,190,&q,sys,comp);
  send_msg(f,q);
  std::map<std::string,float> vals; int expected=-1, seen=0, idle=0;
  while(idle<30){
    pollfd p{f,POLLIN,0}; int pr=poll(&p,1,100);
    bool got=false;
    if(pr>0){
      uint8_t b[8192]; int n=read(f,b,sizeof(b));
      for(int i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_PARAM_VALUE){
        mavlink_param_value_t x{}; mavlink_msg_param_value_decode(&m,&x);
        char id[17]{}; std::memcpy(id,x.param_id,16); std::string name(id);
        expected=x.param_count; seen++;
        if(wanted(name)) vals[name]=x.param_value;
        got=true;
      }
    }
    idle=got?0:idle+1;
    if(expected>0 && seen>=expected) break;
  }
  close(f);
  std::cout<<"\n================ КЛЮЧЕВЫЕ ГРУППЫ ================\n";
  if(vals.empty()) std::cout<<"Нужные параметры в полученном PARAM_VALUE не найдены.\n";
  else for(const auto& [k,v]:vals) std::cout<<std::left<<std::setw(22)<<k<<" = "<<std::setprecision(10)<<v<<"\n";
  std::cout<<"\nПолучено PARAM_VALUE: "<<seen;
  if(expected>0) std::cout<<" / объявлено FC: "<<expected;
  std::cout<<"\n";
  if(expected>0 && seen<expected) std::cout<<"ПРЕДУПРЕЖДЕНИЕ: список параметров получен не полностью. Повторите чтение; FC не изменён.\n";
  std::cout<<"===============================================================\n";
  return 0;
}
