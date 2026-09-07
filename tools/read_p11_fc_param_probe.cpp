#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include "common/mavlink.h"

static int open_serial(){
  int f=open("/dev/ttyAMA0",O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(f<0){perror("/dev/ttyAMA0");return -1;}
  termios t{}; tcgetattr(f,&t); cfmakeraw(&t);
  cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
  t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CRTSCTS;
  tcsetattr(f,TCSANOW,&t);
  return f;
}

static void send_msg(int f,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  uint16_t n=mavlink_msg_to_send_buffer(b,&m);
  if(write(f,b,n)<0) perror("write");
}

static std::string param_name(const mavlink_param_value_t& x){
  char id[17]{};
  std::memcpy(id,x.param_id,16);
  return std::string(id);
}

static bool read_one(int f,uint8_t sys,uint8_t comp,const std::string& name,float& value,uint16_t& count,uint16_t& index){
  mavlink_message_t q{};
  char id[16]{};
  std::strncpy(id,name.c_str(),16);
  mavlink_msg_param_request_read_pack(
      255,190,&q,sys,comp,id,-1);
  send_msg(f,q);

  mavlink_message_t m{};
  mavlink_status_t st{};
  for(int wait=0;wait<30;wait++){
    pollfd p{f,POLLIN,0};
    int pr=poll(&p,1,100);
    if(pr<=0) continue;
    uint8_t b[4096];
    int n=read(f,b,sizeof(b));
    for(int i=0;i<n;i++){
      if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)) continue;
      if(m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;
      mavlink_param_value_t x{};
      mavlink_msg_param_value_decode(&m,&x);
      std::string got=param_name(x);
      std::cout<<"  PARAM_VALUE: "<<got<<" = "<<std::setprecision(10)<<x.param_value
               <<"  index="<<x.param_index<<" count="<<x.param_count<<"\n";
      if(got==name){
        value=x.param_value;
        count=x.param_count;
        index=x.param_index;
        return true;
      }
    }
  }
  return false;
}

int main(){
  std::cout<<"================ P11 PARAM_REQUEST_READ PROBE ================\n";
  std::cout<<"Режим: ТОЛЬКО ЧТЕНИЕ. PARAM_SET не используется.\n";

  int f=open_serial();
  if(f<0) return 2;

  mavlink_message_t m{};
  mavlink_status_t st{};
  uint8_t sys=0,comp=0;
  std::cout<<"[MAV] ожидание HEARTBEAT...\n";
  while(!sys){
    pollfd p{f,POLLIN,0}; poll(&p,1,100);
    uint8_t b[2048];
    int n=read(f,b,sizeof(b));
    for(int i=0;i<n;i++){
      if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st) &&
         m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
        sys=m.sysid; comp=m.compid; break;
      }
    }
  }
  std::cout<<"[MAV] FC найден: sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";

  const char* probes[]={
    "AHRS_ORIENTATION",
    "INS_ACC_ID",
    "INS_ACC2_ID"
  };

  int ok=0;
  for(const char* name:probes){
    std::cout<<"\n[ЗАПРОС] "<<name<<"\n";
    float v=0; uint16_t cnt=0,idx=0;
    if(read_one(f,sys,comp,name,v,cnt,idx)){
      std::cout<<"[ОТВЕТ] "<<name<<" = "<<std::setprecision(10)<<v
               <<"  index="<<idx<<" count="<<cnt<<"\n";
      ok++;
    }else{
      std::cout<<"[НЕТ ОТВЕТА] "<<name<<"\n";
    }
  }

  close(f);
  std::cout<<"\n================ ИТОГ ================\n";
  std::cout<<"Ответов: "<<ok<<" / 3\n";
  if(ok>0){
    std::cout<<"PARAM_REQUEST_READ работает на этом MAVLink-канале.\n";
    std::cout<<"Проблема предыдущего шага специфична для PARAM_REQUEST_LIST/полного списка.\n";
  }else{
    std::cout<<"Нет ни одного PARAM_VALUE даже на одиночные запросы.\n";
    std::cout<<"Следующий шаг: проверять параметрический MAVLink-сервис/канал, а не фильтр имён.\n";
  }
  std::cout<<"FC не изменён.\n";
  return ok>0?0:3;
}
