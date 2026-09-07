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
#include <vector>
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

static std::string pname(const mavlink_param_value_t& x){
  char id[17]{};
  std::memcpy(id,x.param_id,16);
  return std::string(id);
}

static void request_named(int f,uint8_t sys,uint8_t comp,const std::string& name){
  mavlink_message_t q{};
  char id[16]{};
  std::strncpy(id,name.c_str(),15);
  mavlink_msg_param_request_read_pack(255,190,&q,sys,comp,id,-1);
  send_msg(f,q);
}

int main(){
  const std::vector<std::string> names={
    "AHRS_ORIENTATION","AHRS_TRIM_X","AHRS_TRIM_Y","AHRS_TRIM_Z",
    "INS_ACC_ID","INS_ACC2_ID","INS_ACC3_ID",
    "INS_ACCSCAL_X","INS_ACCSCAL_Y","INS_ACCSCAL_Z",
    "INS_ACCOFFS_X","INS_ACCOFFS_Y","INS_ACCOFFS_Z",
    "INS_ACC2SCAL_X","INS_ACC2SCAL_Y","INS_ACC2SCAL_Z",
    "INS_ACC2OFFS_X","INS_ACC2OFFS_Y","INS_ACC2OFFS_Z",
    "INS_ACC3SCAL_X","INS_ACC3SCAL_Y","INS_ACC3SCAL_Z",
    "INS_ACC3OFFS_X","INS_ACC3OFFS_Y","INS_ACC3OFFS_Z"
  };

  std::set<std::string> wanted(names.begin(),names.end());
  std::map<std::string,float> got;
  std::map<std::string,int> attempts;
  std::map<std::string,uint16_t> pcount,pindex;

  std::cout<<"================ P11 FC ACCEL PARAM COLLECTOR ================\n";
  std::cout<<"Режим: ТОЛЬКО ЧТЕНИЕ. PARAM_SET не используется.\n";

  int f=open_serial();
  if(f<0) return 2;

  mavlink_message_t m{};
  mavlink_status_t st{};
  uint8_t sys=0,comp=0;
  std::cout<<"[MAV] ожидание HEARTBEAT...\n";
  while(!sys){
    pollfd p{f,POLLIN,0}; poll(&p,1,100);
    uint8_t b[2048]; int n=read(f,b,sizeof(b));
    for(int i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      sys=m.sysid; comp=m.compid; break;
    }
  }
  std::cout<<"[MAV] FC найден: sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";

  // Several rounds. Replies may be delayed/out-of-order, so we collect by name globally.
  for(int round=1; round<=5 && got.size()<wanted.size(); ++round){
    std::cout<<"\n[РАУНД "<<round<<"] отсутствует "<<(wanted.size()-got.size())<<" параметров\n";
    for(const auto& name:names){
      if(got.count(name)) continue;
      request_named(f,sys,comp,name);
      attempts[name]++;
      usleep(30000);
    }

    int idle=0;
    while(idle<25){
      pollfd p{f,POLLIN,0};
      int pr=poll(&p,1,100);
      bool any=false;
      if(pr>0){
        uint8_t b[8192]; int n=read(f,b,sizeof(b));
        for(int i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)) continue;
          if(m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;
          mavlink_param_value_t x{};
          mavlink_msg_param_value_decode(&m,&x);
          std::string name=pname(x);
          any=true;
          if(wanted.count(name)){
            bool first=!got.count(name);
            got[name]=x.param_value;
            pcount[name]=x.param_count;
            pindex[name]=x.param_index;
            if(first) std::cout<<"  [ПОЛУЧЕН] "<<std::left<<std::setw(18)<<name
                               <<" = "<<std::setprecision(10)<<x.param_value
                               <<" index="<<x.param_index<<" count="<<x.param_count<<"\n";
          }
        }
      }
      idle=any?0:idle+1;
      if(got.size()==wanted.size()) break;
    }
  }

  close(f);

  std::cout<<"\n================ РЕЗУЛЬТАТ ================\n";
  for(const auto& name:names){
    std::cout<<std::left<<std::setw(18)<<name<<" = ";
    auto it=got.find(name);
    if(it==got.end()) std::cout<<"<НЕТ ОТВЕТА>";
    else std::cout<<std::setprecision(10)<<it->second;
    std::cout<<"   attempts="<<attempts[name]<<"\n";
  }

  std::cout<<"\nПолучено: "<<got.size()<<" / "<<wanted.size()<<"\n";
  std::cout<<"FC не изменён.\n";
  return got.empty()?3:0;
}
